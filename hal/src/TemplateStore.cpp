/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_TAG "cityman.iris.store"

#include "TemplateStore.h"

#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <memory>

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/stringprintf.h>
#include <android-base/unique_fd.h>
#include <openssl/aead.h>
#include <openssl/mem.h>
#include <openssl/rand.h>

#include "Config.h"

namespace cityman::iris {

using android::base::StringPrintf;
using android::base::unique_fd;

namespace {

constexpr uint8_t kSealMagic[4] = {'C', 'I', 'R', 'S'};
constexpr uint8_t kSealVersion = 1;
constexpr size_t kNonceLen = 12;
constexpr uint32_t kEnrollMagic = 0x31454943;  // "CIE1"
constexpr uint32_t kStateMagic = 0x32534943;   // "CIS2"
constexpr size_t kMaxFileSize = 4 << 20;

void putU32(std::vector<uint8_t>* v, uint32_t x) {
    for (int i = 0; i < 4; i++) v->push_back(uint8_t(x >> (8 * i)));
}
void putI64(std::vector<uint8_t>* v, int64_t x) {
    for (int i = 0; i < 8; i++) v->push_back(uint8_t(uint64_t(x) >> (8 * i)));
}
bool getU32(const std::vector<uint8_t>& v, size_t* off, uint32_t* x) {
    if (*off + 4 > v.size()) return false;
    *x = 0;
    for (int i = 0; i < 4; i++) *x |= uint32_t(v[*off + i]) << (8 * i);
    *off += 4;
    return true;
}
bool getI64(const std::vector<uint8_t>& v, size_t* off, int64_t* x) {
    if (*off + 8 > v.size()) return false;
    uint64_t u = 0;
    for (int i = 0; i < 8; i++) u |= uint64_t(v[*off + i]) << (8 * i);
    *x = int64_t(u);
    *off += 8;
    return true;
}

bool readFile(const std::string& path, std::vector<uint8_t>* out) {
    std::string s;
    if (!android::base::ReadFileToString(path, &s)) return false;
    if (s.size() > kMaxFileSize) return false;
    out->assign(s.begin(), s.end());
    return true;
}

}  // namespace

// ------------------------------------------------------------------------------------- Sealer

bool Sealer::init(const std::string& keyPath) {
    std::vector<uint8_t> key;
    if (readFile(keyPath, &key) && key.size() == sizeof(mKey)) {
        memcpy(mKey, key.data(), sizeof(mKey));
        OPENSSL_cleanse(key.data(), key.size());
        mReady = true;
        return true;
    }
    if (access(keyPath.c_str(), F_OK) == 0) {
        // Never silently replace a key: every stored template would become unreadable.
        LOG(ERROR) << keyPath << " exists but is unreadable or malformed";
        return false;
    }
    if (RAND_bytes(mKey, sizeof(mKey)) != 1) return false;
    const std::string tmp = keyPath + ".tmp";
    unique_fd fd(TEMP_FAILURE_RETRY(
            ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0600)));
    if (fd < 0 || !android::base::WriteFully(fd, mKey, sizeof(mKey)) || fsync(fd) != 0 ||
        rename(tmp.c_str(), keyPath.c_str()) != 0) {
        PLOG(ERROR) << "cannot create " << keyPath;
        unlink(tmp.c_str());
        OPENSSL_cleanse(mKey, sizeof(mKey));
        return false;
    }
    LOG(INFO) << "created template sealing key " << keyPath;
    mReady = true;
    return true;
}

bool Sealer::seal(const std::vector<uint8_t>& plain, const std::string& aad,
                  std::vector<uint8_t>* out) const {
    if (!mReady) return false;
    EVP_AEAD_CTX ctx;
    if (!EVP_AEAD_CTX_init(&ctx, EVP_aead_aes_256_gcm(), mKey, sizeof(mKey),
                           EVP_AEAD_DEFAULT_TAG_LENGTH, nullptr)) {
        return false;
    }
    uint8_t nonce[kNonceLen];
    RAND_bytes(nonce, sizeof(nonce));
    out->assign(kSealMagic, kSealMagic + 4);
    out->push_back(kSealVersion);
    out->insert(out->end(), nonce, nonce + kNonceLen);
    const size_t header = out->size();
    out->resize(header + plain.size() + EVP_AEAD_max_overhead(EVP_aead_aes_256_gcm()));
    size_t outLen = 0;
    const bool ok = EVP_AEAD_CTX_seal(&ctx, out->data() + header, &outLen, out->size() - header,
                                      nonce, kNonceLen, plain.data(), plain.size(),
                                      reinterpret_cast<const uint8_t*>(aad.data()), aad.size());
    EVP_AEAD_CTX_cleanup(&ctx);
    if (!ok) return false;
    out->resize(header + outLen);
    return true;
}

bool Sealer::open(const std::vector<uint8_t>& sealed, const std::string& aad,
                  std::vector<uint8_t>* out) const {
    if (!mReady) return false;
    const size_t header = 4 + 1 + kNonceLen;
    if (sealed.size() < header || memcmp(sealed.data(), kSealMagic, 4) != 0 ||
        sealed[4] != kSealVersion) {
        return false;
    }
    EVP_AEAD_CTX ctx;
    if (!EVP_AEAD_CTX_init(&ctx, EVP_aead_aes_256_gcm(), mKey, sizeof(mKey),
                           EVP_AEAD_DEFAULT_TAG_LENGTH, nullptr)) {
        return false;
    }
    out->resize(sealed.size() - header);
    size_t outLen = 0;
    const bool ok = EVP_AEAD_CTX_open(&ctx, out->data(), &outLen, out->size(), sealed.data() + 5,
                                      kNonceLen, sealed.data() + header, sealed.size() - header,
                                      reinterpret_cast<const uint8_t*>(aad.data()), aad.size());
    EVP_AEAD_CTX_cleanup(&ctx);
    if (!ok) {
        out->clear();
        return false;
    }
    out->resize(outLen);
    return true;
}

// ---------------------------------------------------------------------------------- encoding

std::vector<uint8_t> encodeEnrollment(const Enrollment& e) {
    std::vector<uint8_t> v;
    putU32(&v, kEnrollMagic);
    putU32(&v, uint32_t(e.id));
    putU32(&v, uint32_t(e.templates.size()));
    for (const auto& t : e.templates) {
        putU32(&v, uint32_t(t.size()));
        v.insert(v.end(), t.begin(), t.end());
    }
    return v;
}

bool decodeEnrollment(const std::vector<uint8_t>& v, Enrollment* e) {
    size_t off = 0;
    uint32_t magic, id, count;
    if (!getU32(v, &off, &magic) || magic != kEnrollMagic || !getU32(v, &off, &id) ||
        !getU32(v, &off, &count) || count > 2) {
        return false;
    }
    e->id = int32_t(id);
    e->templates.clear();
    for (uint32_t i = 0; i < count; i++) {
        uint32_t len;
        if (!getU32(v, &off, &len) || off + len > v.size()) return false;
        e->templates.emplace_back(v.begin() + off, v.begin() + off + len);
        off += len;
    }
    return off == v.size();
}

// ----------------------------------------------------------------------------- TemplateStore

TemplateStore::TemplateStore(int userId, const Sealer* sealer)
    : mUserId(userId), mSealer(sealer), mDir(StringPrintf(kUserDirFormat, userId)) {}

bool TemplateStore::usable() const {
    struct stat st {};
    return mSealer && mSealer->ready() && stat(mDir.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

std::string TemplateStore::pathFor(int32_t id) const {
    return StringPrintf("%s/iris-%d.tpl", mDir.c_str(), id);
}

std::string TemplateStore::aadFor(const char* kind, int32_t id) const {
    return StringPrintf("cityman-iris|%s|v1|user=%d|id=%d", kind, mUserId, id);
}

bool TemplateStore::writeAtomic(const std::string& path, const std::vector<uint8_t>& data) const {
    const std::string tmp = path + ".tmp";
    unique_fd fd(TEMP_FAILURE_RETRY(
            ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0600)));
    if (fd < 0 || !android::base::WriteFully(fd, data.data(), data.size()) || fsync(fd) != 0) {
        PLOG(ERROR) << "write " << tmp;
        unlink(tmp.c_str());
        return false;
    }
    fd.reset();
    if (rename(tmp.c_str(), path.c_str()) != 0) {
        PLOG(ERROR) << "rename " << tmp;
        unlink(tmp.c_str());
        return false;
    }
    unique_fd dirFd(TEMP_FAILURE_RETRY(::open(mDir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC)));
    if (dirFd >= 0) fsync(dirFd);
    return true;
}

std::vector<int32_t> TemplateStore::enrollmentIds() const {
    std::vector<int32_t> ids;
    std::unique_ptr<DIR, int (*)(DIR*)> d(opendir(mDir.c_str()), closedir);
    if (!d) return ids;
    while (struct dirent* de = readdir(d.get())) {
        int id = 0;
        char tail = 0;
        if (sscanf(de->d_name, "iris-%d.tp%c", &id, &tail) == 2 && tail == 'l' && id > 0 &&
            strlen(de->d_name) == StringPrintf("iris-%d.tpl", id).size()) {
            ids.push_back(id);
        }
    }
    return ids;
}

std::optional<Enrollment> TemplateStore::load(int32_t id) const {
    std::vector<uint8_t> sealed, plain;
    if (!readFile(pathFor(id), &sealed)) return std::nullopt;
    Enrollment e;
    if (!mSealer->open(sealed, aadFor("enrollment", id), &plain) ||
        !decodeEnrollment(plain, &e) || e.id != id) {
        LOG(ERROR) << "enrollment " << id << " of user " << mUserId << " is corrupt or foreign";
        return std::nullopt;
    }
    return e;
}

std::vector<Enrollment> TemplateStore::loadAll() const {
    std::vector<Enrollment> all;
    for (int32_t id : enrollmentIds()) {
        if (auto e = load(id)) all.push_back(std::move(*e));
    }
    return all;
}

bool TemplateStore::save(const Enrollment& e) {
    std::vector<uint8_t> sealed;
    if (!mSealer->seal(encodeEnrollment(e), aadFor("enrollment", e.id), &sealed)) return false;
    return writeAtomic(pathFor(e.id), sealed);
}

bool TemplateStore::remove(int32_t id) {
    const std::string p = pathFor(id);
    if (unlink(p.c_str()) != 0 && errno != ENOENT) {
        PLOG(ERROR) << "unlink " << p;
        return false;
    }
    return true;
}

int32_t TemplateStore::newEnrollmentId() const {
    const auto used = enrollmentIds();
    for (;;) {
        uint32_t r = 0;
        RAND_bytes(reinterpret_cast<uint8_t*>(&r), sizeof(r));
        int32_t id = int32_t(r & 0x7fffffff);
        if (id != 0 && std::find(used.begin(), used.end(), id) == used.end()) return id;
    }
}

UserState TemplateStore::loadState() const {
    UserState s;
    std::vector<uint8_t> sealed, plain;
    const std::string path = mDir + "/iris-state.bin";
    if (!readFile(path, &sealed)) return s;
    size_t off = 0;
    uint32_t magic = 0, failed = 0, perm = 0, features = 0;
    int64_t until = 0, authId = 0;
    if (!mSealer->open(sealed, aadFor("state", 0), &plain) || !getU32(plain, &off, &magic) ||
        magic != kStateMagic || !getU32(plain, &off, &failed) || !getI64(plain, &off, &until) ||
        !getU32(plain, &off, &perm) || !getU32(plain, &off, &features) ||
        !getI64(plain, &off, &authId)) {
        // A tampered or unreadable state must not unlock anything: fail closed.
        LOG(ERROR) << path << " is corrupt: assuming permanent lockout";
        s.permanentLockout = true;
        return s;
    }
    s.failedAttempts = failed;
    s.lockedUntilRealtimeMs = until;
    s.permanentLockout = perm != 0;
    s.features = features;
    s.authenticatorId = authId;
    return s;
}

bool TemplateStore::saveState(const UserState& s) {
    std::vector<uint8_t> plain, sealed;
    putU32(&plain, kStateMagic);
    putU32(&plain, s.failedAttempts);
    putI64(&plain, s.lockedUntilRealtimeMs);
    putU32(&plain, s.permanentLockout ? 1 : 0);
    putU32(&plain, s.features);
    putI64(&plain, s.authenticatorId);
    if (!mSealer->seal(plain, aadFor("state", 0), &sealed)) return false;
    return writeAtomic(mDir + "/iris-state.bin", sealed);
}

}  // namespace cityman::iris
