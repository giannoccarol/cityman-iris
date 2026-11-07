/*
 * SPDX-FileCopyrightText: 2026 giannoccarol
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace cityman::iris {

// AES-256-GCM sealing with a per-device random key kept in /data/vendor/iris/seal.key.
//
// There is no usable TEE on cityman (IRIS-CITYMAN-2026-11-06.md section 5.2), so the key is only
// protected by DAC (0600 system) and SELinux (face_vendor_data_file), like the templates
// themselves. What sealing adds: integrity (a modified or swapped file is rejected), binding to
// user and enrollment id (AAD), and templates that are useless when copied off the device
// without the key file. It does NOT protect against root or a SELinux bypass on the phone.
class Sealer {
  public:
    // Loads or creates the key. Returns false when no key is available.
    bool init(const std::string& keyPath);
    bool seal(const std::vector<uint8_t>& plain, const std::string& aad,
              std::vector<uint8_t>* out) const;
    bool open(const std::vector<uint8_t>& sealed, const std::string& aad,
              std::vector<uint8_t>* out) const;
    bool ready() const { return mReady; }

  private:
    uint8_t mKey[32] = {};
    bool mReady = false;
};

struct UserState {
    uint32_t failedAttempts = 0;      // since the last success / reset
    int64_t lockedUntilRealtimeMs = 0;
    bool permanentLockout = false;
    uint32_t features = 0;            // bit per aidl Feature value (stored, not enforced)
    int64_t authenticatorId = 0;      // 0 when the user has no enrollment
};

struct Enrollment {
    int32_t id = 0;
    // One serialized iris_template per eye side (0 = left half of the sensor image, 1 = right).
    std::vector<std::vector<uint8_t>> templates;
};

// Per-user persistent storage in /data/vendor_de/<user>/facedata (see Config.h).
class TemplateStore {
  public:
    TemplateStore(int userId, const Sealer* sealer);

    bool usable() const;
    std::vector<int32_t> enrollmentIds() const;
    std::optional<Enrollment> load(int32_t id) const;
    std::vector<Enrollment> loadAll() const;
    bool save(const Enrollment& e);
    bool remove(int32_t id);
    // Random positive id not yet in use.
    int32_t newEnrollmentId() const;

    UserState loadState() const;
    bool saveState(const UserState& s);

    const std::string& dir() const { return mDir; }

  private:
    std::string pathFor(int32_t id) const;
    std::string aadFor(const char* kind, int32_t id) const;
    bool writeAtomic(const std::string& path, const std::vector<uint8_t>& data) const;

    const int mUserId;
    const Sealer* mSealer;
    std::string mDir;
};

// Exposed for the host unit test.
std::vector<uint8_t> encodeEnrollment(const Enrollment& e);
bool decodeEnrollment(const std::vector<uint8_t>& data, Enrollment* e);

}  // namespace cityman::iris
