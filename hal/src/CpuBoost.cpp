/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_TAG "cityman.iris.boost"

#include "CpuBoost.h"

#include <dlfcn.h>
#include <sched.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <string>

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/parseint.h>
#include <android-base/properties.h>
#include <android-base/strings.h>

#include "Config.h"

namespace cityman::iris {

namespace {

constexpr const char* kMinCpusPath = "/sys/devices/system/cpu/cpu4/core_ctl/min_cpus";
constexpr const char* kSavedProp = "vendor.iris.saved_min_cpus";
constexpr int kBigMinCpus = 2;
constexpr int kFirstBigCpu = 4, kLastBigCpu = 7, kLastCpu = 7;
// MSM8994 perfd opcode (old mpctl encoding, as in the cityman power HAL): CPU4 cluster minimum
// frequency, VV in 100 MHz units, perfd picks the next table frequency (1248 MHz).
constexpr int kBigMinFreq1248 = 0x1F0C;

int readInt(const char* path) {
    std::string s;
    int v = -1;
    if (!android::base::ReadFileToString(path, &s) ||
        !android::base::ParseInt(android::base::Trim(s), &v)) {
        return -1;
    }
    return v;
}

bool writeInt(const char* path, int v) {
    return android::base::WriteStringToFile(std::to_string(v), path);
}

// libqti-perfd-client.so (vendor blob), loaded once.
struct Perfd {
    int (*acq)(int handle, int duration, int list[], int numArgs) = nullptr;
    int (*rel)(int handle) = nullptr;
    Perfd() {
        void* h = dlopen("libqti-perfd-client.so", RTLD_NOW);
        if (!h) {
            LOG(WARNING) << "perfd client unavailable: " << dlerror();
            return;
        }
        acq = reinterpret_cast<decltype(acq)>(dlsym(h, "perf_lock_acq"));
        rel = reinterpret_cast<decltype(rel)>(dlsym(h, "perf_lock_rel"));
        if (!acq || !rel) {
            LOG(WARNING) << "perfd client without perf_lock_acq/rel";
            acq = nullptr;
            rel = nullptr;
        }
    }
};

Perfd& perfd() {
    static Perfd p;
    return p;
}

std::mutex gMinCpusLock;  // the service may run two sessions' operations back to back

}  // namespace

void CpuBoost::recoverAtStartup() {
    std::lock_guard<std::mutex> lock(gMinCpusLock);
    const std::string saved = android::base::GetProperty(kSavedProp, "");
    int v = -1;
    if (!saved.empty() && android::base::ParseInt(saved, &v) && v >= 0 && v <= 4) {
        LOG(WARNING) << "restoring core_ctl min_cpus " << v << " left by a previous instance";
        writeInt(kMinCpusPath, v);
    }
    if (!saved.empty()) android::base::SetProperty(kSavedProp, "");
}

void CpuBoost::begin(int maxDurationMs) {
    if (mActive) end();
    mActive = true;

    // 1. Keep two A57 online (only raise, never lower).
    {
        std::lock_guard<std::mutex> lock(gMinCpusLock);
        const int cur = readInt(kMinCpusPath);
        if (cur >= 0 && cur < kBigMinCpus) {
            android::base::SetProperty(kSavedProp, std::to_string(cur));
            if (writeInt(kMinCpusPath, kBigMinCpus)) {
                mRaisedFrom = cur;
            } else {
                PLOG(WARNING) << "cannot raise " << kMinCpusPath;
                android::base::SetProperty(kSavedProp, "");
            }
        }
    }

    // 2. A57 minimum frequency through perfd, bounded in time.
    if (perfd().acq) {
        int res[] = {kBigMinFreq1248};
        mPerfHandle = perfd().acq(0, maxDurationMs, res, 1);
        if (mPerfHandle <= 0) mPerfHandle = 0;
    }

    // 3. This thread on the A57 cluster, if any of its cores is online (the kernel rejects a mask
    //    without an online CPU: then the thread stays where it is).
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int c = kFirstBigCpu; c <= kLastBigCpu; c++) CPU_SET(c, &set);
    mPinned = sched_setaffinity(0, sizeof(set), &set) == 0;
    LOG(DEBUG) << "boost: min_cpus " << (mRaisedFrom >= 0 ? "raised" : "unchanged") << ", perf lock "
               << mPerfHandle << ", " << (mPinned ? "pinned to A57" : "A57 offline, not pinned");
}

void CpuBoost::end() {
    if (!mActive) return;
    mActive = false;
    if (mPinned) {
        cpu_set_t set;
        CPU_ZERO(&set);
        for (int c = 0; c <= kLastCpu; c++) CPU_SET(c, &set);
        sched_setaffinity(0, sizeof(set), &set);
        mPinned = false;
    }
    if (mPerfHandle > 0 && perfd().rel) perfd().rel(mPerfHandle);
    mPerfHandle = 0;
    if (mRaisedFrom >= 0) {
        std::lock_guard<std::mutex> lock(gMinCpusLock);
        // Put the old value back unless someone (perfd, the power HAL) changed it meanwhile.
        if (readInt(kMinCpusPath) == kBigMinCpus) writeInt(kMinCpusPath, mRaisedFrom);
        android::base::SetProperty(kSavedProp, "");
        mRaisedFrom = -1;
    }
}

}  // namespace cityman::iris
