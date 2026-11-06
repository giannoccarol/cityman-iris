/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_TAG "cityman.iris.led"

#include "IrLed.h"

#include <fcntl.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <sstream>

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/strings.h>

#include "Config.h"

namespace cityman::iris {

namespace {
constexpr int64_t kMs = 1000000LL;
// The condition variable sleeps on CLOCK_MONOTONIC: wake this early, then spin on BOOTTIME.
constexpr int64_t kSpinNs = 600000;
}  // namespace

int64_t boottimeNs() {
    struct timespec ts {};
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int IrLed::clampCurrent(int requestedMa, int driverMaxMa) {
    int limit = kLedMaxCurrentMa;
    if (driverMaxMa > 0) limit = std::min(limit, driverMaxMa);
    if (requestedMa > limit) requestedMa = limit;
    if (requestedMa < kLedMinCurrentMa) requestedMa = std::min(kLedMinCurrentMa, limit);
    return requestedMa;
}

int64_t IrLed::minGapNs(int64_t pulseNs) {
    // on / (on + gap) <= duty  <=>  gap >= on * (1000 - duty) / duty
    const int64_t byDuty = pulseNs * (1000 - kLedMaxDutyPermille) / kLedMaxDutyPermille;
    return std::max<int64_t>(byDuty, int64_t(kLedMinGapMs) * kMs);
}

IrLed::IrLed(std::vector<std::string> brightnessPaths, std::vector<std::string> maxBrightnessPaths)
    : mPaths(std::move(brightnessPaths)) {
    mFds.assign(mPaths.size(), -1);
    for (size_t i = 0; i < mPaths.size(); i++) {
        int channelMax = 0;
        std::string maxStr;
        if (i < maxBrightnessPaths.size() &&
            android::base::ReadFileToString(maxBrightnessPaths[i], &maxStr)) {
            channelMax = std::atoi(android::base::Trim(maxStr).c_str());
        }
        if (channelMax <= 0) {
            // qpnp-flash-led reports qcom,max-current (200 mA per torch channel on PMI8994).
            LOG(WARNING) << "cannot read the max_brightness of " << mPaths[i];
            channelMax = 0;
        }
        mFds[i] = open(mPaths[i].c_str(), O_WRONLY | O_CLOEXEC);
        if (mFds[i] < 0) {
            PLOG(ERROR) << "cannot open " << mPaths[i];
        } else {
            mDriverMaxMa += channelMax;
        }
    }
    {
        std::lock_guard<std::mutex> lock(mLock);
        forceOffLocked();
    }
    mThread = std::thread(&IrLed::threadLoop, this);
}

IrLed::~IrLed() {
    {
        std::lock_guard<std::mutex> lock(mLock);
        mQuit = true;
        mGeneration++;
        forceOffLocked();
    }
    mCond.notify_all();
    if (mThread.joinable()) mThread.join();
    std::lock_guard<std::mutex> lock(mLock);
    forceOffLocked();
    for (int fd : mFds) {
        if (fd >= 0) close(fd);
    }
}

bool IrLed::usable() const {
    std::lock_guard<std::mutex> lock(mLock);
    for (int fd : mFds) {
        if (fd < 0) return false;
    }
    return !mFds.empty() && mDriverMaxMa > 0;
}

bool IrLed::writeChannel(size_t i, int value) {
    if (mFds[i] < 0) {
        mFds[i] = open(mPaths[i].c_str(), O_WRONLY | O_CLOEXEC);
        if (mFds[i] < 0) return false;
    }
    char buf[16];
    int len = snprintf(buf, sizeof(buf), "%d\n", value);
    ssize_t rc = TEMP_FAILURE_RETRY(pwrite(mFds[i], buf, len, 0));
    if (rc != len) {
        PLOG(ERROR) << "write " << value << " to " << mPaths[i];
        return false;
    }
    return true;
}

bool IrLed::writeTotal(int totalMa) {
    // Even split; the first channel takes the odd milliampere.
    const int n = int(mPaths.size());
    bool ok = n > 0;
    for (int i = 0; i < n; i++) {
        const int share = totalMa / n + (i < totalMa % n ? 1 : 0);
        ok = writeChannel(size_t(i), share) && ok;
    }
    return ok;
}

void IrLed::forceOffLocked() {
    for (size_t i = 0; i < mPaths.size(); i++) {
        if (!writeChannel(i, 0)) {
            // Retry through a fresh descriptor: the LED must not stay lit.
            if (mFds[i] >= 0) close(mFds[i]);
            mFds[i] = -1;
            if (!writeChannel(i, 0)) LOG(ERROR) << "UNABLE TO SWITCH OFF " << mPaths[i];
        }
    }
    if (mLit && !mHistory.empty() && mHistory.back().offNs == INT64_MAX) {
        mHistory.back().offNs = boottimeNs();
        mOperationOnNs += mHistory.back().offNs - mHistory.back().onNs;
    }
    mLit = false;
}

int64_t IrLed::onTimeInWindowLocked(int64_t now) const {
    const int64_t windowStart = now - int64_t(kLedBudgetWindowMs) * kMs;
    int64_t total = 0;
    for (const auto& p : mHistory) {
        int64_t on = std::max(p.onNs, windowStart);
        int64_t off = p.offNs == INT64_MAX ? now : p.offNs;
        if (off > on) total += off - on;
    }
    return total;
}

int64_t IrLed::earliestOn(int64_t lengthNs) const {
    std::lock_guard<std::mutex> lock(mLock);
    if (mHistory.empty()) return 0;
    const auto& last = mHistory.back();
    if (last.offNs == INT64_MAX) return INT64_MAX;
    // The gap after a pulse must respect the duty of that pulse, and the gap before the new
    // pulse the duty of the new one.
    const int64_t prevLen = last.offNs - last.onNs;
    return last.offNs + std::max(minGapNs(prevLen), minGapNs(lengthNs));
}

bool IrLed::budgetAvailable(int64_t lengthNs) const {
    std::lock_guard<std::mutex> lock(mLock);
    return mOperationOnNs + lengthNs <= int64_t(kLedMaxOnPerOperationMs) * kMs &&
           onTimeInWindowLocked(boottimeNs()) + lengthNs <= int64_t(kLedMaxOnPerWindowMs) * kMs;
}

bool IrLed::allowedLocked(int64_t onNs, int64_t offNs) const {
    const int64_t len = offNs - onNs;
    if (len <= 0 || len > int64_t(kLedMaxPulseMs) * kMs) return false;
    if (mOperationOnNs + len > int64_t(kLedMaxOnPerOperationMs) * kMs) return false;
    if (onTimeInWindowLocked(boottimeNs()) + len > int64_t(kLedMaxOnPerWindowMs) * kMs) {
        return false;
    }
    if (!mHistory.empty()) {
        const auto& last = mHistory.back();
        if (last.offNs == INT64_MAX) return false;
        const int64_t gap = onNs - last.offNs;
        if (gap < minGapNs(last.offNs - last.onNs) || gap < minGapNs(len)) return false;
    }
    return true;
}

void IrLed::beginOperation() {
    std::lock_guard<std::mutex> lock(mLock);
    mOperationOnNs = 0;
}

bool IrLed::schedule(int64_t onNs, int64_t offNs, int currentMa) {
    std::lock_guard<std::mutex> lock(mLock);
    if (mQuit || mHasRequest || mLit) return false;
    if (onNs < boottimeNs()) return false;
    if (!allowedLocked(onNs, offNs)) return false;
    mRequest = {onNs, offNs, clampCurrent(currentMa, mDriverMaxMa)};
    mHasRequest = true;
    mCond.notify_all();
    return true;
}

bool IrLed::pending() const {
    std::lock_guard<std::mutex> lock(mLock);
    return mHasRequest || mLit;
}

void IrLed::cancel() {
    {
        std::lock_guard<std::mutex> lock(mLock);
        mHasRequest = false;
        mGeneration++;
        forceOffLocked();
    }
    mCond.notify_all();
}

bool IrLed::waitUntilLocked(std::unique_lock<std::mutex>& lock, int64_t deadlineNs, uint64_t gen) {
    for (;;) {
        if (mQuit || mGeneration != gen) return false;
        const int64_t left = deadlineNs - boottimeNs();
        if (left <= 0) return true;
        if (left > kSpinNs) {
            mCond.wait_for(lock, std::chrono::nanoseconds(left - kSpinNs));
            continue;
        }
        // Short spin for sub-millisecond accuracy; the lock is released so cancel() still works.
        lock.unlock();
        while (deadlineNs - boottimeNs() > 0) sched_yield();
        lock.lock();
    }
}

void IrLed::threadLoop() {
    std::unique_lock<std::mutex> lock(mLock);
    while (!mQuit) {
        mCond.wait(lock, [this] { return mQuit || mHasRequest; });
        if (mQuit) break;
        const Request req = mRequest;
        const uint64_t gen = mGeneration;
        if (!waitUntilLocked(lock, req.onNs, gen) || !mHasRequest) {
            mHasRequest = false;
            continue;
        }
        // Re-check every limit at the moment of switching on: budgets may have moved.
        const int64_t now = boottimeNs();
        const int64_t len = req.offNs - req.onNs;
        while (!mHistory.empty() && mHistory.front().offNs != INT64_MAX &&
               mHistory.front().offNs < now - int64_t(kLedBudgetWindowMs) * kMs) {
            mHistory.pop_front();
        }
        if (!allowedLocked(now, now + len)) {
            LOG(WARNING) << "IR LED pulse dropped by the limits";
            mHasRequest = false;
            continue;
        }
        mLit = true;
        if (!writeTotal(req.currentMa)) {
            forceOffLocked();
            mHasRequest = false;
            continue;
        }
        mHistory.push_back({boottimeNs(), INT64_MAX, req.currentMa});
        // Never longer than the requested length, measured from the real switch-on.
        const int64_t off = std::min(req.offNs, mHistory.back().onNs + len);
        waitUntilLocked(lock, off, gen);
        forceOffLocked();
        mHasRequest = false;
    }
    forceOffLocked();
}

std::vector<LedPulse> IrLed::pulsesSince(int64_t since) const {
    std::lock_guard<std::mutex> lock(mLock);
    std::vector<LedPulse> out;
    for (const auto& p : mHistory) {
        if (p.offNs >= since) out.push_back(p);
    }
    return out;
}

std::string IrLed::toString() const {
    std::lock_guard<std::mutex> lock(mLock);
    std::ostringstream os;
    os << "IR LED " << android::base::Join(mPaths, " + ") << ": driver max (sum) " << mDriverMaxMa << " mA, hard cap "
       << kLedMaxCurrentMa << " mA, pulse <= " << kLedMaxPulseMs << " ms, duty <= "
       << kLedMaxDutyPermille / 10 << " %, lit " << mLit << ", pending " << mHasRequest
       << ", operation on-time " << mOperationOnNs / kMs << " ms / " << kLedMaxOnPerOperationMs
       << " ms, window on-time " << onTimeInWindowLocked(boottimeNs()) / kMs << " ms / "
       << kLedMaxOnPerWindowMs << " ms per " << kLedBudgetWindowMs / 1000 << " s\n";
    return os.str();
}

}  // namespace cityman::iris
