/*
 * SPDX-FileCopyrightText: 2026 giannoccarol
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace cityman::iris {

// One lit interval of the IR LED, CLOCK_BOOTTIME (the clock of the camera timestamps on msm8994).
// The times are the moments the sysfs writes returned, not the requested ones.
struct LedPulse {
    int64_t onNs;
    int64_t offNs;  // INT64_MAX while the pulse is still running
    int currentMa;
};

// Frame-locked pulse driver for the iris IR LED, with hard, non-overridable limits (Config.h). The
// emitter is fed by several flash channels whose currents add up: every current here is the
// total, split evenly over the channels.
//
// The caller schedules single pulses on absolute CLOCK_BOOTTIME times (one camera frame each, see
// IrisPipeline). The private thread lights the LED only for scheduled pulses that pass every
// limit: pulse <= kLedMaxPulseMs, duty cycle <= kLedMaxDutyPermille against the previous pulse,
// <= the per-operation limit set by beginOperation(), <= kLedMaxOnPerWindowMs per rolling window shared by
// the whole process, current <= kLedMaxCurrentMa and the driver's max_brightness. The LED is
// forced off on construction, cancel(), destruction and on any write error; init also writes 0
// when the service stops or restarts (rc file).
class IrLed {
  public:
    IrLed(std::vector<std::string> brightnessPaths, std::vector<std::string> maxBrightnessPaths);
    ~IrLed();
    IrLed(const IrLed&) = delete;
    IrLed& operator=(const IrLed&) = delete;

    bool usable() const;
    // Resets the per-operation on-time counter and sets the lit-time budget for this operation.
    void beginOperation(int maxOnMs);
    // Cancels any pending pulse and switches the LED off. Safe to call at any time.
    void cancel();
    // Schedules one pulse. Returns false (nothing scheduled) when the pulse would break a limit
    // or another pulse is still pending.
    bool schedule(int64_t onNs, int64_t offNs, int currentMa);
    bool pending() const;
    // Earliest start allowed for a pulse of `lengthNs` by the duty-cycle rule.
    int64_t earliestOn(int64_t lengthNs) const;
    // True when the budgets can still cover a pulse of `lengthNs`.
    bool budgetAvailable(int64_t lengthNs) const;
    // Pulses that ended after `sinceNs` (or are still running).
    std::vector<LedPulse> pulsesSince(int64_t sinceNs) const;
    int clampCurrent(int requestedMa) const { return clampCurrent(requestedMa, mDriverMaxMa); }
    std::string toString() const;

    // Exposed for the host unit test.
    static int clampCurrent(int requestedMa, int driverMaxMa);
    static int64_t minGapNs(int64_t pulseNs);

  private:
    struct Request {
        int64_t onNs = 0;
        int64_t offNs = 0;
        int currentMa = 0;
    };

    void threadLoop();
    bool writeTotal(int totalMa);
    bool writeChannel(size_t i, int value);
    void forceOffLocked();
    bool allowedLocked(int64_t onNs, int64_t offNs) const;
    int64_t onTimeInWindowLocked(int64_t nowNs) const;
    // Waits until `deadlineNs` (BOOTTIME) or until cancelled / a new request; returns false when
    // the wait was interrupted.
    bool waitUntilLocked(std::unique_lock<std::mutex>& lock, int64_t deadlineNs, uint64_t gen);

    const std::vector<std::string> mPaths;
    int mDriverMaxMa = 0;  // sum of the channels' max_brightness
    std::vector<int> mFds;

    mutable std::mutex mLock;
    std::condition_variable mCond;
    std::thread mThread;
    bool mQuit = false;
    bool mHasRequest = false;
    Request mRequest;
    uint64_t mGeneration = 0;  // bumped by cancel()
    bool mLit = false;
    int64_t mOperationOnNs = 0;
    int64_t mOperationLimitNs = 8000LL * 1000000LL;  // enroll default; set by beginOperation()
    std::deque<LedPulse> mHistory;  // pulses of the last kLedBudgetWindowMs
};

// Now on the clock of the camera timestamps (CLOCK_MONOTONIC unless the camera reports REALTIME).
int64_t boottimeNs();
void setCameraClockRealtime(bool realtime);

}  // namespace cityman::iris
