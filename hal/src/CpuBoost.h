/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <cstdint>
#include <mutex>

namespace cityman::iris {

// Big cores for the iris analysis during an enroll/authenticate operation, always undone:
// - the calling thread is pinned to the A57 cluster (cpu4-7) when one of them is online;
// - core_ctl keeps at least two A57 online (cpu4/core_ctl/min_cpus raised to 2, only if lower;
//   the previous value is saved in vendor.iris.saved_min_cpus so that a restarted service and the
//   rc onrestart put it back);
// - a perfd lock sets the A57 minimum frequency to about 1.25 GHz for at most the operation
//   duration (it expires by itself if the service dies).
// The power-save profile turns the A57 cluster off through core_ctl max_cpus: that choice is not
// overridden (min_cpus never exceeds max_cpus), the analysis then runs on the A53.
class CpuBoost {
  public:
    // Undoes what a previous instance of the service left (call once at start).
    static void recoverAtStartup();

    // `maxDurationMs` bounds the frequency lock. Call begin/end on the analysis thread.
    void begin(int maxDurationMs);
    void end();
    bool active() const { return mActive; }

  private:
    bool mActive = false;
    bool mPinned = false;
    int mRaisedFrom = -1;  // previous min_cpus when we raised it
    int mPerfHandle = 0;
};

// RAII helper for an operation.
class ScopedCpuBoost {
  public:
    ScopedCpuBoost(CpuBoost& b, int maxDurationMs) : mBoost(b) { mBoost.begin(maxDurationMs); }
    ~ScopedCpuBoost() { mBoost.end(); }
    ScopedCpuBoost(const ScopedCpuBoost&) = delete;
    ScopedCpuBoost& operator=(const ScopedCpuBoost&) = delete;

  private:
    CpuBoost& mBoost;
};

}  // namespace cityman::iris
