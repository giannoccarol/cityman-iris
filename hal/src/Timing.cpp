/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */
#include "Timing.h"

#include <algorithm>
#include <climits>

namespace cityman::iris {

namespace {
constexpr int64_t kMsNs = 1000000LL;
}  // namespace

bool frameFullyLit(int64_t ts, int64_t skew, int64_t exposure, int64_t margin,
                   const std::vector<LedPulse>& pulses) {
    const int64_t from = ts - margin;
    const int64_t to = ts + skew + exposure + margin;
    for (const auto& p : pulses) {
        if (p.onNs <= from && p.offNs != INT64_MAX && p.offNs >= to) return true;
    }
    return false;
}

bool frameDark(int64_t ts, int64_t skew, int64_t exposure, const std::vector<LedPulse>& pulses) {
    // 1 ms of slack on both sides for the switch-on/off of the emitter.
    const int64_t from = ts - kMsNs;
    const int64_t to = ts + skew + exposure + kMsNs;
    for (const auto& p : pulses) {
        const int64_t off = p.offNs == INT64_MAX ? INT64_MAX : p.offNs;
        if (p.onNs < to && off > from) return false;
    }
    return true;
}

int64_t nextTarget(int64_t lastTs, int64_t fd, int64_t margin, int64_t now, int64_t previousTarget,
                   int period, int64_t earliestOn) {
    int64_t lower = now + kMsNs + margin;
    if (previousTarget > 0) lower = std::max(lower, previousTarget + period * fd - fd / 2);
    if (earliestOn > 0 && earliestOn != INT64_MAX) lower = std::max(lower, earliestOn + margin);
    int64_t m = 1;
    if (lower > lastTs + fd) m = (lower - lastTs + fd - 1) / fd;
    return lastTs + m * fd;
}

}  // namespace cityman::iris
