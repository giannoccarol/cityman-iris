/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Frame/pulse timing of the iris capture. No Android dependency: tested on the host
 * (tests/timing_test.cpp).
 */
#pragma once

#include <cstdint>
#include <vector>

#include "IrLed.h"

namespace cityman::iris {

// True when one pulse covers the whole exposure of the frame: [ts - margin, ts + skew + exp +
// margin] (SENSOR_TIMESTAMP = start of exposure of the first row, measured on the phone).
bool frameFullyLit(int64_t tsNs, int64_t skewNs, int64_t exposureNs, int64_t marginNs,
                   const std::vector<LedPulse>& pulses);
// True when no pulse touches the exposure window of the frame (a dark reference).
bool frameDark(int64_t tsNs, int64_t skewNs, int64_t exposureNs,
               const std::vector<LedPulse>& pulses);
// Start of the next frame to light: the first ts + m * fd (m >= 1) whose pulse can start after
// `earliestOnNs` and `nowNs` + 1 ms, at least (period - 0.5) frames after the previous target.
int64_t nextTarget(int64_t lastTsNs, int64_t frameDurationNs, int64_t marginNs, int64_t nowNs,
                   int64_t previousTargetNs, int period, int64_t earliestOnNs);

}  // namespace cityman::iris
