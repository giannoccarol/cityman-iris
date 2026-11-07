/*
 * SPDX-FileCopyrightText: 2026 giannoccarol
 * SPDX-License-Identifier: Apache-2.0
 *
 * Fixed limits and tunables of the cityman iris face HAL.
 *
 * Everything that protects the user's eyes or the LED is a constexpr here and is NOT
 * overridable at runtime: properties can only move a value inside these bounds.
 */
#pragma once

#include <cstdint>

namespace cityman::iris {

// ---------------------------------------------------------------------------------------------
// IR illuminator (PMI8994 flash block, channel 0 in torch mode: /sys/class/leds/led:torch_0).
//
// Reference: the Windows driver qccamflashtertiary8994.sys only accepted LED module 7 at four
// power levels (codes 0x0A/0x0F/0x17/0x16, about 140/200/300/290 mA at 12.5 mA per step) and
// fixed pulse lengths of 10 ms (flash) or 70 ms (assist), with a heat-load/cooldown model whose
// constants are unknown (IRIS-CITYMAN-2026-11-06.md section 1.4). We stay inside that envelope:
// current never above the 200 mA level, pulses never longer than the 70 ms assist pulse, and an
// explicit duty-cycle and per-minute budget of our own (the Windows cooldown values are unknown,
// so ours are deliberately conservative).
// ---------------------------------------------------------------------------------------------
// led:torch_0 and led:torch_1 drive the SAME IR emitter and their currents add up (measured 6/11:
// 50 + 50 mA gives the same image as 100 mA on one channel). Every current below is the TOTAL of
// both channels; the HAL splits it in half over the two channels (max_brightness 200 each).
inline constexpr const char* kLedBrightnessPaths[] = {
        "/sys/class/leds/led:torch_0/brightness",
        "/sys/class/leds/led:torch_1/brightness",
};
inline constexpr const char* kLedMaxBrightnessPaths[] = {
        "/sys/class/leds/led:torch_0/max_brightness",
        "/sys/class/leds/led:torch_1/max_brightness",
};

inline constexpr int kLedMaxCurrentMa = 200;      // hard cap on the TOTAL (Windows 200 mA level)
inline constexpr int kLedMinCurrentMa = 25;       // below this the image is useless anyway
inline constexpr int kLedDefaultCurrentMa = 140;  // Windows lowest level (code 0x0A)
inline constexpr int kLedMaxPulseMs = 70;         // Windows assist pulse length
inline constexpr int kLedMaxDutyPermille = 350;   // on / (on + following gap) <= 35 %
inline constexpr int kLedMinGapMs = 50;           // absolute floor between two pulses
inline constexpr int kLedMaxOnPerOperationMs = 4000;  // lit time per enroll/auth operation
inline constexpr int kLedBudgetWindowMs = 60000;      // rolling window of the thermal budget
inline constexpr int kLedMaxOnPerWindowMs = 8000;     // => <= 13 % average duty over a minute

// Frame-locked pulses (measured 6/11 on the phone, test kernel 6, iris_grab): the sysfs write
// takes 0.37 ms on average (max 1.3 ms) and the light follows within ~1 ms, on CLOCK_BOOTTIME,
// the clock of the camera timestamps. SENSOR_TIMESTAMP is the start of exposure of the first row;
// row r starts at ts + skew * r / H; rolling shutter skew 29.81 ms, frame duration 33.44 ms.
// A frame is fully lit only if one pulse covers [ts - margin, ts + skew + exposure + margin]:
// about 32 ms + exposure, so one frame in 3-4 is lit (free-running pulses lit 0 of 112 frames).
inline constexpr int kLedMarginUs = 2000;        // persist.vendor.iris.led_margin_us, <= max
inline constexpr int kLedMaxMarginUs = 8000;
inline constexpr int64_t kDefaultSkewNs = 29810000;  // used when the result has no skew
inline constexpr int64_t kDefaultFrameDurationNs = 33440000;
// Light one frame every K (K = 4 with 10 ms exposure: 44 ms pulse every 134 ms, duty 33 %,
// 23/23 pulses gave exactly one fully lit frame). The two frames next to the lit one are partly
// lit and dropped; the frame two away from it is the dark (ambient) reference.
inline constexpr int kLedFramePeriod = 4;
// Brightness cross-check: a frame counted as lit must be this much brighter (mean Y) than the
// last unlit frame, or it is discarded (wrong timing model, LED fault).
inline constexpr float kLitMinGain = 1.25f;
inline constexpr float kLitMinDelta = 4.0f;

// ---------------------------------------------------------------------------------------------
// Camera. The cityman camera HAL advertises the iris camera (ck71ir) with the SYSTEM_CAMERA
// capability: cameraserver hides it from apps and shows it to vendor clients of
// android.frameworks.cameraservice.service such as this HAL. It also forces the mono effect.
// ---------------------------------------------------------------------------------------------
inline constexpr int kFrameWidth = 1920;   // YUV_420_888 analysis stream (sensor is 1920x1088)
inline constexpr int kFrameHeight = 1080;
inline constexpr int64_t kFrameDurationNs = 33366700;  // requested (29.97 fps)
// Measured 6/11 (iris_grab, dark room, LED on a near surface): 10 ms ISO 800 200 mA gave
// 39 counts above black (10 bit); the signal is linear in current and exposure. The manual ISO
// of this HAL stops at 800.
inline constexpr int kDefaultExposureUs = 8000;
inline constexpr int kMinExposureUs = 500;
inline constexpr int kMaxExposureUs = 20000;  // skew + 20 ms + margins < 70 ms pulse
inline constexpr int kDefaultIso = 800;
inline constexpr int kMinIso = 100;
inline constexpr int kMaxIso = 800;
inline constexpr int kCameraOpenTimeoutMs = 3000;
inline constexpr int kFirstFrameTimeoutMs = 3000;

// ---------------------------------------------------------------------------------------------
// Operations
// ---------------------------------------------------------------------------------------------
inline constexpr int kSensorId = 0;
inline constexpr int kMaxEnrollmentsPerUser = 1;
inline constexpr int kEnrollTimeoutMs = 60000;  // framework ENROLL_TIMEOUT_SEC is 75 s
inline constexpr int kEnrollTargetSamples = 6;  // good samples per eye before building
inline constexpr int kEnrollMaxSamples = 16;    // samples kept per eye
inline constexpr int kAuthTimeoutMs = 4000;
inline constexpr int kAuthProbesForReject = 3;  // valid, non-matching probes => one failure
inline constexpr int kChallengeTimeoutMs = 10 * 60 * 1000;
inline constexpr int kMaxChallenges = 8;
inline constexpr int64_t kHatMaxAgeMs = 10 * 60 * 1000;  // "minutes, not hours"
inline constexpr int kFrameMessageIntervalMs = 200;      // acquired-info rate limit
inline constexpr int kMaxDumpFrames = 64;                // persist.vendor.iris.dump=1 only

// Lockout (CDD 7.3.10 style): timed lockout every 5 failures, permanent after 20.
inline constexpr int kLockoutTimedThreshold = 5;
inline constexpr int kLockoutTimedDurationMs = 30000;
inline constexpr int kLockoutPermanentThreshold = 20;

// Vendor acquired codes (index into framework-res face_acquired_vendor, device overlay).
inline constexpr int kVendorEyesNotOpen = 0;
inline constexpr int kVendorLedCooldown = 1;
inline constexpr int kVendorNoGlint = 2;

// Storage
inline constexpr const char* kSealKeyPath = "/data/vendor/iris/seal.key";
inline constexpr const char* kDumpDir = "/data/vendor/iris/dump";
// Per user: /data/vendor_de/<user>/facedata, created 0700 system:system by vold
// (system/vold/vold_prepare_subdirs.cpp) and labelled face_vendor_data_file.
inline constexpr const char* kUserDirFormat = "/data/vendor_de/%d/facedata";

}  // namespace cityman::iris
