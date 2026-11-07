/*
 * SPDX-FileCopyrightText: 2026 giannoccarol
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "IrLed.h"
#include "IrisCamera.h"
#include "Timing.h"
#include "iris_engine.h"

namespace cityman::iris {

struct TemplateDeleter {
    void operator()(iris_template* t) const { iris_template_destroy(t); }
};
using TemplatePtr = std::unique_ptr<iris_template, TemplateDeleter>;

// One eye search on one lit frame. `side` is the half of the frame that was searched (0 = first
// half along the split axis, 1 = second half; always 0 with split=none).
struct Probe {
    iris_status status = IRIS_E_INTERNAL;
    iris_quality quality{};
    TemplatePtr tmpl;
    int side = 0;
};

// Frame-locked capture: streams the iris camera, lights one frame every `period` frames with a
// pulse that covers its whole exposure (rolling shutter included), checks the lit frame against
// a dark reference, adapts exposure / LED current, and runs the engine on each half of the lit
// frame. Not thread safe except interrupt().
class IrisCapture {
  public:
    enum class Result { kProbes, kTimeout, kInterrupted, kCameraError, kLedUnavailable };

    // `engineContext` may be null (the engine then allocates per call).
    IrisCapture(IrLed* led, const iris_engine* engine, iris_context* engineContext);
    ~IrisCapture();

    bool start(std::string* error);
    void stop();
    void interrupt();
    bool firstFrameSeen() const { return mFirstFrame; }

    // Waits for the next processed lit frame (or timeout / interrupt / failure).
    Result next(int timeoutMs, std::vector<Probe>* probes);
    std::string stats() const;
    // The frame behind the last kProbes result (valid until the next call of next()).
    const Frame& lastLit() const { return mLastLit; }
    int sensorOrientation() const { return mCamera.sensorOrientation(); }

  private:
    void scheduleAfter(const Frame& f) { scheduleFrom(f.timestampNs, f.skewNs); }
    // Schedules the next pulse from the timestamp of a frame that has started (skew 0: unknown).
    void scheduleFrom(int64_t timestampNs, int64_t skewNs);
    void process(const Frame& lit, std::vector<Probe>* probes);
    void adapt(const std::vector<Probe>& probes, float meanY);
    void dump(const Frame& f, const char* tag);

    IrLed* mLed;
    const iris_engine* mEngine;
    iris_context* mEngineCtx;
    // Where each eye was in the last lit frames: the engine searches there first.
    struct EyeHint {
        bool valid = false;
        float x = 0, y = 0, r = 0;
        int misses = 0;
    } mHints[2];
    IrisCamera mCamera;
    std::atomic<bool> mInterrupted{false};
    bool mFirstFrame = false;

    // Tunables (vendor properties, clamped to Config.h).
    int mExposureUs = 0;
    int mIso = 0;
    int mLedMa = 0;
    int64_t mMarginNs = 0;
    int mPeriod = 0;
    int mSplit = 1;  // 0 none, 1 rows, 2 columns
    bool mSubtract = false;
    bool mDump = false;

    // Timing state.
    int64_t mLastSeq = 0;
    int64_t mLastTs = 0;
    int64_t mFrameDurationNs = 0;
    int64_t mTargetTs = 0;        // frame we lit (or will light); 0 = none
    int64_t mPreviousTarget = 0;
    int64_t mExposureSeenNs = 0;  // last exposure reported by a result
    float mDarkMean = -1.f;
    Frame mDark;
    Frame mLastLit;
    Frame mFrame;  // reused by next()

    // Counters for dumpsys / logs.
    int mPulses = 0, mLit = 0, mRejectedDim = 0, mMissed = 0, mProcessed = 0;
    int mDumped = 0;
};

// Last exposure / current that gave a well exposed eye, reused at the next operation.
void rememberGoodSettings(int exposureUs, int iso, int ledMa);

}  // namespace cityman::iris
