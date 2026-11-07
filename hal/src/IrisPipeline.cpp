/*
 * SPDX-FileCopyrightText: 2026 giannoccarol
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_TAG "cityman.iris.pipeline"

#include "IrisPipeline.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <sstream>

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/stringprintf.h>
#include <android-base/unique_fd.h>

#include "Config.h"

namespace cityman::iris {

using android::base::GetBoolProperty;
using android::base::GetIntProperty;
using android::base::GetProperty;

namespace {

constexpr int64_t kMsNs = 1000000LL;

std::mutex gGoodLock;
int gGoodExposureUs = 0;
int gGoodIso = 0;
int gGoodLedMa = 0;

}  // namespace

void rememberGoodSettings(int exposureUs, int iso, int ledMa) {
    std::lock_guard<std::mutex> lock(gGoodLock);
    gGoodExposureUs = exposureUs;
    gGoodIso = iso;
    gGoodLedMa = ledMa;
}

IrisCapture::IrisCapture(IrLed* led, const iris_engine* engine, iris_context* engineContext)
    : mLed(led), mEngine(engine), mEngineCtx(engineContext) {}

IrisCapture::~IrisCapture() {
    stop();
}

bool IrisCapture::start(std::string* error) {
    int goodExp, goodIso, goodLed;
    {
        std::lock_guard<std::mutex> lock(gGoodLock);
        goodExp = gGoodExposureUs;
        goodIso = gGoodIso;
        goodLed = gGoodLedMa;
    }
    mExposureUs = GetIntProperty("persist.vendor.iris.exposure_us",
                                 goodExp ? goodExp : kDefaultExposureUs, kMinExposureUs,
                                 kMaxExposureUs);
    mIso = GetIntProperty("persist.vendor.iris.iso", goodIso ? goodIso : kDefaultIso, kMinIso,
                          kMaxIso);
    mLedMa = mLed->clampCurrent(GetIntProperty("persist.vendor.iris.led_ma",
                                               goodLed ? goodLed : kLedDefaultCurrentMa,
                                               kLedMinCurrentMa, kLedMaxCurrentMa));
    mMarginNs = int64_t(GetIntProperty("persist.vendor.iris.led_margin_us", kLedMarginUs, 0,
                                       kLedMaxMarginUs)) *
                1000;
    mPeriod = GetIntProperty("persist.vendor.iris.period", kLedFramePeriod, 3, 8);
    const std::string split = GetProperty("persist.vendor.iris.split", "rows");
    mSplit = split == "none" ? 0 : split == "cols" ? 2 : 1;
    mSubtract = GetBoolProperty("persist.vendor.iris.subtract", false);
    mDump = GetBoolProperty("persist.vendor.iris.dump", false);

    mInterrupted = false;
    mFirstFrame = false;
    mLastSeq = mLastTs = mFrameDurationNs = mTargetTs = mPreviousTarget = 0;
    mExposureSeenNs = 0;
    mDarkMean = -1.f;
    mDark = Frame{};
    mPulses = mLit = mRejectedDim = mMissed = mProcessed = 0;
    mHints[0] = mHints[1] = EyeHint{};

    if (!mLed->usable()) {
        *error = "IR LED unavailable";
        return false;
    }
    mLed->beginOperation();
    if (!mCamera.open(mExposureUs, mIso, error)) return false;
    LOG(INFO) << "capture: exposure " << mExposureUs << " us, ISO " << mIso << ", LED "
              << mLedMa << " mA, one frame in " << mPeriod << ", margin " << mMarginNs / 1000
              << " us, split " << split << (mSubtract ? ", ambient subtraction" : "");
    return true;
}

void IrisCapture::stop() {
    mLed->cancel();
    mCamera.close();
    if (mPulses) LOG(INFO) << "capture done: " << stats();
}

void IrisCapture::interrupt() {
    mInterrupted = true;
    mCamera.interrupt();
    mLed->cancel();
}

std::string IrisCapture::stats() const {
    std::ostringstream os;
    os << "pulses " << mPulses << ", lit frames " << mLit << ", processed " << mProcessed
       << ", too dim " << mRejectedDim << ", missed " << mMissed << ", frame "
       << mFrameDurationNs / 1000 << " us, exposure " << mExposureUs << " us ISO " << mIso
       << ", LED " << mLedMa << " mA";
    return os.str();
}

void IrisCapture::scheduleFrom(int64_t timestampNs, int64_t skewNs) {
    if (mTargetTs != 0 || mLed->pending() || timestampNs <= 0) return;
    const int64_t fd = mFrameDurationNs ? mFrameDurationNs : kDefaultFrameDurationNs;
    const int64_t exposure = std::max<int64_t>(mExposureSeenNs, int64_t(mExposureUs) * 1000);
    const int64_t skew = skewNs > 0 ? skewNs : kDefaultSkewNs;
    const int64_t len = skew + exposure + 2 * mMarginNs;
    if (len > int64_t(kLedMaxPulseMs) * kMsNs) {
        LOG(WARNING) << "pulse of " << len / 1000 << " us would be too long";
        return;
    }
    if (!mLed->budgetAvailable(len)) return;
    const int64_t t = nextTarget(timestampNs, fd, mMarginNs, boottimeNs(), mPreviousTarget,
                                 mPeriod, mLed->earliestOn(len));
    if (mLed->schedule(t - mMarginNs, t + skew + exposure + mMarginNs, mLedMa)) {
        mTargetTs = t;
        mPreviousTarget = t;
        mPulses++;
    }
}

IrisCapture::Result IrisCapture::next(int timeoutMs, std::vector<Probe>* probes) {
    probes->clear();
    const int64_t start = boottimeNs();
    const int64_t deadline = start + int64_t(timeoutMs) * kMsNs;
    while (!mInterrupted) {
        const int64_t now = boottimeNs();
        if (now >= deadline) return Result::kTimeout;
        if (!mFirstFrame && now - start > int64_t(kFirstFrameTimeoutMs) * kMsNs) {
            LOG(ERROR) << "no frame from the iris camera in " << kFirstFrameTimeoutMs << " ms";
            return Result::kCameraError;
        }
        Frame& f = mFrame;  // copy-assigned by waitFrame(): keeps its capacity
        // Until the first image arrives (it comes 55-68 ms after its timestamp), poll the shutter
        // notifications: the first pulse can then be planned about one frame earlier.
        const int waitMs = int(std::min<int64_t>((deadline - now) / kMsNs + 1, mFirstFrame ? 500 : 5));
        if (!mCamera.waitFrame(mLastSeq, waitMs, &f)) {
            if (mInterrupted) return Result::kInterrupted;
            if (mCamera.failed()) return Result::kCameraError;
            if (!mFirstFrame) scheduleFrom(mCamera.lastStartTimestamp(), 0);
            continue;
        }
        mFirstFrame = true;
        if (mLastTs && f.sequence == mLastSeq + 1) {
            const int64_t d = f.timestampNs - mLastTs;
            if (d > 25 * kMsNs && d < 45 * kMsNs) {
                mFrameDurationNs = mFrameDurationNs ? (mFrameDurationNs * 7 + d) / 8 : d;
            }
        }
        mLastSeq = f.sequence;
        mLastTs = f.timestampNs;
        if (f.exposureNs > 0) mExposureSeenNs = f.exposureNs;
        const int64_t fd = mFrameDurationNs ? mFrameDurationNs : kDefaultFrameDurationNs;
        const int64_t exposure =
                f.exposureNs > 0 ? f.exposureNs : int64_t(mExposureUs) * 1000;
        const int64_t skew = f.skewNs > 0 ? f.skewNs : kDefaultSkewNs;
        const auto pulses = mLed->pulsesSince(f.timestampNs - 200 * kMsNs);

        if (frameDark(f.timestampNs, skew, exposure, pulses)) {
            mDarkMean = f.meanY;
            if (mSubtract) {
                mDark.y.assign(f.y.begin(), f.y.end());
                mDark.width = f.width;
                mDark.height = f.height;
                mDark.timestampNs = f.timestampNs;
            }
        }
        // The callback comes 55-68 ms after the timestamp: the lit frame is recognised by its
        // timestamp, not by arrival order. Frames next to it are partly lit and ignored.
        const bool isTarget = mTargetTs != 0 && std::llabs(f.timestampNs - mTargetTs) <= fd / 4;
        if (!isTarget) {
            if (mTargetTs != 0 && f.timestampNs > mTargetTs + fd / 4) {
                mMissed++;  // the lit frame was dropped
                mTargetTs = 0;
            }
            scheduleAfter(f);
            if (mTargetTs == 0 && !mLed->pending() &&
                !mLed->budgetAvailable(kDefaultSkewNs + int64_t(mExposureUs) * 1000)) {
                LOG(WARNING) << "IR LED budget exhausted";
                return Result::kLedUnavailable;
            }
            continue;
        }
        mTargetTs = 0;
        scheduleAfter(f);  // light the next one before the engine runs
        // The pulse was scheduled with mMarginNs on both sides; the recorded switch times only need
        // to cover the exposure (the write itself takes 0.4-1.3 ms).
        if (!frameFullyLit(f.timestampNs, skew, exposure, 0, pulses)) {
            mMissed++;
            continue;
        }
        if (mDarkMean >= 0.f && f.meanY - mDarkMean < kLitMinDelta &&
            f.meanY < mDarkMean * kLitMinGain) {
            mRejectedDim++;
            if (mRejectedDim % 8 == 1) {
                LOG(WARNING) << "lit frame not brighter than the dark one (" << f.meanY << " vs "
                             << mDarkMean << "): timing or LED problem";
            }
            continue;
        }
        mLit++;
        // Copy-assign: the buffers keep their capacity, no allocation per frame.
        mLastLit.y.assign(f.y.begin(), f.y.end());
        mLastLit.width = f.width;
        mLastLit.height = f.height;
        mLastLit.timestampNs = f.timestampNs;
        mLastLit.exposureNs = f.exposureNs;
        mLastLit.skewNs = f.skewNs;
        mLastLit.meanY = f.meanY;
        mLastLit.sequence = f.sequence;
        process(f, probes);
        adapt(*probes, f.meanY);
        return Result::kProbes;
    }
    return Result::kInterrupted;
}

void IrisCapture::process(const Frame& lit, std::vector<Probe>* probes) {
    const uint8_t* pixels = lit.y.data();
    std::vector<uint8_t> diff;
    const int64_t fd = mFrameDurationNs ? mFrameDurationNs : kDefaultFrameDurationNs;
    if (mSubtract && !mDark.y.empty() && mDark.width == lit.width &&
        mDark.height == lit.height && std::llabs(lit.timestampNs - mDark.timestampNs) <= 3 * fd) {
        diff.resize(lit.y.size());
        for (size_t i = 0; i < diff.size(); i++) {
            const int v = int(lit.y[i]) - int(mDark.y[i]);
            diff[i] = uint8_t(v < 0 ? 0 : v);
        }
        pixels = diff.data();
    }
    dump(lit, diff.empty() ? "lit" : "lit-raw");
    if (!diff.empty()) {
        Frame d = lit;
        d.y = diff;
        dump(d, "lit-sub");
    }

    const int sides = mSplit == 0 ? 1 : 2;
    for (int s = 0; s < sides; s++) {
        iris_image img{};
        img.data = pixels;
        img.width = lit.width;
        img.height = lit.height;
        img.stride = lit.width;
        img.format = IRIS_PIX_GRAY8;
        // Halves overlap by 15 % so an eye near the middle is whole in one of them.
        if (mSplit == 1) {
            const int h = lit.height * 23 / 40;
            img.roi_x = 0;
            img.roi_w = lit.width;
            img.roi_y = s == 0 ? 0 : lit.height - h;
            img.roi_h = h;
        } else if (mSplit == 2) {
            const int w = lit.width * 23 / 40;
            img.roi_y = 0;
            img.roi_h = lit.height;
            img.roi_x = s == 0 ? 0 : lit.width - w;
            img.roi_w = w;
        }
        Probe p;
        p.side = s;
        p.quality.struct_size = sizeof(p.quality);
        iris_template* t = nullptr;
        EyeHint& h = mHints[s];
        const iris_search_hint hint{h.x, h.y, h.r};
        p.status = iris_extract_ex(mEngine, mEngineCtx, &img, h.valid ? &hint : nullptr, &t,
                                   &p.quality);
        p.tmpl.reset(t);
        if (p.quality.eye_found && p.quality.iris_r > 0.f) {
            h.valid = true;
            h.x = p.quality.iris_x, h.y = p.quality.iris_y, h.r = p.quality.iris_r;
            h.misses = 0;
        } else if (h.valid && ++h.misses >= 2) {
            h.valid = false;  // lost for two lit frames (~0.3 s): search the whole half again
        }
        probes->push_back(std::move(p));
    }
    mProcessed++;
}

void IrisCapture::adapt(const std::vector<Probe>& probes, float meanY) {
    const Probe* best = nullptr;
    for (const auto& p : probes) {
        if (p.quality.eye_found && (!best || p.quality.overall > best->quality.overall)) best = &p;
    }
    int dir = 0;
    if (best) {
        if (best->status == IRIS_Q_UNDEREXPOSED || best->quality.exposure < 0.30f) dir = 1;
        if (best->status == IRIS_Q_OVEREXPOSED || best->quality.exposure > 0.97f) dir = -1;
        if (best->status == IRIS_OK) rememberGoodSettings(mExposureUs, mIso, mLedMa);
    } else {
        for (const auto& p : probes) {
            if (p.status == IRIS_Q_UNDEREXPOSED) dir = 1;
            if (p.status == IRIS_Q_OVEREXPOSED) dir = -1;
        }
        if (dir == 0 && meanY < 20.f) dir = 1;
        if (dir == 0 && meanY > 210.f) dir = -1;
    }
    if (dir == 0) return;
    int exposure = mExposureUs;
    int led = mLedMa;
    if (dir > 0) {
        // Current first: a short exposure freezes eye and hand movement.
        if (led < mLed->clampCurrent(kLedMaxCurrentMa)) {
            led = mLed->clampCurrent(led * 5 / 4 + 5);
        } else {
            exposure = std::min(kMaxExposureUs, exposure * 7 / 5);
        }
    } else {
        if (exposure > kDefaultExposureUs) {
            exposure = std::max(kDefaultExposureUs, exposure * 5 / 7);
        } else if (led > kLedMinCurrentMa) {
            led = mLed->clampCurrent(led * 4 / 5);
        } else {
            exposure = std::max(kMinExposureUs, exposure * 5 / 7);
        }
    }
    if (led != mLedMa) {
        LOG(DEBUG) << "LED " << mLedMa << " -> " << led << " mA";
        mLedMa = led;
    }
    if (exposure != mExposureUs) {
        LOG(DEBUG) << "exposure " << mExposureUs << " -> " << exposure << " us";
        mExposureUs = exposure;
        mCamera.setExposure(mExposureUs, mIso);
    }
}

void IrisCapture::dump(const Frame& f, const char* tag) {
    if (!mDump || mDumped >= kMaxDumpFrames) return;
    mkdir(kDumpDir, 0700);
    const std::string path = android::base::StringPrintf(
            "%s/%lld-%s.pgm", kDumpDir, static_cast<long long>(f.timestampNs), tag);
    android::base::unique_fd fd(TEMP_FAILURE_RETRY(
            open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0600)));
    if (fd < 0) return;
    const std::string hdr = android::base::StringPrintf("P5\n%d %d\n255\n", f.width, f.height);
    if (android::base::WriteFully(fd, hdr.data(), hdr.size()) &&
        android::base::WriteFully(fd, f.y.data(), f.y.size())) {
        mDumped++;
    }
}

}  // namespace cityman::iris
