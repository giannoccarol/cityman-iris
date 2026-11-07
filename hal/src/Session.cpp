/*
 * SPDX-FileCopyrightText: 2026 giannoccarol
 * SPDX-License-Identifier: Apache-2.0
 *
 * Structure follows the AOSP example face HAL (hardware/interfaces/biometrics/face/aidl/default):
 * one worker thread per session runs the operations, binder calls only queue them.
 */
#define LOG_TAG "cityman.iris.session"

#include "Session.h"

#include <time.h>

#include <algorithm>
#include <chrono>
#include <sstream>

#include <android-base/logging.h>
#include <android-base/properties.h>
#include <openssl/rand.h>

#include "Config.h"

namespace aidl::android::hardware::biometrics::face {

using namespace ::cityman::iris;

namespace {

int64_t clockMs(clockid_t id) {
    struct timespec ts {};
    clock_gettime(id, &ts);
    return int64_t(ts.tv_sec) * 1000 + ts.tv_nsec / 1000000;
}
int64_t bootMs() {
    return clockMs(CLOCK_BOOTTIME);
}
// Lockout deadlines are stored on disk: wall clock, so they survive a reboot.
int64_t wallMs() {
    return clockMs(CLOCK_REALTIME);
}

int64_t randomNonZero() {
    int64_t v = 0;
    while (v == 0) RAND_bytes(reinterpret_cast<uint8_t*>(&v), sizeof(v));
    return v;
}

void onClientDeath(void* cookie) {
    LOG(INFO) << "face client died";
    auto* session = static_cast<Session*>(cookie);
    if (session && !session->isClosed()) session->close();
}

// Acquired info for the best probe of a lit frame.
std::pair<AcquiredInfo, int32_t> acquiredFor(const std::vector<Probe>& probes) {
    const Probe* best = nullptr;
    for (const auto& p : probes) {
        if (p.status == IRIS_OK) return {AcquiredInfo::GOOD, 0};
        if (!best || (p.quality.eye_found && !best->quality.eye_found) ||
            (p.quality.eye_found == best->quality.eye_found &&
             p.quality.overall > best->quality.overall)) {
            best = &p;
        }
    }
    if (!best) return {AcquiredInfo::NOT_DETECTED, 0};
    switch (best->status) {
        case IRIS_Q_UNDEREXPOSED:
            return {AcquiredInfo::TOO_DARK, 0};
        case IRIS_Q_OVEREXPOSED:
            return {AcquiredInfo::TOO_BRIGHT, 0};
        case IRIS_Q_IMAGE_TOO_SMALL:
        case IRIS_Q_NO_EYE:
            return {AcquiredInfo::NOT_DETECTED, 0};
        case IRIS_Q_NO_IRIS:
        case IRIS_Q_BAD_GEOMETRY:
            return {AcquiredInfo::POOR_GAZE, 0};
        case IRIS_Q_IRIS_TOO_SMALL:
            return {AcquiredInfo::TOO_FAR, 0};
        case IRIS_Q_OUT_OF_FRAME:
            return {AcquiredInfo::TOO_CLOSE, 0};
        case IRIS_Q_BLURRED:
            return {AcquiredInfo::TOO_MUCH_MOTION, 0};
        case IRIS_Q_OCCLUDED:
            return {AcquiredInfo::VENDOR, kVendorEyesNotOpen};
        case IRIS_Q_LOW_TEXTURE:
        default:
            return {AcquiredInfo::INSUFFICIENT, 0};
    }
}

}  // namespace

// ------------------------------------------------------------------------------ cancellation

ndk::ScopedAStatus CancellationSignal::cancel() {
    if (auto s = mSession.lock()) s->cancelOperation(mOpId);
    return ndk::ScopedAStatus::ok();
}

// ----------------------------------------------------------------------------------- session

Session::Session(int32_t userId, std::shared_ptr<ISessionCallback> cb, IrisContext* ctx)
    : mUserId(userId), mCb(std::move(cb)), mCtx(ctx) {
    mDeathRecipient = AIBinder_DeathRecipient_new(onClientDeath);
    mWorker = std::thread(&Session::workerLoop, this);
    mTimer = std::thread([this] {
        std::unique_lock<std::mutex> lock(mLock);
        while (!mStopWorker) {
            if (mTimerDeadlineMs == 0) {
                mTimerCond.wait(lock);
                continue;
            }
            const int64_t left = mTimerDeadlineMs - bootMs();
            if (left > 0) {
                mTimerCond.wait_for(lock, std::chrono::milliseconds(left));
                continue;
            }
            mTimerDeadlineMs = 0;
            const uint64_t id = mNextOpId++;
            mQueue.emplace_back(id, [this](uint64_t) {
                TemplateStore store(mUserId, &mCtx->sealer);
                UserState state = store.loadState();
                if (!state.permanentLockout && state.lockedUntilRealtimeMs != 0 &&
                    state.lockedUntilRealtimeMs <= wallMs()) {
                    state.lockedUntilRealtimeMs = 0;
                    store.saveState(state);
                    LOG(INFO) << "timed lockout expired";
                    mCb->onLockoutCleared();
                }
            });
            mCond.notify_all();
        }
    });
}

Session::~Session() {
    {
        std::lock_guard<std::mutex> lock(mLock);
        mStopWorker = true;
        if (mCapture) mCapture->interrupt();
    }
    mCond.notify_all();
    mTimerCond.notify_all();
    if (mWorker.joinable() && mWorker.get_id() != std::this_thread::get_id()) mWorker.join();
    if (mTimer.joinable()) mTimer.join();
    if (mWorker.joinable()) mWorker.detach();
    AIBinder_DeathRecipient_delete(mDeathRecipient);
}

binder_status_t Session::linkToDeath(AIBinder* binder) {
    return AIBinder_linkToDeath(binder, mDeathRecipient, this);
}

std::shared_ptr<common::ICancellationSignal> Session::post(Task task) {
    uint64_t id;
    {
        std::lock_guard<std::mutex> lock(mLock);
        id = mNextOpId++;
        mQueue.emplace_back(id, std::move(task));
    }
    mCond.notify_all();
    return ndk::SharedRefBase::make<CancellationSignal>(ref<Session>(), id);
}

void Session::workerLoop() {
    std::unique_lock<std::mutex> lock(mLock);
    for (;;) {
        mCond.wait(lock, [this] { return mStopWorker || !mQueue.empty(); });
        if (mStopWorker) break;
        auto [id, task] = std::move(mQueue.front());
        mQueue.pop_front();
        mCurrentOp = id;
        lock.unlock();
        task(id);
        lock.lock();
        mCurrentOp = 0;
        mCancelled.erase(id);
    }
}

void Session::cancelOperation(uint64_t opId) {
    std::lock_guard<std::mutex> lock(mLock);
    mCancelled.insert(opId);
    if (mCurrentOp == opId && mCapture) mCapture->interrupt();
}

bool Session::opCancelled(uint64_t opId) {
    std::lock_guard<std::mutex> lock(mLock);
    return mCancelled.count(opId) != 0 || mStopWorker;
}

bool Session::checkHat(const keymaster::HardwareAuthToken& hat, bool needChallenge) {
    // Weak (class 2) sensor: no shared HMAC key with gatekeeper/keymaster on this device, so the
    // MAC is required to be present but cannot be verified. The challenge and age checks keep a
    // stale or replayed token from being used for enrollment.
    if (hat.mac.empty()) {
        LOG(ERROR) << "HAT without MAC";
        return false;
    }
    const int64_t now = bootMs();
    if (needChallenge) {
        mChallenges.erase(std::remove_if(mChallenges.begin(), mChallenges.end(),
                                         [&](const Challenge& c) {
                                             return now - c.createdMs > kChallengeTimeoutMs;
                                         }),
                          mChallenges.end());
        const bool known = std::any_of(mChallenges.begin(), mChallenges.end(),
                                       [&](const Challenge& c) { return c.value == hat.challenge; });
        if (!known) {
            LOG(ERROR) << "HAT challenge unknown or expired";
            return false;
        }
    }
    const int64_t ts = hat.timestamp.milliSeconds;
    if (ts > 0 && ts <= now && now - ts > kHatMaxAgeMs) {
        LOG(ERROR) << "HAT too old (" << (now - ts) / 1000 << " s)";
        return false;
    }
    return true;
}

void Session::scheduleLockoutCleared(int64_t delayMs) {
    {
        std::lock_guard<std::mutex> lock(mLock);
        mTimerDeadlineMs = bootMs() + delayMs;
    }
    mTimerCond.notify_all();
}

bool Session::rateLimit(AcquiredInfo info, int32_t vendorCode) {
    const int64_t now = bootMs();
    if (info == mLastInfo && vendorCode == mLastVendor &&
        now - mLastInfoMs < kFrameMessageIntervalMs * 5) {
        return false;
    }
    if (now - mLastInfoMs < kFrameMessageIntervalMs && info != AcquiredInfo::GOOD &&
        info != AcquiredInfo::START && info != AcquiredInfo::FIRST_FRAME_RECEIVED) {
        return false;
    }
    mLastInfo = info;
    mLastVendor = vendorCode;
    mLastInfoMs = now;
    return true;
}

void Session::sendEnrollFrame(AcquiredInfo info, int32_t vendorCode, EnrollmentStage stage) {
    if (!rateLimit(info, vendorCode)) return;
    EnrollmentFrame frame;
    frame.stage = stage;
    frame.data.acquiredInfo = info;
    frame.data.vendorCode = vendorCode;
    frame.data.isCancellable = true;
    mCb->onEnrollmentFrame(frame);
}

void Session::sendAuthFrame(AcquiredInfo info, int32_t vendorCode) {
    if (!rateLimit(info, vendorCode)) return;
    AuthenticationFrame frame;
    frame.data.acquiredInfo = info;
    frame.data.vendorCode = vendorCode;
    frame.data.isCancellable = true;
    mCb->onAuthenticationFrame(frame);
}

// ------------------------------------------------------------------------------- operations

ndk::ScopedAStatus Session::generateChallenge() {
    post([this](uint64_t) {
        const int64_t now = bootMs();
        Challenge c{randomNonZero(), now};
        if (mChallenges.size() >= size_t(kMaxChallenges)) mChallenges.erase(mChallenges.begin());
        mChallenges.push_back(c);
        mCb->onChallengeGenerated(c.value);
    });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::revokeChallenge(int64_t challenge) {
    post([this, challenge](uint64_t) {
        mChallenges.erase(std::remove_if(mChallenges.begin(), mChallenges.end(),
                                         [&](const Challenge& c) { return c.value == challenge; }),
                          mChallenges.end());
        mCb->onChallengeRevoked(challenge);
    });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::getEnrollmentConfig(EnrollmentType,
                                                std::vector<EnrollmentStageConfig>* out) {
    *out = {};
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::enroll(const keymaster::HardwareAuthToken& hat, EnrollmentType,
                                   const std::vector<Feature>& features,
                                   const std::optional<NativeHandle>&,
                                   std::shared_ptr<common::ICancellationSignal>* out) {
    // The deprecated NativeHandle preview is not supported (Settings passes a Surface through
    // enrollWithOptions on Android 16).
    *out = post([this, hat, features](uint64_t id) { doEnroll(id, hat, features, nullptr); });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::enrollWithContext(const keymaster::HardwareAuthToken& hat,
                                              EnrollmentType type,
                                              const std::vector<Feature>& features,
                                              const std::optional<NativeHandle>& preview,
                                              const common::OperationContext&,
                                              std::shared_ptr<common::ICancellationSignal>* out) {
    return enroll(hat, type, features, preview, out);
}

ndk::ScopedAStatus Session::enrollWithOptions(const FaceEnrollOptions& options,
                                              std::shared_ptr<common::ICancellationSignal>* out) {
    const auto hat = options.hardwareAuthToken;
    const auto features = options.features;
    // Settings' square preview Surface: the HAL draws the IR enrollment preview into it.
    std::shared_ptr<ANativeWindow> preview;
    if (options.surfacePreview && options.surfacePreview->get()) {
        ANativeWindow* w = options.surfacePreview->get();
        ANativeWindow_acquire(w);
        preview.reset(w, ANativeWindow_release);
    }
    *out = post([this, hat, features, preview](uint64_t id) {
        doEnroll(id, hat, features, preview);
    });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::authenticate(int64_t operationId,
                                         std::shared_ptr<common::ICancellationSignal>* out) {
    *out = post([this, operationId](uint64_t id) { doAuthenticate(id, operationId); });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::authenticateWithContext(
        int64_t operationId, const common::OperationContext&,
        std::shared_ptr<common::ICancellationSignal>* out) {
    return authenticate(operationId, out);
}

ndk::ScopedAStatus Session::detectInteraction(std::shared_ptr<common::ICancellationSignal>* out) {
    // SensorProps.supportsDetectInteraction is false.
    *out = post([this](uint64_t) { mCb->onError(Error::UNABLE_TO_PROCESS, 0); });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::detectInteractionWithContext(
        const common::OperationContext&, std::shared_ptr<common::ICancellationSignal>* out) {
    return detectInteraction(out);
}

ndk::ScopedAStatus Session::onContextChanged(const common::OperationContext&) {
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::enumerateEnrollments() {
    post([this](uint64_t) {
        TemplateStore store(mUserId, &mCtx->sealer);
        mCb->onEnrollmentsEnumerated(store.enrollmentIds());
    });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::removeEnrollments(const std::vector<int32_t>& ids) {
    post([this, ids](uint64_t) {
        TemplateStore store(mUserId, &mCtx->sealer);
        std::vector<int32_t> removed;
        for (int32_t id : ids) {
            if (store.remove(id)) removed.push_back(id);
        }
        if (store.enrollmentIds().empty()) {
            UserState state = store.loadState();
            state.authenticatorId = 0;
            state.features = 0;
            store.saveState(state);
        }
        if (removed.size() != ids.size()) {
            mCb->onError(Error::UNABLE_TO_REMOVE, 0);
        }
        mCb->onEnrollmentsRemoved(removed);
    });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::getFeatures() {
    post([this](uint64_t) {
        TemplateStore store(mUserId, &mCtx->sealer);
        if (store.enrollmentIds().empty()) {
            mCb->onError(Error::UNABLE_TO_PROCESS, 0);
            return;
        }
        const UserState state = store.loadState();
        std::vector<Feature> features;
        for (Feature f : {Feature::REQUIRE_ATTENTION, Feature::REQUIRE_DIVERSE_POSES,
                          Feature::DEBUG}) {
            if (state.features & (1u << static_cast<int>(f))) features.push_back(f);
        }
        mCb->onFeaturesRetrieved(features);
    });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::setFeature(const keymaster::HardwareAuthToken& hat, Feature feature,
                                       bool enabled) {
    post([this, hat, feature, enabled](uint64_t) {
        TemplateStore store(mUserId, &mCtx->sealer);
        if (!checkHat(hat, false) || store.enrollmentIds().empty()) {
            mCb->onError(Error::UNABLE_TO_PROCESS, 0);
            return;
        }
        UserState state = store.loadState();
        const uint32_t bit = 1u << static_cast<int>(feature);
        state.features = enabled ? (state.features | bit) : (state.features & ~bit);
        if (!store.saveState(state)) {
            mCb->onError(Error::UNABLE_TO_PROCESS, 0);
            return;
        }
        mCb->onFeatureSet(feature);
    });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::getAuthenticatorId() {
    post([this](uint64_t) {
        TemplateStore store(mUserId, &mCtx->sealer);
        const int64_t id =
                store.enrollmentIds().empty() ? 0 : store.loadState().authenticatorId;
        mCb->onAuthenticatorIdRetrieved(id);
    });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::invalidateAuthenticatorId() {
    post([this](uint64_t) {
        TemplateStore store(mUserId, &mCtx->sealer);
        UserState state = store.loadState();
        state.authenticatorId = store.enrollmentIds().empty() ? 0 : randomNonZero();
        store.saveState(state);
        mCb->onAuthenticatorIdInvalidated(state.authenticatorId);
    });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::resetLockout(const keymaster::HardwareAuthToken& hat) {
    post([this, hat](uint64_t) {
        if (!checkHat(hat, false)) {
            mCb->onError(Error::UNABLE_TO_PROCESS, 0);
            return;
        }
        TemplateStore store(mUserId, &mCtx->sealer);
        UserState state = store.loadState();
        state.failedAttempts = 0;
        state.lockedUntilRealtimeMs = 0;
        state.permanentLockout = false;
        store.saveState(state);
        mCb->onLockoutCleared();
    });
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Session::close() {
    if (mClosed.exchange(true)) return ndk::ScopedAStatus::ok();
    LOG(INFO) << "session of user " << mUserId << " closed";
    {
        std::lock_guard<std::mutex> lock(mLock);
        for (const auto& q : mQueue) mCancelled.insert(q.first);
        if (mCurrentOp) mCancelled.insert(mCurrentOp);
        if (mCapture) mCapture->interrupt();
        mTimerDeadlineMs = 0;
    }
    mTimerCond.notify_all();
    // Let the queued operations end (as cancelled), then report the close from the worker.
    post([this](uint64_t) { mCb->onSessionClosed(); });
    return ndk::ScopedAStatus::ok();
}

// ------------------------------------------------------------------------------ enrollment

namespace {
// Upper bound of an operation, for the time-limited frequency lock.
constexpr int kOpBoostMs = kEnrollTimeoutMs + 2000;  // enrollment; authentication uses its own
}  // namespace

void Session::doEnroll(uint64_t opId, const keymaster::HardwareAuthToken& hat,
                       const std::vector<Feature>& features,
                       std::shared_ptr<ANativeWindow> preview) {
    if (opCancelled(opId)) {
        mCb->onError(Error::CANCELED, 0);
        return;
    }
    if (!checkHat(hat, true)) {
        mCb->onError(Error::UNABLE_TO_PROCESS, 0);
        return;
    }
    TemplateStore store(mUserId, &mCtx->sealer);
    if (!store.usable() || !mCtx->engine) {
        LOG(ERROR) << "template store " << store.dir() << " or engine unusable";
        mCb->onError(Error::UNABLE_TO_PROCESS, 0);
        return;
    }
    if (store.enrollmentIds().size() >= size_t(kMaxEnrollmentsPerUser)) {
        mCb->onError(Error::NO_SPACE, 0);
        return;
    }
    const int32_t enrollmentId = store.newEnrollmentId();

    std::lock_guard<std::mutex> captureLock(mCtx->captureLock);
    IrisCapture capture(mCtx->led.get(), mCtx->engine, mCtx->engineContext);
    // Analysis on the A57 cluster, two A57 online, A57 floor ~1.25 GHz: until the end of the
    // operation whatever the way out.
    ScopedCpuBoost boost(mCtx->boost, kOpBoostMs);
    {
        std::lock_guard<std::mutex> lock(mLock);
        mCapture = &capture;
    }
    auto finish = [&] {
        std::lock_guard<std::mutex> lock(mLock);
        mCapture = nullptr;
    };
    std::string error;
    if (opCancelled(opId) || !capture.start(&error)) {
        if (!error.empty()) LOG(ERROR) << "enroll: " << error;
        capture.stop();
        finish();
        mCb->onError(opCancelled(opId) ? Error::CANCELED : Error::HW_UNAVAILABLE, 0);
        return;
    }
    mLastInfoMs = 0;
    sendEnrollFrame(AcquiredInfo::START, 0, EnrollmentStage::UNKNOWN);

    // Enrollment preview. Rotation: the sensor orientation unless overridden (the real
    // orientation of the iris module still has to be checked on the phone); mirrored like a
    // front camera.
    std::unique_ptr<PreviewSink> sink;
    if (preview) sink = std::make_unique<PreviewSink>(preview.get());
    int rotation = ::android::base::GetIntProperty("persist.vendor.iris.preview_rotation", -1);
    if (rotation != 0 && rotation != 90 && rotation != 180 && rotation != 270) {
        rotation = capture.sensorOrientation();
    }
    const bool mirror = ::android::base::GetBoolProperty("persist.vendor.iris.preview_mirror", true);

    std::vector<TemplatePtr> samples[2];
    bool firstFrame = false;
    int lastReported = -1;
    const int64_t deadline = bootMs() + kEnrollTimeoutMs;
    std::vector<Probe> probes;
    Error failure = Error::UNKNOWN;
    bool done = false;

    while (!done) {
        const int64_t left = deadline - bootMs();
        if (left <= 0) {
            failure = Error::TIMEOUT;
            break;
        }
        const auto res = capture.next(int(left), &probes);
        if (!firstFrame && capture.firstFrameSeen()) {
            firstFrame = true;
            sendEnrollFrame(AcquiredInfo::FIRST_FRAME_RECEIVED, 0,
                            EnrollmentStage::FIRST_FRAME_RECEIVED);
        }
        if (res != IrisCapture::Result::kProbes) {
            failure = res == IrisCapture::Result::kTimeout       ? Error::TIMEOUT
                      : res == IrisCapture::Result::kInterrupted ? Error::CANCELED
                      : res == IrisCapture::Result::kLedUnavailable
                              ? Error::HW_UNAVAILABLE
                              : Error::HW_UNAVAILABLE;
            if (res == IrisCapture::Result::kLedUnavailable) {
                sendEnrollFrame(AcquiredInfo::VENDOR, kVendorLedCooldown,
                                EnrollmentStage::WAITING_FOR_CENTERING);
            }
            break;
        }
        if (opCancelled(opId)) {
            failure = Error::CANCELED;
            break;
        }
        const auto [info, vendor] = acquiredFor(probes);
        bool added = false;
        for (auto& p : probes) {
            if (p.status != IRIS_OK || !p.tmpl) continue;
            auto& v = samples[p.side & 1];
            v.push_back(std::move(p.tmpl));
            if (v.size() > size_t(kEnrollMaxSamples)) v.erase(v.begin());
            added = true;
        }
        if (sink) {
            PreviewEye eyes[2];
            for (const auto& p : probes) {
                PreviewEye& e = eyes[p.side & 1];
                e.found = p.quality.eye_found != 0 && p.quality.iris_r > 0.f;
                e.good = p.status == IRIS_OK;
                e.irisX = p.quality.iris_x;
                e.irisY = p.quality.iris_y;
                e.irisR = p.quality.iris_r;
                e.pupilX = p.quality.pupil_x;
                e.pupilY = p.quality.pupil_y;
                e.pupilR = p.quality.pupil_r;
            }
            for (int s = 0; s < 2; s++) {
                eyes[s].progress = std::min(1.f, float(samples[s].size()) / kEnrollTargetSamples);
            }
            sink->submit(capture.lastLit(), eyes, rotation, mirror);
        }
        sendEnrollFrame(info, vendor,
                        added ? EnrollmentStage::HOLD_STILL_IN_CENTER
                              : EnrollmentStage::WAITING_FOR_CENTERING);
        if (!added) continue;

        const int best = int(std::max(samples[0].size(), samples[1].size()));
        const int remaining = std::max(0, kEnrollTargetSamples - best);
        if (remaining > 0) {
            if (remaining != lastReported) {
                lastReported = remaining;
                mCb->onEnrollmentProgress(enrollmentId, remaining);
            }
            continue;
        }
        // Build one template per eye that has enough mutually consistent samples.
        iris_config cfg{};
        iris_engine_get_config(mCtx->engine, &cfg);
        Enrollment e;
        e.id = enrollmentId;
        for (auto& side : samples) {
            if (side.size() < cfg.enroll_min_samples) continue;
            std::vector<const iris_template*> ptrs;
            for (const auto& t : side) ptrs.push_back(t.get());
            iris_template* out = nullptr;
            iris_enroll_info einfo{};
            const iris_status st =
                    iris_enroll(mCtx->engine, ptrs.data(), ptrs.size(), &out, &einfo);
            TemplatePtr tmpl(out);
            if (st != IRIS_OK) {
                LOG(INFO) << "enroll: side not consistent yet (" << iris_status_string(st)
                          << ", " << einfo.samples_used << "/" << einfo.samples_in << ")";
                continue;
            }
            size_t size = 0;
            iris_template_serialize(tmpl.get(), nullptr, 0, &size);
            std::vector<uint8_t> buf(size);
            if (iris_template_serialize(tmpl.get(), buf.data(), buf.size(), &size) != IRIS_OK) {
                continue;
            }
            buf.resize(size);
            e.templates.push_back(std::move(buf));
            LOG(INFO) << "enroll: eye template from " << einfo.samples_used << " samples, "
                      << einfo.codes_stored << " codes, mean intra score "
                      << einfo.mean_intra_score;
        }
        if (e.templates.empty()) continue;  // keep collecting (oldest samples roll out)

        UserState state = store.loadState();
        state.failedAttempts = 0;
        state.lockedUntilRealtimeMs = 0;
        state.permanentLockout = false;
        state.authenticatorId = randomNonZero();
        state.features = 0;
        for (Feature f : features) state.features |= 1u << static_cast<int>(f);
        if (!store.save(e) || !store.saveState(state)) {
            store.remove(e.id);
            failure = Error::UNABLE_TO_PROCESS;
            break;
        }
        done = true;
    }
    capture.stop();
    finish();
    if (done) {
        LOG(INFO) << "enrolled " << enrollmentId << " for user " << mUserId;
        sendEnrollFrame(AcquiredInfo::GOOD, 0, EnrollmentStage::ENROLLMENT_FINISHED);
        mCb->onEnrollmentProgress(enrollmentId, 0);
    } else {
        LOG(INFO) << "enroll failed (" << face::toString(failure) << ")";
        mCb->onError(failure, 0);
    }
}

// --------------------------------------------------------------------------- authentication

bool Session::checkLockout(TemplateStore& store, UserState& state) {
    if (state.permanentLockout) {
        mCb->onLockoutPermanent();
        return true;
    }
    if (state.lockedUntilRealtimeMs != 0) {
        const int64_t left = state.lockedUntilRealtimeMs - wallMs();
        if (left > 0 && left <= kLockoutTimedDurationMs) {
            mCb->onLockoutTimed(left);
            return true;
        }
        // Expired (or a clock jump made it implausible): clear it.
        state.lockedUntilRealtimeMs = 0;
        store.saveState(state);
        mCb->onLockoutCleared();
    }
    return false;
}

void Session::recordFailure(TemplateStore& store, UserState& state) {
    state.failedAttempts++;
    if (state.failedAttempts >= uint32_t(kLockoutPermanentThreshold)) {
        state.permanentLockout = true;
        store.saveState(state);
        mCb->onLockoutPermanent();
    } else if (state.failedAttempts % kLockoutTimedThreshold == 0) {
        state.lockedUntilRealtimeMs = wallMs() + kLockoutTimedDurationMs;
        store.saveState(state);
        mCb->onLockoutTimed(kLockoutTimedDurationMs);
        scheduleLockoutCleared(kLockoutTimedDurationMs);
    } else {
        store.saveState(state);
    }
    mCb->onAuthenticationFailed();
}

void Session::doAuthenticate(uint64_t opId, int64_t operationId) {
    if (opCancelled(opId)) {
        mCb->onError(Error::CANCELED, 0);
        return;
    }
    TemplateStore store(mUserId, &mCtx->sealer);
    if (!store.usable() || !mCtx->engine) {
        mCb->onError(Error::UNABLE_TO_PROCESS, 0);
        return;
    }
    UserState state = store.loadState();
    if (checkLockout(store, state)) return;

    struct Gallery {
        int32_t id;
        TemplatePtr tmpl;
    };
    std::vector<Gallery> gallery;
    for (auto& e : store.loadAll()) {
        for (const auto& buf : e.templates) {
            iris_template* t = nullptr;
            if (iris_template_deserialize(buf.data(), buf.size(), &t) == IRIS_OK) {
                gallery.push_back({e.id, TemplatePtr(t)});
            }
        }
    }
    if (gallery.empty()) {
        LOG(ERROR) << "authenticate: no usable enrollment";
        mCb->onError(Error::UNABLE_TO_PROCESS, 0);
        return;
    }

    std::lock_guard<std::mutex> captureLock(mCtx->captureLock);
    IrisCapture capture(mCtx->led.get(), mCtx->engine, mCtx->engineContext);
    // Analysis on the A57 cluster, two A57 online, A57 floor ~1.25 GHz: until the end of the
    // operation whatever the way out.
    ScopedCpuBoost boost(mCtx->boost, kAuthTimeoutMs + 2000);
    {
        std::lock_guard<std::mutex> lock(mLock);
        mCapture = &capture;
    }
    auto finish = [&] {
        capture.stop();
        std::lock_guard<std::mutex> lock(mLock);
        mCapture = nullptr;
    };
    std::string error;
    if (opCancelled(opId) || !capture.start(&error)) {
        if (!error.empty()) LOG(ERROR) << "authenticate: " << error;
        finish();
        mCb->onError(opCancelled(opId) ? Error::CANCELED : Error::HW_UNAVAILABLE, 0);
        return;
    }
    mLastInfoMs = 0;
    sendAuthFrame(AcquiredInfo::START, 0);

    const int64_t deadline = bootMs() + kAuthTimeoutMs;
    bool firstFrame = false;
    int validFrames = 0;
    float bestScore = 1.f;
    std::vector<Probe> probes;
    for (;;) {
        const int64_t left = deadline - bootMs();
        const auto res = left > 0 ? capture.next(int(left), &probes)
                                  : IrisCapture::Result::kTimeout;
        if (!firstFrame && capture.firstFrameSeen()) {
            firstFrame = true;
            sendAuthFrame(AcquiredInfo::FIRST_FRAME_RECEIVED, 0);
        }
        if (res != IrisCapture::Result::kProbes || opCancelled(opId)) {
            finish();
            if (opCancelled(opId) || res == IrisCapture::Result::kInterrupted) {
                mCb->onError(Error::CANCELED, 0);
            } else if (res == IrisCapture::Result::kTimeout && validFrames > 0) {
                LOG(INFO) << "authenticate: no match, best score " << bestScore;
                recordFailure(store, state);
            } else if (res == IrisCapture::Result::kTimeout) {
                mCb->onError(Error::TIMEOUT, 0);
            } else {
                if (res == IrisCapture::Result::kLedUnavailable) {
                    sendAuthFrame(AcquiredInfo::VENDOR, kVendorLedCooldown);
                }
                mCb->onError(Error::HW_UNAVAILABLE, 0);
            }
            return;
        }
        const auto [info, vendor] = acquiredFor(probes);
        bool valid = false;
        for (const auto& p : probes) {
            if (p.status != IRIS_OK || !p.tmpl) continue;
            valid = true;
            for (const auto& g : gallery) {
                int32_t accepted = 0;
                float score = 1.f;
                if (iris_verify(mCtx->engine, p.tmpl.get(), g.tmpl.get(), &accepted, &score) !=
                    IRIS_OK) {
                    continue;
                }
                bestScore = std::min(bestScore, score);
                if (!accepted) continue;
                finish();
                LOG(INFO) << "authenticated enrollment " << g.id << " (score " << score << ")";
                state.failedAttempts = 0;
                state.lockedUntilRealtimeMs = 0;
                store.saveState(state);
                keymaster::HardwareAuthToken hat;
                hat.challenge = operationId;
                hat.authenticatorId = state.authenticatorId;
                hat.authenticatorType = keymaster::HardwareAuthenticatorType::NONE;
                hat.timestamp.milliSeconds = bootMs();
                // Weak sensor: the token is not MACed and keystore does not accept it.
                mCb->onAuthenticationSucceeded(g.id, hat);
                return;
            }
        }
        sendAuthFrame(info, vendor);
        if (valid && ++validFrames >= kAuthProbesForReject) {
            finish();
            LOG(INFO) << "authenticate: " << validFrames << " good frames, no match, best score "
                      << bestScore;
            recordFailure(store, state);
            return;
        }
    }
}

std::string Session::toString() const {
    std::ostringstream os;
    os << "session user " << mUserId << (mClosed ? " (closed)" : "") << "\n";
    return os.str();
}

}  // namespace aidl::android::hardware::biometrics::face
