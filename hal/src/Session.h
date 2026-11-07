/*
 * SPDX-FileCopyrightText: 2026 giannoccarol
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <aidl/android/hardware/biometrics/common/BnCancellationSignal.h>
#include <aidl/android/hardware/biometrics/face/BnSession.h>
#include <aidl/android/hardware/biometrics/face/FaceEnrollOptions.h>
#include <aidl/android/hardware/biometrics/face/ISessionCallback.h>

#include "CpuBoost.h"
#include "IrLed.h"
#include "PreviewSink.h"
#include "IrisPipeline.h"
#include "TemplateStore.h"
#include "iris_engine.h"

namespace aidl::android::hardware::biometrics::face {

namespace common = aidl::android::hardware::biometrics::common;
namespace keymaster = aidl::android::hardware::keymaster;
using aidl::android::hardware::common::NativeHandle;

// Resources shared by every session of the process (one IR LED, one camera, one engine).
struct IrisContext {
    std::unique_ptr<cityman::iris::IrLed> led;
    cityman::iris::Sealer sealer;
    iris_engine* engine = nullptr;
    iris_context* engineContext = nullptr;  // engine buffers, used under captureLock
    cityman::iris::CpuBoost boost;           // used under captureLock
    std::mutex captureLock;  // one camera operation at a time
};

class Session : public BnSession {
  public:
    Session(int32_t userId, std::shared_ptr<ISessionCallback> cb, IrisContext* ctx);
    ~Session() override;

    ndk::ScopedAStatus generateChallenge() override;
    ndk::ScopedAStatus revokeChallenge(int64_t challenge) override;
    ndk::ScopedAStatus getEnrollmentConfig(EnrollmentType enrollmentType,
                                           std::vector<EnrollmentStageConfig>* out) override;
    ndk::ScopedAStatus enroll(const keymaster::HardwareAuthToken& hat, EnrollmentType type,
                              const std::vector<Feature>& features,
                              const std::optional<NativeHandle>& previewSurface,
                              std::shared_ptr<common::ICancellationSignal>* out) override;
    ndk::ScopedAStatus authenticate(int64_t operationId,
                                    std::shared_ptr<common::ICancellationSignal>* out) override;
    ndk::ScopedAStatus detectInteraction(
            std::shared_ptr<common::ICancellationSignal>* out) override;
    ndk::ScopedAStatus enumerateEnrollments() override;
    ndk::ScopedAStatus removeEnrollments(const std::vector<int32_t>& enrollmentIds) override;
    ndk::ScopedAStatus getFeatures() override;
    ndk::ScopedAStatus setFeature(const keymaster::HardwareAuthToken& hat, Feature feature,
                                  bool enabled) override;
    ndk::ScopedAStatus getAuthenticatorId() override;
    ndk::ScopedAStatus invalidateAuthenticatorId() override;
    ndk::ScopedAStatus resetLockout(const keymaster::HardwareAuthToken& hat) override;
    ndk::ScopedAStatus close() override;
    ndk::ScopedAStatus authenticateWithContext(
            int64_t operationId, const common::OperationContext& context,
            std::shared_ptr<common::ICancellationSignal>* out) override;
    ndk::ScopedAStatus enrollWithContext(const keymaster::HardwareAuthToken& hat,
                                         EnrollmentType type, const std::vector<Feature>& features,
                                         const std::optional<NativeHandle>& previewSurface,
                                         const common::OperationContext& context,
                                         std::shared_ptr<common::ICancellationSignal>* out) override;
    ndk::ScopedAStatus detectInteractionWithContext(
            const common::OperationContext& context,
            std::shared_ptr<common::ICancellationSignal>* out) override;
    ndk::ScopedAStatus onContextChanged(const common::OperationContext& context) override;
    ndk::ScopedAStatus enrollWithOptions(const FaceEnrollOptions& options,
                                         std::shared_ptr<common::ICancellationSignal>* out) override;

    binder_status_t linkToDeath(AIBinder* binder);
    void cancelOperation(uint64_t opId);
    bool isClosed() const { return mClosed; }
    std::string toString() const;

  private:
    using Task = std::function<void(uint64_t opId)>;

    std::shared_ptr<common::ICancellationSignal> post(Task task);
    void workerLoop();
    bool opCancelled(uint64_t opId);
    bool checkHat(const keymaster::HardwareAuthToken& hat, bool needChallenge);
    void scheduleLockoutCleared(int64_t delayMs);

    void doEnroll(uint64_t opId, const keymaster::HardwareAuthToken& hat,
                  const std::vector<Feature>& features, std::shared_ptr<ANativeWindow> preview);
    void doAuthenticate(uint64_t opId, int64_t operationId);
    bool checkLockout(cityman::iris::TemplateStore& store, cityman::iris::UserState& state);
    void recordFailure(cityman::iris::TemplateStore& store, cityman::iris::UserState& state);

    void sendEnrollFrame(AcquiredInfo info, int32_t vendorCode, EnrollmentStage stage);
    void sendAuthFrame(AcquiredInfo info, int32_t vendorCode);
    bool rateLimit(AcquiredInfo info, int32_t vendorCode);

    const int32_t mUserId;
    const std::shared_ptr<ISessionCallback> mCb;
    IrisContext* const mCtx;
    AIBinder_DeathRecipient* mDeathRecipient = nullptr;

    std::mutex mLock;
    std::condition_variable mCond;
    std::deque<std::pair<uint64_t, Task>> mQueue;
    std::thread mWorker;
    bool mStopWorker = false;
    std::atomic<bool> mClosed{false};
    uint64_t mNextOpId = 1;
    uint64_t mCurrentOp = 0;
    std::set<uint64_t> mCancelled;
    cityman::iris::IrisCapture* mCapture = nullptr;  // running capture, for cancel

    struct Challenge {
        int64_t value;
        int64_t createdMs;
    };
    std::vector<Challenge> mChallenges;  // worker thread only

    // Lockout-cleared timer.
    std::thread mTimer;
    std::condition_variable mTimerCond;
    int64_t mTimerDeadlineMs = 0;

    // Acquired-info rate limit.
    AcquiredInfo mLastInfo = AcquiredInfo::UNKNOWN;
    int32_t mLastVendor = -1;
    int64_t mLastInfoMs = 0;
};

class CancellationSignal : public common::BnCancellationSignal {
  public:
    CancellationSignal(std::weak_ptr<Session> session, uint64_t opId)
        : mSession(std::move(session)), mOpId(opId) {}
    ndk::ScopedAStatus cancel() override;

  private:
    std::weak_ptr<Session> mSession;
    const uint64_t mOpId;
};

}  // namespace aidl::android::hardware::biometrics::face
