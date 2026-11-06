/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <memory>
#include <mutex>

#include <aidl/android/hardware/biometrics/face/BnFace.h>

#include "Session.h"

namespace aidl::android::hardware::biometrics::face {

// IFace/default of the Lumia 950 XL: iris recognition (IR camera + IR LED) presented to the
// framework as a weak (class 2) IR face sensor.
class Face : public BnFace {
  public:
    Face();
    ~Face() override;

    ndk::ScopedAStatus getSensorProps(std::vector<SensorProps>* out) override;
    ndk::ScopedAStatus createSession(int32_t sensorId, int32_t userId,
                                     const std::shared_ptr<ISessionCallback>& cb,
                                     std::shared_ptr<ISession>* out) override;
    binder_status_t dump(int fd, const char** args, uint32_t numArgs) override;

  private:
    IrisContext mCtx;
    std::mutex mLock;
    std::shared_ptr<Session> mSession;
};

}  // namespace aidl::android::hardware::biometrics::face
