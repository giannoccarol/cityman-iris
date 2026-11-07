/*
 * SPDX-FileCopyrightText: 2026 giannoccarol
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_TAG "cityman.iris"

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>

#include "Face.h"

using aidl::android::hardware::biometrics::face::Face;

int main() {
    android::base::InitLogging(nullptr, android::base::LogdLogger(android::base::SYSTEM));
    // The camera service and the image reader call back over binder: the pool must run, or no
    // frame ever arrives (measured with iris_grab).
    ABinderProcess_setThreadPoolMaxThreadCount(4);
    ABinderProcess_startThreadPool();

    std::shared_ptr<Face> hal = ndk::SharedRefBase::make<Face>();
    const std::string instance = std::string(Face::descriptor) + "/default";
    const binder_status_t status = AServiceManager_addService(hal->asBinder().get(),
                                                              instance.c_str());
    CHECK_EQ(status, STATUS_OK) << "cannot register " << instance;
    LOG(INFO) << "registered " << instance;
    ABinderProcess_joinThreadPool();
    return EXIT_FAILURE;  // should not reach here
}
