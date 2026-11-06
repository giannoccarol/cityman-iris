/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_TAG "cityman.iris.face"

#include "Face.h"

#include <cstdlib>
#include <string>
#include <vector>

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/stringprintf.h>

#include "Config.h"

namespace aidl::android::hardware::biometrics::face {

using namespace ::cityman::iris;

namespace {

float floatProperty(const char* name, float def, float lo, float hi) {
    const std::string v = ::android::base::GetProperty(name, "");
    if (v.empty()) return def;
    char* end = nullptr;
    const float f = std::strtof(v.c_str(), &end);
    if (end == v.c_str() || *end != '\0' || f < lo || f > hi) {
        LOG(WARNING) << name << "=" << v << " ignored (range " << lo << ".." << hi << ")";
        return def;
    }
    return f;
}

}  // namespace

Face::Face() {
    mCtx.led = std::make_unique<IrLed>(
            std::vector<std::string>(std::begin(kLedBrightnessPaths), std::end(kLedBrightnessPaths)),
            std::vector<std::string>(std::begin(kLedMaxBrightnessPaths),
                                     std::end(kLedMaxBrightnessPaths)));
    if (!mCtx.sealer.init(kSealKeyPath)) LOG(ERROR) << "no template sealing key";
    CpuBoost::recoverAtStartup();

    iris_config cfg;
    iris_config_default(&cfg);
    // Geometry and threshold depend on the optics and on real data: tunable, inside sane ranges.
    cfg.min_iris_radius =
            floatProperty("persist.vendor.iris.min_radius", cfg.min_iris_radius, 20.f, 200.f);
    cfg.max_iris_radius =
            floatProperty("persist.vendor.iris.max_radius", cfg.max_iris_radius, 40.f, 400.f);
    // Never looser than 0.40: above that unrelated irises start to match.
    cfg.match_threshold =
            floatProperty("persist.vendor.iris.threshold", cfg.match_threshold, 0.20f, 0.40f);
    if (iris_engine_create(&cfg, &mCtx.engine) != IRIS_OK) {
        LOG(ERROR) << "inconsistent engine config, using the defaults";
        iris_engine_create(nullptr, &mCtx.engine);
    }
    if (mCtx.engine && iris_context_create(mCtx.engine, &mCtx.engineContext) != IRIS_OK) {
        mCtx.engineContext = nullptr;  // iris_extract_ex then allocates per call
    }
    LOG(INFO) << "iris face HAL ready: engine API " << iris_engine_api_version() << ", "
              << mCtx.led->toString();
}

Face::~Face() {
    iris_context_destroy(mCtx.engineContext);
    if (mCtx.engine) iris_engine_destroy(mCtx.engine);
}

ndk::ScopedAStatus Face::getSensorProps(std::vector<SensorProps>* out) {
    common::ComponentInfo hw;
    hw.componentId = "irisSensor";
    hw.hardwareVersion = "msft/cityman/ck71ir";  // Toshiba 0x0861 + PMI8994 flash IR emitter
    hw.firmwareVersion = "";
    hw.serialNumber = "";
    hw.softwareVersion = "";
    common::ComponentInfo sw;
    sw.componentId = "matchingAlgorithm";
    sw.hardwareVersion = "";
    sw.firmwareVersion = "";
    sw.serialNumber = "";
    sw.softwareVersion = ::android::base::StringPrintf("cityman-iris engine %d, template %d",
                                                     iris_engine_api_version(),
                                                     IRIS_TEMPLATE_FORMAT_VERSION);

    common::CommonProps commonProps;
    commonProps.sensorId = kSensorId;
    commonProps.sensorStrength = common::SensorStrength::WEAK;
    commonProps.maxEnrollmentsPerUser = kMaxEnrollmentsPerUser;
    commonProps.componentInfo = {std::move(hw), std::move(sw)};

    SensorProps props;
    props.commonProps = std::move(commonProps);
    props.sensorType = FaceSensorType::IR;
    props.halControlsPreview = false;
    props.supportsDetectInteraction = false;
    *out = {std::move(props)};
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Face::createSession(int32_t sensorId, int32_t userId,
                                       const std::shared_ptr<ISessionCallback>& cb,
                                       std::shared_ptr<ISession>* out) {
    if (sensorId != kSensorId || !cb) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    std::shared_ptr<Session> old;
    {
        std::lock_guard<std::mutex> lock(mLock);
        old = std::move(mSession);
        mSession = ndk::SharedRefBase::make<Session>(userId, cb, &mCtx);
        mSession->linkToDeath(cb->asBinder().get());
        *out = mSession;
    }
    if (old && !old->isClosed()) old->close();
    LOG(INFO) << "session for user " << userId;
    return ndk::ScopedAStatus::ok();
}

binder_status_t Face::dump(int fd, const char**, uint32_t) {
    std::vector<SensorProps> props;
    getSensorProps(&props);
    std::string out = "cityman iris face HAL\n";
    for (const auto& p : props) out += p.toString() + "\n";
    out += mCtx.led->toString();
    {
        std::lock_guard<std::mutex> lock(mLock);
        out += mSession ? mSession->toString() : "no session\n";
    }
    ::android::base::WriteStringToFd(out, fd);
    return STATUS_OK;
}

}  // namespace aidl::android::hardware::biometrics::face
