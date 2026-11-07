/*
 * SPDX-FileCopyrightText: 2026 giannoccarol
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_TAG "cityman.iris.camera"

#include "IrisCamera.h"

#include <chrono>
#include <cstdlib>
#include <cstring>

#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/strings.h>
#include <camera/NdkCameraMetadata.h>

#include "Config.h"

namespace cityman::iris {

namespace {

constexpr size_t kQueueFrames = 4;
constexpr size_t kMetaEntries = 16;

bool hasCapability(const ACameraMetadata* chars, uint8_t cap) {
    ACameraMetadata_const_entry e{};
    if (ACameraMetadata_getConstEntry(chars, ACAMERA_REQUEST_AVAILABLE_CAPABILITIES, &e) !=
        ACAMERA_OK) {
        return false;
    }
    for (uint32_t i = 0; i < e.count; i++) {
        if (e.data.u8[i] == cap) return true;
    }
    return false;
}

std::string firstMonoId() {
    const auto ids =
            android::base::Split(android::base::GetProperty("ro.vendor.camera.mono_ids", ""), ", ");
    for (const auto& id : ids) {
        if (!id.empty()) return id;
    }
    return "";
}

}  // namespace

IrisCamera::IrisCamera() {
    mDeviceCallbacks.context = this;
    mDeviceCallbacks.onDisconnected = &IrisCamera::onDeviceDisconnected;
    mDeviceCallbacks.onError = &IrisCamera::onDeviceError;
    mSessionCallbacks.context = this;
    mSessionCallbacks.onClosed = &IrisCamera::onSessionClosed;
    mSessionCallbacks.onReady = &IrisCamera::onSessionReady;
    mSessionCallbacks.onActive = &IrisCamera::onSessionActive;
    mCaptureCallbacks.context = this;
    mCaptureCallbacks.onCaptureCompleted = &IrisCamera::onCaptureCompleted;
    mCaptureCallbacks.onCaptureStarted = &IrisCamera::onCaptureStarted;
    mListener.context = this;
    mListener.onImageAvailable = &IrisCamera::onImageAvailable;
}

IrisCamera::~IrisCamera() {
    close();
}

std::string IrisCamera::findCameraId(std::string* error) {
    // Explicit override for bring-up.
    const std::string forced = android::base::GetProperty("persist.vendor.iris.camera_id", "");
    ACameraIdList* list = nullptr;
    if (ACameraManager_getCameraIdList(mManager, &list) != ACAMERA_OK || list == nullptr) {
        *error = "getCameraIdList failed (is android.frameworks.cameraservice.service declared?)";
        return "";
    }
    std::string bySystem, all;
    bool forcedFound = false;
    for (int i = 0; i < list->numCameras; i++) {
        const char* id = list->cameraIds[i];
        all += std::string(all.empty() ? "" : ",") + id;
        if (!forced.empty() && forced == id) forcedFound = true;
        ACameraMetadata* chars = nullptr;
        if (ACameraManager_getCameraCharacteristics(mManager, id, &chars) != ACAMERA_OK) continue;
        if (bySystem.empty() &&
            hasCapability(chars, ACAMERA_REQUEST_AVAILABLE_CAPABILITIES_SYSTEM_CAMERA)) {
            bySystem = id;
        }
        ACameraMetadata_free(chars);
    }
    ACameraManager_deleteCameraIdList(list);
    if (!forced.empty()) {
        if (forcedFound) return forced;
        *error = "camera " + forced + " (persist.vendor.iris.camera_id) not found in [" + all + "]";
        return "";
    }
    if (!bySystem.empty()) return bySystem;
    // Fallback: the first camera without a colour filter, if the camera HAL lists it publicly.
    const std::string mono = firstMonoId();
    if (!mono.empty() && ("," + all + ",").find("," + mono + ",") != std::string::npos) {
        LOG(WARNING) << "no SYSTEM_CAMERA, using ro.vendor.camera.mono_ids camera " << mono;
        return mono;
    }
    *error = "no iris camera among [" + all + "] (SYSTEM_CAMERA capability or mono_ids)";
    return "";
}

bool IrisCamera::open(int exposureUs, int iso, std::string* error) {
    close();
    {
        std::lock_guard<std::mutex> lock(mFrameLock);
        mInterrupted = false;
        mQueue.clear();
        mMeta.clear();
    }
    mError = false;
    mSequence = 0;
    mLastStartNs = 0;

    mManager = ACameraManager_create();
    if (!mManager) {
        *error = "ACameraManager_create failed";
        return false;
    }
    mCameraId = findCameraId(error);
    if (mCameraId.empty()) {
        close();
        return false;
    }
    ACameraMetadata* chars = nullptr;
    if (ACameraManager_getCameraCharacteristics(mManager, mCameraId.c_str(), &chars) ==
        ACAMERA_OK) {
        ACameraMetadata_const_entry e{};
        if (ACameraMetadata_getConstEntry(chars, ACAMERA_SENSOR_ORIENTATION, &e) == ACAMERA_OK &&
            e.count) {
            mSensorOrientation = e.data.i32[0];
        }
        ACameraMetadata_free(chars);
    }
    camera_status_t rc =
            ACameraManager_openCamera(mManager, mCameraId.c_str(), &mDeviceCallbacks, &mDevice);
    if (rc != ACAMERA_OK) {
        // CAMERA_IN_USE / MAX_CAMERA_IN_USE: an app holds the front camera (shares CSIPHY2;
        // cost 77 + 32 > 100) and has a higher priority than this vendor client.
        *error = "openCamera(" + mCameraId + ") failed: " + std::to_string(rc);
        close();
        return false;
    }

    media_status_t mrc = AImageReader_new(kFrameWidth, kFrameHeight, AIMAGE_FORMAT_YUV_420_888,
                                          int32_t(kQueueFrames + 2), &mReader);
    if (mrc != AMEDIA_OK || AImageReader_getWindow(mReader, &mReaderWindow) != AMEDIA_OK) {
        *error = "AImageReader setup failed";
        close();
        return false;
    }
    AImageReader_setImageListener(mReader, &mListener);

    if (ACaptureSessionOutputContainer_create(&mOutputs) != ACAMERA_OK ||
        ACaptureSessionOutput_create(mReaderWindow, &mReaderOutput) != ACAMERA_OK ||
        ACaptureSessionOutputContainer_add(mOutputs, mReaderOutput) != ACAMERA_OK) {
        *error = "output container setup failed";
        close();
        return false;
    }
    if (ACameraDevice_createCaptureRequest(mDevice, TEMPLATE_PREVIEW, &mRequest) != ACAMERA_OK ||
        ACameraOutputTarget_create(mReaderWindow, &mReaderTarget) != ACAMERA_OK ||
        ACaptureRequest_addTarget(mRequest, mReaderTarget) != ACAMERA_OK) {
        *error = "capture request setup failed";
        close();
        return false;
    }
    // Manual exposure: the LED, not AE, decides the brightness. The mono effect is forced by the
    // camera HAL (ro.vendor.camera.mono_ids); asking for it here too costs nothing.
    const uint8_t aeOff = ACAMERA_CONTROL_AE_MODE_OFF;
    const uint8_t afOff = ACAMERA_CONTROL_AF_MODE_OFF;
    const uint8_t mono = ACAMERA_CONTROL_EFFECT_MODE_MONO;
    const int64_t frameDuration = kFrameDurationNs;
    ACaptureRequest_setEntry_u8(mRequest, ACAMERA_CONTROL_AE_MODE, 1, &aeOff);
    ACaptureRequest_setEntry_u8(mRequest, ACAMERA_CONTROL_AF_MODE, 1, &afOff);
    ACaptureRequest_setEntry_u8(mRequest, ACAMERA_CONTROL_EFFECT_MODE, 1, &mono);
    ACaptureRequest_setEntry_i64(mRequest, ACAMERA_SENSOR_FRAME_DURATION, 1, &frameDuration);

    rc = ACameraDevice_createCaptureSession(mDevice, mOutputs, &mSessionCallbacks, &mSession);
    if (rc != ACAMERA_OK) {
        *error = "createCaptureSession failed: " + std::to_string(rc);
        mSession = nullptr;
        close();
        return false;
    }
    if (!setExposure(exposureUs, iso)) {
        *error = "setRepeatingRequest failed";
        close();
        return false;
    }
    LOG(INFO) << "iris camera " << mCameraId << " streaming " << kFrameWidth << "x"
              << kFrameHeight << ", exposure " << exposureUs << " us, ISO " << iso;
    return true;
}

bool IrisCamera::setExposure(int exposureUs, int iso) {
    if (!mSession || !mRequest) return false;
    const int64_t exposureNs = int64_t(exposureUs) * 1000;
    const int32_t sensitivity = iso;
    ACaptureRequest_setEntry_i64(mRequest, ACAMERA_SENSOR_EXPOSURE_TIME, 1, &exposureNs);
    ACaptureRequest_setEntry_i32(mRequest, ACAMERA_SENSOR_SENSITIVITY, 1, &sensitivity);
    camera_status_t rc = ACameraCaptureSession_setRepeatingRequest(mSession, &mCaptureCallbacks, 1,
                                                                   &mRequest, nullptr);
    if (rc != ACAMERA_OK) {
        LOG(ERROR) << "setRepeatingRequest: " << rc;
        return false;
    }
    return true;
}

void IrisCamera::close() {
    interrupt();
    if (mSession) {
        ACameraCaptureSession_stopRepeating(mSession);
        ACameraCaptureSession_close(mSession);
        mSession = nullptr;
    }
    if (mDevice) {
        ACameraDevice_close(mDevice);
        mDevice = nullptr;
    }
    if (mRequest) {
        if (mReaderTarget) ACaptureRequest_removeTarget(mRequest, mReaderTarget);
        ACaptureRequest_free(mRequest);
        mRequest = nullptr;
    }
    if (mReaderTarget) ACameraOutputTarget_free(mReaderTarget);
    mReaderTarget = nullptr;
    if (mOutputs) {
        if (mReaderOutput) ACaptureSessionOutputContainer_remove(mOutputs, mReaderOutput);
        ACaptureSessionOutputContainer_free(mOutputs);
        mOutputs = nullptr;
    }
    if (mReaderOutput) ACaptureSessionOutput_free(mReaderOutput);
    mReaderOutput = nullptr;
    if (mReader) {
        AImageReader_delete(mReader);  // also releases mReaderWindow
        mReader = nullptr;
        mReaderWindow = nullptr;
    }
    if (mManager) {
        ACameraManager_delete(mManager);
        mManager = nullptr;
    }
    std::lock_guard<std::mutex> lock(mFrameLock);
    mQueue.clear();
    mMeta.clear();
}

void IrisCamera::interrupt() {
    std::lock_guard<std::mutex> lock(mFrameLock);
    mInterrupted = true;
    mFrameCond.notify_all();
}

void IrisCamera::lookupMeta(Frame* f) {
    // Called with mFrameLock held.
    for (auto it = mMeta.rbegin(); it != mMeta.rend(); ++it) {
        if (it->timestampNs == f->timestampNs) {
            f->exposureNs = it->exposureNs;
            f->skewNs = it->skewNs;
            return;
        }
    }
}

bool IrisCamera::waitFrame(int64_t lastSequence, int timeoutMs, Frame* out) {
    std::unique_lock<std::mutex> lock(mFrameLock);
    auto ready = [&] {
        if (mInterrupted || mError) return true;
        for (const auto& f : mQueue) {
            if (f.sequence > lastSequence) return true;
        }
        return false;
    };
    if (!mFrameCond.wait_for(lock, std::chrono::milliseconds(timeoutMs), ready)) return false;
    if (mInterrupted || mError) return false;
    for (auto& f : mQueue) {
        if (f.sequence > lastSequence) {
            // The result usually arrives before the buffer, but not always: look it up now.
            if (f.exposureNs == 0) lookupMeta(&f);
            *out = f;
            return true;
        }
    }
    return false;
}

void IrisCamera::onImageAvailable(void* ctx, AImageReader* reader) {
    auto* self = static_cast<IrisCamera*>(ctx);
    AImage* image = nullptr;
    if (AImageReader_acquireNextImage(reader, &image) != AMEDIA_OK || !image) return;
    int32_t w = 0, h = 0, stride = 0, len = 0;
    int64_t ts = 0;
    uint8_t* data = nullptr;
    if (AImage_getWidth(image, &w) == AMEDIA_OK && AImage_getHeight(image, &h) == AMEDIA_OK &&
        AImage_getPlaneRowStride(image, 0, &stride) == AMEDIA_OK &&
        AImage_getPlaneData(image, 0, &data, &len) == AMEDIA_OK &&
        AImage_getTimestamp(image, &ts) == AMEDIA_OK && data && w > 0 && h > 0 &&
        stride >= w && int64_t(len) >= int64_t(stride) * (h - 1) + w) {
        Frame f;
        {
            std::lock_guard<std::mutex> lock(self->mFrameLock);
            if (!self->mPool.empty()) {
                f = std::move(self->mPool.back());
                self->mPool.pop_back();
            }
        }
        f.exposureNs = f.skewNs = 0;
        f.width = w;
        f.height = h;
        f.timestampNs = ts;
        f.y.resize(size_t(w) * h);
        uint64_t sum = 0;
        uint32_t n = 0;
        for (int row = 0; row < h; row++) {
            const uint8_t* src = data + size_t(row) * stride;
            memcpy(f.y.data() + size_t(row) * w, src, w);
            if ((row & 7) == 0) {
                for (int x = 0; x < w; x += 8, n++) sum += src[x];
            }
        }
        f.meanY = n ? float(sum) / n : 0.f;
        std::lock_guard<std::mutex> lock(self->mFrameLock);
        self->lookupMeta(&f);
        f.sequence = ++self->mSequence;
        self->mQueue.push_back(std::move(f));
        while (self->mQueue.size() > kQueueFrames) {
            self->mPool.push_back(std::move(self->mQueue.front()));
            self->mQueue.pop_front();
        }
        self->mFrameCond.notify_all();
    }
    AImage_delete(image);
}

void IrisCamera::onCaptureCompleted(void* ctx, ACameraCaptureSession*, ACaptureRequest*,
                                    const ACameraMetadata* result) {
    auto* self = static_cast<IrisCamera*>(ctx);
    Meta m{0, 0, 0};
    ACameraMetadata_const_entry e{};
    if (ACameraMetadata_getConstEntry(result, ACAMERA_SENSOR_TIMESTAMP, &e) == ACAMERA_OK &&
        e.count) {
        m.timestampNs = e.data.i64[0];
    }
    if (ACameraMetadata_getConstEntry(result, ACAMERA_SENSOR_EXPOSURE_TIME, &e) == ACAMERA_OK &&
        e.count) {
        m.exposureNs = e.data.i64[0];
    }
    if (ACameraMetadata_getConstEntry(result, ACAMERA_SENSOR_ROLLING_SHUTTER_SKEW, &e) ==
                ACAMERA_OK &&
        e.count) {
        m.skewNs = e.data.i64[0];
    }
    if (m.timestampNs == 0) return;
    std::lock_guard<std::mutex> lock(self->mFrameLock);
    self->mMeta.push_back(m);
    while (self->mMeta.size() > kMetaEntries) self->mMeta.pop_front();
    // Fill frames that arrived before their result.
    for (auto& f : self->mQueue) {
        if (f.timestampNs == m.timestampNs) {
            f.exposureNs = m.exposureNs;
            f.skewNs = m.skewNs;
        }
    }
}

void IrisCamera::onCaptureStarted(void* ctx, ACameraCaptureSession*, const ACaptureRequest*,
                                  int64_t timestamp) {
    static_cast<IrisCamera*>(ctx)->mLastStartNs = timestamp;
}

void IrisCamera::onDeviceDisconnected(void* ctx, ACameraDevice*) {
    auto* self = static_cast<IrisCamera*>(ctx);
    LOG(WARNING) << "iris camera disconnected (evicted by a higher-priority client?)";
    self->mError = true;
    self->interrupt();
}

void IrisCamera::onDeviceError(void* ctx, ACameraDevice*, int error) {
    auto* self = static_cast<IrisCamera*>(ctx);
    LOG(ERROR) << "iris camera error " << error;
    self->mError = true;
    self->interrupt();
}

void IrisCamera::onSessionClosed(void*, ACameraCaptureSession*) {}
void IrisCamera::onSessionReady(void*, ACameraCaptureSession*) {}
void IrisCamera::onSessionActive(void*, ACameraCaptureSession*) {}

}  // namespace cityman::iris
