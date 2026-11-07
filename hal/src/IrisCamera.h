/*
 * SPDX-FileCopyrightText: 2026 giannoccarol
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <android/native_window.h>
#include <camera/NdkCameraCaptureSession.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCaptureRequest.h>
#include <media/NdkImageReader.h>

namespace cityman::iris {

struct Frame {
    std::vector<uint8_t> y;  // luma plane, tightly packed width x height
    int width = 0;
    int height = 0;
    int64_t timestampNs = 0;  // SENSOR_TIMESTAMP: start of exposure of the first row, BOOTTIME
    int64_t exposureNs = 0;   // from the capture result (0 = unknown)
    int64_t skewNs = 0;       // rolling shutter skew from the capture result (0 = unknown)
    float meanY = 0.f;        // subsampled mean luma
    int64_t sequence = 0;
};

// Iris camera client over the vendor camera2 NDK (libcamera2ndk_vendor), which talks to
// cameraserver through the stable AIDL android.frameworks.cameraservice.service. The cityman
// camera HAL advertises the iris camera with the SYSTEM_CAMERA capability, so cameraserver hides
// it from apps and shows it to vendor clients such as this HAL.
//
// One YUV_420_888 analysis stream (kFrameWidth x kFrameHeight), manual exposure. Frames are kept
// in a short queue so that the frame lit by a pulse is not overwritten while the engine runs.
class IrisCamera {
  public:
    IrisCamera();
    ~IrisCamera();
    IrisCamera(const IrisCamera&) = delete;
    IrisCamera& operator=(const IrisCamera&) = delete;

    bool open(int exposureUs, int iso, std::string* error);
    void close();
    bool isOpen() const { return mSession != nullptr; }
    // Changes the manual exposure of the repeating request (applied a few frames later; the
    // frame metadata tells when).
    bool setExposure(int exposureUs, int iso);

    // Next queued frame newer than `lastSequence` (frames dropped from the queue are skipped).
    // Returns false on timeout, interrupt() or a camera error.
    bool waitFrame(int64_t lastSequence, int timeoutMs, Frame* out);
    void interrupt();
    bool failed() const { return mError.load(); }
    const std::string& cameraId() const { return mCameraId; }
    // ACAMERA_SENSOR_ORIENTATION of the iris camera (clockwise degrees), 90 if unknown.
    int sensorOrientation() const { return mSensorOrientation; }
    int64_t framesReceived() const { return mSequence.load(); }
    // SENSOR_TIMESTAMP of the last capture started (shutter notify): known about one frame
    // before the image itself arrives. 0 before the first one.
    int64_t lastStartTimestamp() const { return mLastStartNs.load(); }

  private:
    struct Meta {
        int64_t timestampNs;
        int64_t exposureNs;
        int64_t skewNs;
    };

    std::string findCameraId(std::string* error);
    void lookupMeta(Frame* f);
    static void onImageAvailable(void* ctx, AImageReader* reader);
    static void onCaptureCompleted(void* ctx, ACameraCaptureSession* session,
                                   ACaptureRequest* request, const ACameraMetadata* result);
    static void onCaptureStarted(void* ctx, ACameraCaptureSession* session,
                                 const ACaptureRequest* request, int64_t timestamp);
    static void onDeviceDisconnected(void* ctx, ACameraDevice* device);
    static void onDeviceError(void* ctx, ACameraDevice* device, int error);
    static void onSessionClosed(void* ctx, ACameraCaptureSession* session);
    static void onSessionReady(void* ctx, ACameraCaptureSession* session);
    static void onSessionActive(void* ctx, ACameraCaptureSession* session);

    ACameraManager* mManager = nullptr;
    ACameraDevice* mDevice = nullptr;
    AImageReader* mReader = nullptr;
    ANativeWindow* mReaderWindow = nullptr;  // owned by mReader
    ACaptureSessionOutputContainer* mOutputs = nullptr;
    ACaptureSessionOutput* mReaderOutput = nullptr;
    ACameraOutputTarget* mReaderTarget = nullptr;
    ACaptureRequest* mRequest = nullptr;
    ACameraCaptureSession* mSession = nullptr;

    ACameraDevice_StateCallbacks mDeviceCallbacks{};
    ACameraCaptureSession_stateCallbacks mSessionCallbacks{};
    ACameraCaptureSession_captureCallbacks mCaptureCallbacks{};
    AImageReader_ImageListener mListener{};

    std::string mCameraId;
    int mSensorOrientation = 90;

    std::mutex mFrameLock;
    std::condition_variable mFrameCond;
    std::deque<Frame> mQueue;
    std::vector<Frame> mPool;  // recycled frames: no 2 MB allocation per image
    std::deque<Meta> mMeta;
    bool mInterrupted = false;
    std::atomic<bool> mError{false};
    std::atomic<int64_t> mSequence{0};
    std::atomic<int64_t> mLastStartNs{0};
};

}  // namespace cityman::iris
