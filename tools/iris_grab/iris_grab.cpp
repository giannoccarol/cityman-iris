// SPDX-License-Identifier: Apache-2.0
//
// iris_grab: capture IR frames from the Lumia 950 XL iris camera through the camera2 NDK,
// the same path the iris HAL uses, for tests and for feeding the recognition engine.
//
//   iris_grab [-c ID] [-n FRAMES] [-e EXPOSURE_US] [-i ISO] [-l LED_MA] [-w WARMUP]
//             [-o OUTDIR] [-x]
//
// The capture runs a YUV repeating request (warm-up, AE when -e/-i are 0), then a burst of
// FRAMES still requests on the RAW16 and YUV outputs. Each frame is saved as
//   frame_N.raw16  sensor data, W*H little-endian uint16 (10-bit, black level 64)
//   frame_N.pgm    8-bit view of the RAW16 frame (black removed, 1st-99.5th percentile)
//   frame_N_y.pgm  Y plane of the YUV output
// plus meta.txt with exposure, sensitivity and timestamp of every frame.
// -l lights the IR LED (PMI8994 flash channel 0, led:torch_0) only while the camera is
// streaming; it is switched off on exit, on errors and on SIGINT/SIGTERM.
// -x keeps the ISP colour pipeline (no CONTROL_EFFECT_MODE_MONO).
// Run as root (the LED is a root-only sysfs file; cameraserver treats uid 0 as trusted).
// Times are on CLOCK_MONOTONIC, the clock of the msm8994 camera timestamps (source UNKNOWN).

#include <camera/NdkCameraCaptureSession.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraMetadata.h>
#include <camera/NdkCaptureRequest.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace {

char kLed[64] = "/sys/class/leds/led:torch_0/brightness";
const int kLedMaxMa = 200;
const int kBlack = 64;

std::atomic<bool> gLedOn{false};

void led_write(int ma) {
    FILE *f = std::fopen(kLed, "w");
    if (!f) return;
    std::fprintf(f, "%d\n", ma);
    std::fclose(f);
}

void led_off() {
    if (gLedOn.exchange(false)) led_write(0);
}

void on_signal(int sig) {
    // write() is async-signal-safe; fopen is not.
    int fd = open(kLed, O_WRONLY);
    if (fd >= 0) {
        (void)!write(fd, "0\n", 2);
        close(fd);
    }
    _exit(128 + sig);
}

struct Options {
    std::string id = "2";
    int frames = 5;
    int64_t exposure_us = 0;
    int iso = 0;
    int led_ma = 0;
    int warmup = 15;
    std::string out = "/data/local/tmp/iris_grab";
    bool colour = false;
    int led_channel = 0;
    // Pulse mode (-p ON,OFF,TOTAL ms): LED pulse train during the preview stream, no burst;
    // per-frame row-band brightness and pulse times are written to frames.csv / pulses.csv.
    int pulse_on_ms = 0, pulse_off_ms = 0, pulse_total_ms = 0;
    // Frame-locked mode (-S K,TOTAL ms): light one whole frame every K frames, timing the pulse
    // from the previous frame's timestamp, frame duration and rolling shutter skew.
    int sync_k = 0;
};

struct FrameStat {
    int64_t ts;
    double band[16];
};

struct PulseTimes {
    int64_t on_before, on_after, off_before, off_after;  // CLOCK_MONOTONIC ns around each write
};

int64_t boottime_ns() {
    timespec t{};
    clock_gettime(CLOCK_MONOTONIC, &t);
    return int64_t(t.tv_sec) * 1000000000LL + t.tv_nsec;
}

struct FrameMeta {
    int64_t exposure_ns = 0;
    int32_t iso = 0;
    int64_t timestamp = 0;
};

struct State {
    std::mutex m;
    std::condition_variable cv;
    Options opt;
    int raw_w = 0, raw_h = 0, yuv_w = 0, yuv_h = 0;
    bool capturing = false;
    int raw_saved = 0, yuv_saved = 0, yuv_seen = 0, raw_seen = 0;
    int failed = 0;
    std::map<int64_t, FrameMeta> meta;  // keyed by sensor timestamp
    std::vector<int64_t> raw_ts, yuv_ts;
    std::vector<FrameStat> frame_stats;
    int64_t last_ts = 0, max_cb_latency = 0, sum_cb_latency = 0;
    int n_cb = 0;
    std::vector<PulseTimes> pulses;
    int64_t frame_duration_ns = 0, skew_ns = 0;
};

State gS;
int64_t gT0 = 0;
std::atomic<bool> gFirstFrame{false};
void stamp(const char *what) {
    timespec t{};
    clock_gettime(CLOCK_MONOTONIC, &t);
    int64_t now = int64_t(t.tv_sec) * 1000000000LL + t.tv_nsec;
    if (getenv("IRIS_GRAB_TIMING")) std::fprintf(stderr, "t+%4lld ms %s\n", (long long)((now - gT0) / 1000000), what);
}

bool write_file(const std::string &path, const void *data, size_t len) {
    FILE *f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    bool ok = std::fwrite(data, 1, len, f) == len;
    return std::fclose(f) == 0 && ok;
}

bool write_pgm(const std::string &path, int w, int h, const std::vector<uint8_t> &px) {
    std::string hdr = "P5\n" + std::to_string(w) + " " + std::to_string(h) + "\n255\n";
    std::vector<uint8_t> buf(hdr.begin(), hdr.end());
    buf.insert(buf.end(), px.begin(), px.end());
    return write_file(path, buf.data(), buf.size());
}

void save_raw(AImage *img, int idx) {
    uint8_t *data = nullptr;
    int len = 0, stride = 0;
    int32_t w = 0, h = 0;
    AImage_getWidth(img, &w);
    AImage_getHeight(img, &h);
    if (AImage_getPlaneData(img, 0, &data, &len) != AMEDIA_OK ||
        AImage_getPlaneRowStride(img, 0, &stride) != AMEDIA_OK) {
        std::fprintf(stderr, "raw %d: no plane data\n", idx);
        return;
    }
    std::vector<uint16_t> px(size_t(w) * h);
    for (int y = 0; y < h; y++) std::memcpy(&px[size_t(y) * w], data + size_t(y) * stride, size_t(w) * 2);
    char name[64];
    std::snprintf(name, sizeof name, "/frame_%d.raw16", idx);
    write_file(gS.opt.out + name, px.data(), px.size() * 2);

    // 8-bit view: black removed, stretched between the 1st and 99.5th percentile.
    std::vector<uint16_t> sorted(px.begin(), px.end());
    size_t lo_i = sorted.size() / 100, hi_i = sorted.size() * 995 / 1000;
    std::nth_element(sorted.begin(), sorted.begin() + lo_i, sorted.end());
    int lo = sorted[lo_i];
    std::nth_element(sorted.begin(), sorted.begin() + hi_i, sorted.end());
    int hi = std::max<int>(sorted[hi_i], lo + 1);
    std::vector<uint8_t> v(px.size());
    double sum = 0;
    for (size_t i = 0; i < px.size(); i++) {
        sum += px[i];
        int t = (int(px[i]) - lo) * 255 / (hi - lo);
        v[i] = uint8_t(std::clamp(t, 0, 255));
    }
    std::snprintf(name, sizeof name, "/frame_%d.pgm", idx);
    write_pgm(gS.opt.out + name, w, h, v);
    std::printf("raw %d: %dx%d stride %d mean %.1f (above black %.1f) p1 %d p99.5 %d\n", idx, w, h, stride,
                sum / px.size(), sum / px.size() - kBlack, lo, hi);
}

void save_y(AImage *img, int idx) {
    uint8_t *data = nullptr;
    int len = 0, stride = 0;
    int32_t w = 0, h = 0;
    AImage_getWidth(img, &w);
    AImage_getHeight(img, &h);
    if (AImage_getPlaneData(img, 0, &data, &len) != AMEDIA_OK ||
        AImage_getPlaneRowStride(img, 0, &stride) != AMEDIA_OK) {
        std::fprintf(stderr, "yuv %d: no plane data\n", idx);
        return;
    }
    std::vector<uint8_t> v(size_t(w) * h);
    double sum = 0;
    for (int y = 0; y < h; y++) {
        std::memcpy(&v[size_t(y) * w], data + size_t(y) * stride, w);
        for (int x = 0; x < w; x++) sum += v[size_t(y) * w + x];
    }
    // Chroma check: with MONO the U/V planes should sit at 128.
    double usum = 0;
    int n = 0;
    uint8_t *u = nullptr;
    int ulen = 0, ustride = 0, upix = 0;
    if (AImage_getPlaneData(img, 1, &u, &ulen) == AMEDIA_OK && AImage_getPlaneRowStride(img, 1, &ustride) == AMEDIA_OK &&
        AImage_getPlanePixelStride(img, 1, &upix) == AMEDIA_OK) {
        for (int y = 0; y < h / 2; y += 8)
            for (int x = 0; x < w / 2; x += 8) {
                usum += u[size_t(y) * ustride + size_t(x) * upix];
                n++;
            }
    }
    char name[64];
    std::snprintf(name, sizeof name, "/frame_%d_y.pgm", idx);
    write_pgm(gS.opt.out + name, w, h, v);
    std::printf("yuv %d: %dx%d mean Y %.1f mean U %.1f\n", idx, w, h, sum / v.size(), n ? usum / n : -1.0);
}

void on_raw(void *, AImageReader *r) {
    AImage *img = nullptr;
    if (AImageReader_acquireNextImage(r, &img) != AMEDIA_OK || !img) return;
    {
        std::lock_guard<std::mutex> l(gS.m);
        gS.raw_seen++;
    }
    int64_t ts = 0;
    AImage_getTimestamp(img, &ts);
    int idx = -1;
    {
        std::lock_guard<std::mutex> l(gS.m);
        if (gS.capturing && gS.raw_saved < gS.opt.frames) {
            idx = gS.raw_saved;
            gS.raw_ts.push_back(ts);
        }
    }
    if (idx >= 0) save_raw(img, idx);
    AImage_delete(img);
    if (idx >= 0) {
        std::lock_guard<std::mutex> l(gS.m);
        gS.raw_saved++;
        gS.cv.notify_all();
    }
}

void frame_stat(AImage *img, int64_t ts) {
    uint8_t *data = nullptr;
    int len = 0, stride = 0;
    int32_t w = 0, h = 0;
    AImage_getWidth(img, &w);
    AImage_getHeight(img, &h);
    if (AImage_getPlaneData(img, 0, &data, &len) != AMEDIA_OK ||
        AImage_getPlaneRowStride(img, 0, &stride) != AMEDIA_OK)
        return;
    FrameStat fs{};
    fs.ts = ts;
    int64_t lat = boottime_ns() - ts;
    for (int b = 0; b < 16; b++) {
        int y0 = h * b / 16, y1 = h * (b + 1) / 16;
        double sum = 0;
        long n = 0;
        for (int y = y0; y < y1; y += 2)
            for (int x = 0; x < w; x += 8) {
                sum += data[size_t(y) * stride + x];
                n++;
            }
        fs.band[b] = n ? sum / n : 0;
    }
    std::lock_guard<std::mutex> l(gS.m);
    gS.frame_stats.push_back(fs);
    gS.last_ts = ts;
    gS.max_cb_latency = std::max(gS.max_cb_latency, lat);
    gS.sum_cb_latency += lat;
    gS.n_cb++;
    gS.cv.notify_all();
}

void sleep_until_boottime(int64_t t) {
    timespec ts{};
    ts.tv_sec = t / 1000000000LL;
    ts.tv_nsec = t % 1000000000LL;
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
    }
}

void on_yuv(void *, AImageReader *r) {
    AImage *img = nullptr;
    if (AImageReader_acquireNextImage(r, &img) != AMEDIA_OK || !img) return;
    if (!gFirstFrame.exchange(true)) stamp("first frame");
    int64_t ts = 0;
    AImage_getTimestamp(img, &ts);
    if (gS.opt.pulse_total_ms > 0) {
        frame_stat(img, ts);
        AImage_delete(img);
        std::lock_guard<std::mutex> l(gS.m);
        gS.yuv_seen++;
        gS.cv.notify_all();
        return;
    }
    int idx = -1;
    {
        std::lock_guard<std::mutex> l(gS.m);
        gS.yuv_seen++;
        if (gS.capturing && gS.yuv_saved < gS.opt.frames) {
            idx = gS.yuv_saved;
            gS.yuv_ts.push_back(ts);
        }
    }
    if (idx >= 0) save_y(img, idx);
    AImage_delete(img);
    {
        std::lock_guard<std::mutex> l(gS.m);
        if (idx >= 0) gS.yuv_saved++;
        gS.cv.notify_all();
    }
}

void cap_completed(void *, ACameraCaptureSession *, ACaptureRequest *, const ACameraMetadata *res) {
    FrameMeta fm;
    ACameraMetadata_const_entry e{};
    if (ACameraMetadata_getConstEntry(res, ACAMERA_SENSOR_EXPOSURE_TIME, &e) == ACAMERA_OK && e.count) fm.exposure_ns = e.data.i64[0];
    if (ACameraMetadata_getConstEntry(res, ACAMERA_SENSOR_SENSITIVITY, &e) == ACAMERA_OK && e.count) fm.iso = e.data.i32[0];
    if (ACameraMetadata_getConstEntry(res, ACAMERA_SENSOR_TIMESTAMP, &e) == ACAMERA_OK && e.count) fm.timestamp = e.data.i64[0];
    std::lock_guard<std::mutex> l(gS.m);
    if (ACameraMetadata_getConstEntry(res, ACAMERA_SENSOR_FRAME_DURATION, &e) == ACAMERA_OK && e.count)
        gS.frame_duration_ns = e.data.i64[0];
    if (ACameraMetadata_getConstEntry(res, ACAMERA_SENSOR_ROLLING_SHUTTER_SKEW, &e) == ACAMERA_OK && e.count)
        gS.skew_ns = e.data.i64[0];
    gS.meta[fm.timestamp] = fm;
}

void cap_failed(void *, ACameraCaptureSession *, ACaptureRequest *, ACameraCaptureFailure *f) {
    std::lock_guard<std::mutex> l(gS.m);
    gS.failed++;
    std::fprintf(stderr, "capture failed: frame %lld reason %d\n", (long long)f->frameNumber, f->reason);
    gS.cv.notify_all();
}

void dev_disconnected(void *, ACameraDevice *) { std::fprintf(stderr, "camera disconnected\n"); }
void dev_error(void *, ACameraDevice *, int err) { std::fprintf(stderr, "camera error %d\n", err); }
void sess_noop(void *, ACameraCaptureSession *) {}

void largest(const ACameraMetadata *chars, int32_t fmt, int &w, int &h) {
    ACameraMetadata_const_entry e{};
    w = h = 0;
    if (ACameraMetadata_getConstEntry(chars, ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &e) != ACAMERA_OK) return;
    for (uint32_t i = 0; i + 3 < e.count; i += 4) {
        if (e.data.i32[i] != fmt || e.data.i32[i + 3] != 0) continue;  // outputs only
        if (int64_t(e.data.i32[i + 1]) * e.data.i32[i + 2] > int64_t(w) * h) {
            w = e.data.i32[i + 1];
            h = e.data.i32[i + 2];
        }
    }
}

// The camera service and the image readers call back over binder: without a thread pool the
// buffers never come back. libbinder_ndk is not in the NDK stubs, so load it at run time.
bool start_binder_pool() {
    void *h = dlopen("libbinder_ndk.so", RTLD_NOW);
    if (!h) return false;
    auto set_max = reinterpret_cast<bool (*)(uint32_t)>(dlsym(h, "ABinderProcess_setThreadPoolMaxThreadCount"));
    auto start = reinterpret_cast<void (*)()>(dlsym(h, "ABinderProcess_startThreadPool"));
    if (!start) return false;
    if (set_max) set_max(4);
    start();
    return true;
}

int usage() {
    std::fprintf(stderr,
                 "usage: iris_grab [-c ID] [-n FRAMES] [-e EXPOSURE_US] [-i ISO] [-l LED_MA (<=%d)] [-L 0|1] [-w WARMUP]\n"
                 "                 [-p ON_MS,OFF_MS,TOTAL_MS (pulse mode)] [-S K,TOTAL_MS (frame-locked pulses)] [-o OUTDIR] [-x (keep colour pipeline)]\n",
                 kLedMaxMa);
    return 2;
}

}  // namespace

int main(int argc, char **argv) {
    Options &o = gS.opt;
    int c;
    while ((c = getopt(argc, argv, "c:n:e:i:l:L:p:S:w:o:xh")) != -1) {
        switch (c) {
            case 'c': o.id = optarg; break;
            case 'n': o.frames = std::atoi(optarg); break;
            case 'e': o.exposure_us = std::atoll(optarg); break;
            case 'i': o.iso = std::atoi(optarg); break;
            case 'l': o.led_ma = std::atoi(optarg); break;
            case 'w': o.warmup = std::atoi(optarg); break;
            case 'o': o.out = optarg; break;
            case 'x': o.colour = true; break;
            case 'L': o.led_channel = std::atoi(optarg); break;
            case 'S':
                if (std::sscanf(optarg, "%d,%d", &o.sync_k, &o.pulse_total_ms) != 2 || o.sync_k < 2 || o.sync_k > 30)
                    return usage();
                o.pulse_on_ms = 1;
                o.pulse_off_ms = 99;
                break;
            case 'p':
                if (std::sscanf(optarg, "%d,%d,%d", &o.pulse_on_ms, &o.pulse_off_ms, &o.pulse_total_ms) != 3) return usage();
                break;
            default: return usage();
        }
    }
    if (o.frames < 1 || o.frames > 30 || o.led_ma < 0 || o.led_ma > kLedMaxMa || o.warmup < 0 || o.warmup > 120)
        return usage();
    if (o.led_channel != 0 && o.led_channel != 1) return usage();
    // Pulse limits of the Windows driver envelope: on <= 70 ms, duty <= 35 %, train <= 4 s.
    if (o.pulse_total_ms > 0 && (o.pulse_on_ms < 1 || o.pulse_on_ms > 70 || o.pulse_off_ms < 1 ||
                                 o.pulse_on_ms * 100 > 35 * (o.pulse_on_ms + o.pulse_off_ms) ||
                                 o.pulse_total_ms > 4000 || o.led_ma < 1))
        return usage();
    std::snprintf(kLed, sizeof kLed, "/sys/class/leds/led:torch_%d/brightness", o.led_channel);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGHUP, on_signal);
    std::atexit(led_off);
    std::string mk = "mkdir -p '" + o.out + "'";
    if (std::system(mk.c_str()) != 0) return std::fprintf(stderr, "cannot create %s\n", o.out.c_str()), 1;

    {
        timespec t{};
        clock_gettime(CLOCK_MONOTONIC, &t);
        gT0 = int64_t(t.tv_sec) * 1000000000LL + t.tv_nsec;
    }
    if (!start_binder_pool()) std::fprintf(stderr, "warning: no binder thread pool\n");
    ACameraManager *mgr = ACameraManager_create();
    ACameraMetadata *chars = nullptr;
    if (ACameraManager_getCameraCharacteristics(mgr, o.id.c_str(), &chars) != ACAMERA_OK)
        return std::fprintf(stderr, "camera %s: no characteristics\n", o.id.c_str()), 1;
    largest(chars, AIMAGE_FORMAT_RAW16, gS.raw_w, gS.raw_h);
    largest(chars, AIMAGE_FORMAT_YUV_420_888, gS.yuv_w, gS.yuv_h);
    std::printf("camera %s: RAW16 %dx%d, YUV %dx%d\n", o.id.c_str(), gS.raw_w, gS.raw_h, gS.yuv_w, gS.yuv_h);
    if (!gS.yuv_w) return std::fprintf(stderr, "no YUV output\n"), 1;

    ACameraDevice_StateCallbacks dcb{nullptr, dev_disconnected, dev_error};
    ACameraDevice *dev = nullptr;
    stamp("open");
    camera_status_t st = ACameraManager_openCamera(mgr, o.id.c_str(), &dcb, &dev);
    stamp("opened");
    if (st != ACAMERA_OK) return std::fprintf(stderr, "openCamera: %d\n", st), 1;

    AImageReader *raw_r = nullptr, *yuv_r = nullptr;
    ANativeWindow *raw_w = nullptr, *yuv_w = nullptr;
    AImageReader_ImageListener raw_l{nullptr, on_raw}, yuv_l{nullptr, on_yuv};
    if (gS.raw_w) {
        AImageReader_new(gS.raw_w, gS.raw_h, AIMAGE_FORMAT_RAW16, 4, &raw_r);
        AImageReader_setImageListener(raw_r, &raw_l);
        AImageReader_getWindow(raw_r, &raw_w);
    }
    AImageReader_new(gS.yuv_w, gS.yuv_h, AIMAGE_FORMAT_YUV_420_888, 4, &yuv_r);
    AImageReader_setImageListener(yuv_r, &yuv_l);
    AImageReader_getWindow(yuv_r, &yuv_w);

    ACaptureSessionOutputContainer *outs = nullptr;
    ACaptureSessionOutputContainer_create(&outs);
    ACaptureSessionOutput *raw_o = nullptr, *yuv_o = nullptr;
    if (raw_w) {
        ACaptureSessionOutput_create(raw_w, &raw_o);
        ACaptureSessionOutputContainer_add(outs, raw_o);
    }
    ACaptureSessionOutput_create(yuv_w, &yuv_o);
    ACaptureSessionOutputContainer_add(outs, yuv_o);
    ACameraCaptureSession_stateCallbacks scb{nullptr, sess_noop, sess_noop, sess_noop};
    ACameraCaptureSession *sess = nullptr;
    st = ACameraDevice_createCaptureSession(dev, outs, &scb, &sess);
    stamp("session");
    if (st != ACAMERA_OK) return std::fprintf(stderr, "createCaptureSession: %d\n", st), 1;

    ACameraOutputTarget *raw_t = nullptr, *yuv_t = nullptr;
    if (raw_w) ACameraOutputTarget_create(raw_w, &raw_t);
    ACameraOutputTarget_create(yuv_w, &yuv_t);

    auto setup = [&](ACaptureRequest *rq) {
        uint8_t effect = o.colour ? ACAMERA_CONTROL_EFFECT_MODE_OFF : ACAMERA_CONTROL_EFFECT_MODE_MONO;
        ACaptureRequest_setEntry_u8(rq, ACAMERA_CONTROL_EFFECT_MODE, 1, &effect);
        if (o.exposure_us > 0 || o.iso > 0) {
            uint8_t ae = ACAMERA_CONTROL_AE_MODE_OFF;
            ACaptureRequest_setEntry_u8(rq, ACAMERA_CONTROL_AE_MODE, 1, &ae);
            int64_t ns = (o.exposure_us > 0 ? o.exposure_us : 20000) * 1000;
            int32_t iso = o.iso > 0 ? o.iso : 400;
            ACaptureRequest_setEntry_i64(rq, ACAMERA_SENSOR_EXPOSURE_TIME, 1, &ns);
            ACaptureRequest_setEntry_i32(rq, ACAMERA_SENSOR_SENSITIVITY, 1, &iso);
            // Keep the frame duration at least as long as the exposure.
            int64_t fd = std::max<int64_t>(ns, 33333333);
            ACaptureRequest_setEntry_i64(rq, ACAMERA_SENSOR_FRAME_DURATION, 1, &fd);
        }
    };

    ACaptureRequest *prev = nullptr, *still = nullptr;
    ACameraDevice_createCaptureRequest(dev, TEMPLATE_PREVIEW, &prev);
    ACaptureRequest_addTarget(prev, yuv_t);
    // IRIS_GRAB_RAW_REPEATING=1: also stream RAW16 in the repeating request (iris HAL test).
    if (getenv("IRIS_GRAB_RAW_REPEATING") && raw_t) ACaptureRequest_addTarget(prev, raw_t);
    setup(prev);
    ACameraDevice_createCaptureRequest(dev, TEMPLATE_STILL_CAPTURE, &still);
    if (raw_t) ACaptureRequest_addTarget(still, raw_t);
    ACaptureRequest_addTarget(still, yuv_t);
    setup(still);

    if (o.led_ma > 0 && o.pulse_total_ms == 0) {
        gLedOn = true;
        led_write(o.led_ma);
    }
    ACameraCaptureSession_captureCallbacks ccb{};
    ccb.onCaptureCompleted = cap_completed;
    ccb.onCaptureFailed = cap_failed;
    int seq = 0;
    st = ACameraCaptureSession_setRepeatingRequest(sess, &ccb, 1, &prev, &seq);
    if (st != ACAMERA_OK) return std::fprintf(stderr, "setRepeatingRequest: %d\n", st), 1;

    if (o.pulse_total_ms > 0) {
        {
            std::unique_lock<std::mutex> l(gS.m);
            gS.cv.wait_for(l, std::chrono::seconds(4), [] { return gS.yuv_seen >= gS.opt.warmup; });
        }
        int64_t start = boottime_ns(), end = start + int64_t(o.pulse_total_ms) * 1000000;
        gLedOn = true;
        const int64_t margin = 2000000, exp_ns = (o.exposure_us > 0 ? o.exposure_us : 20000) * 1000;
        int64_t next_target = 0;
        while (o.sync_k > 0 && boottime_ns() < end) {
            int64_t last, fd, skew;
            {
                std::unique_lock<std::mutex> l(gS.m);
                gS.cv.wait_for(l, std::chrono::milliseconds(200));
                last = gS.last_ts;
                fd = gS.frame_duration_ns;
                skew = gS.skew_ns;
            }
            if (!last || !fd) continue;
            int64_t now = boottime_ns();
            // First frame start that is far enough in the future and K frames after the last pulse.
            int64_t t = last + fd;
            while (t - margin < now + 1000000 || (next_target && t < next_target - fd / 2)) t += fd;
            int64_t off_at = t + skew + exp_ns + margin;
            if (off_at - (t - margin) > 70000000LL) {
                std::fprintf(stderr, "pulse would exceed 70 ms\n");
                break;
            }
            sleep_until_boottime(t - margin);
            PulseTimes p{};
            p.on_before = boottime_ns();
            led_write(o.led_ma);
            p.on_after = boottime_ns();
            sleep_until_boottime(off_at);
            p.off_before = boottime_ns();
            led_write(0);
            p.off_after = boottime_ns();
            gS.pulses.push_back(p);
            next_target = t + int64_t(o.sync_k) * fd;
        }
        while (o.sync_k == 0 && boottime_ns() < end) {
            PulseTimes p{};
            p.on_before = boottime_ns();
            led_write(o.led_ma);
            p.on_after = boottime_ns();
            usleep(o.pulse_on_ms * 1000);
            p.off_before = boottime_ns();
            led_write(0);
            p.off_after = boottime_ns();
            gS.pulses.push_back(p);
            usleep(o.pulse_off_ms * 1000);
        }
        led_off();
        usleep(300000);
        ACameraCaptureSession_stopRepeating(sess);
        usleep(200000);
        std::lock_guard<std::mutex> l(gS.m);
        std::string fp = o.out + "/frames.csv", pp = o.out + "/pulses.csv";
        if (FILE *f = std::fopen(fp.c_str(), "w")) {
            std::fprintf(f, "# frame_duration_ns %lld rolling_skew_ns %lld exposure_us %lld\nts", (long long)gS.frame_duration_ns,
                         (long long)gS.skew_ns, (long long)o.exposure_us);
            for (int b = 0; b < 16; b++) std::fprintf(f, ",b%d", b);
            std::fprintf(f, "\n");
            for (auto &fs : gS.frame_stats) {
                std::fprintf(f, "%lld", (long long)fs.ts);
                for (int b = 0; b < 16; b++) std::fprintf(f, ",%.2f", fs.band[b]);
                std::fprintf(f, "\n");
            }
            std::fclose(f);
        }
        if (FILE *f = std::fopen(pp.c_str(), "w")) {
            std::fprintf(f, "on_before,on_after,off_before,off_after\n");
            for (auto &p : gS.pulses)
                std::fprintf(f, "%lld,%lld,%lld,%lld\n", (long long)p.on_before, (long long)p.on_after,
                             (long long)p.off_before, (long long)p.off_after);
            std::fclose(f);
        }
        std::printf("callback latency mean %.1f ms max %.1f ms\n", gS.n_cb ? gS.sum_cb_latency / 1e6 / gS.n_cb : 0.0,
                    gS.max_cb_latency / 1e6);
        std::printf("raw frames in the repeating stream: %d\n", gS.raw_seen);
        std::printf("pulse mode: %zu frames, %zu pulses (%d mA, ch %d), frame %lld ns, skew %lld ns -> %s\n",
                    gS.frame_stats.size(), gS.pulses.size(), o.led_ma, o.led_channel, (long long)gS.frame_duration_ns,
                    (long long)gS.skew_ns, o.out.c_str());
        ACameraCaptureSession_close(sess);
        ACameraDevice_close(dev);
        return 0;
    }

    auto t0 = std::chrono::steady_clock::now();
    {
        std::unique_lock<std::mutex> l(gS.m);
        if (!gS.cv.wait_for(l, std::chrono::seconds(4), [] { return gS.yuv_seen >= gS.opt.warmup; }))
            std::fprintf(stderr, "warm-up: only %d preview frames in 4 s\n", gS.yuv_seen);
    }
    // The preview frames still in flight would be saved as burst frames: stop the
    // repeating request first, then send the burst.
    ACameraCaptureSession_stopRepeating(sess);
    usleep(150000);
    {
        std::lock_guard<std::mutex> l(gS.m);
        gS.capturing = true;
    }
    std::vector<ACaptureRequest *> burst(o.frames, still);
    st = ACameraCaptureSession_capture(sess, &ccb, o.frames, burst.data(), &seq);
    if (st != ACAMERA_OK) std::fprintf(stderr, "capture: %d\n", st);
    bool done;
    {
        std::unique_lock<std::mutex> l(gS.m);
        done = gS.cv.wait_for(l, std::chrono::seconds(5), [&] {
            return gS.yuv_saved >= o.frames && (!raw_r || gS.raw_saved >= o.frames || gS.failed >= o.frames);
        });
    }
    led_off();
    double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    ACameraCaptureSession_abortCaptures(sess);
    usleep(200000);

    std::string mpath = o.out + "/meta.txt";
    if (FILE *m = std::fopen(mpath.c_str(), "w")) {
        std::lock_guard<std::mutex> l(gS.m);
        std::fprintf(m, "camera %s raw %dx%d yuv %dx%d led_ma %d effect %s requested_exposure_us %lld iso %d\n",
                     o.id.c_str(), gS.raw_w, gS.raw_h, gS.yuv_w, gS.yuv_h, o.led_ma, o.colour ? "off" : "mono",
                     (long long)o.exposure_us, o.iso);
        for (size_t i = 0; i < gS.raw_ts.size(); i++) {
            auto it = gS.meta.find(gS.raw_ts[i]);
            std::fprintf(m, "raw %zu ts %lld exposure_ns %lld iso %d\n", i, (long long)gS.raw_ts[i],
                         it != gS.meta.end() ? (long long)it->second.exposure_ns : -1LL,
                         it != gS.meta.end() ? it->second.iso : -1);
        }
        for (size_t i = 0; i < gS.yuv_ts.size(); i++) {
            auto it = gS.meta.find(gS.yuv_ts[i]);
            std::fprintf(m, "yuv %zu ts %lld exposure_ns %lld iso %d\n", i, (long long)gS.yuv_ts[i],
                         it != gS.meta.end() ? (long long)it->second.exposure_ns : -1LL,
                         it != gS.meta.end() ? it->second.iso : -1);
        }
        std::fclose(m);
    }
    std::printf("%s: %d raw, %d yuv, %d failed, LED %d mA for %.1f s, files in %s\n", done ? "done" : "timeout",
                gS.raw_saved, gS.yuv_saved, gS.failed, o.led_ma, secs, o.out.c_str());

    ACameraCaptureSession_close(sess);
    ACameraDevice_close(dev);
    if (raw_r) AImageReader_delete(raw_r);
    AImageReader_delete(yuv_r);
    ACameraMetadata_free(chars);
    ACameraManager_delete(mgr);
    return done ? 0 : 1;
}
