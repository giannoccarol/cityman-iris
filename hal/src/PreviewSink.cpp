/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_TAG "cityman.iris.preview"

#include "PreviewSink.h"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include <android-base/logging.h>
#include <android/hardware_buffer.h>

namespace cityman::iris {

namespace {
constexpr int kDefaultSize = 720;  // when the window does not report a size
}  // namespace

struct PreviewSink::State {
    ANativeWindow* window = nullptr;
    std::mutex lock;
    std::condition_variable cond;
    bool quit = false;
    bool pending = false;
    bool dead = false;
    // Slot filled by submit(); swapped with `work` by the thread (no allocation per frame).
    std::vector<uint8_t> slot, work;
    PreviewInput slotIn, workIn;
    PreviewRenderer renderer;
    int frames = 0;

    ~State() {
        if (window) ANativeWindow_release(window);
    }

    // Locks the window and renders (or clears when `in` is null). False when the window died.
    bool draw(const PreviewInput* in) {
        ANativeWindow_Buffer buf{};
        const int32_t rc = ANativeWindow_lock(window, &buf, nullptr);
        if (rc != 0) {
            LOG(WARNING) << "preview surface unusable (" << rc << "), drawing stopped";
            return false;
        }
        if (buf.format != AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM &&
            buf.format != AHARDWAREBUFFER_FORMAT_R8G8B8X8_UNORM) {
            LOG(WARNING) << "preview surface format " << buf.format << ", drawing stopped";
            ANativeWindow_unlockAndPost(window);
            return false;
        }
        if (buf.width != renderer.width() || buf.height != renderer.height()) {
            renderer.configure(buf.width, buf.height);  // only when the size changes
        }
        uint8_t* bits = static_cast<uint8_t*>(buf.bits);
        if (in) {
            renderer.render(*in, bits, buf.stride);
        } else {
            renderer.clear(bits, buf.stride);
        }
        return ANativeWindow_unlockAndPost(window) == 0;
    }

    void loop() {
        int w = ANativeWindow_getWidth(window);
        int h = ANativeWindow_getHeight(window);
        if (w <= 0 || h <= 0) w = h = kDefaultSize;
        ANativeWindow_setBuffersGeometry(window, w, h, AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM);
        renderer.configure(w, h);
        LOG(INFO) << "preview " << w << "x" << h;
        // Background at once, so the enrollment circle is not empty before the first lit frame.
        bool ok = draw(nullptr);
        std::unique_lock<std::mutex> l(lock);
        dead = !ok;
        while (!quit && !dead) {
            cond.wait(l, [this] { return quit || pending; });
            if (quit) break;
            std::swap(slot, work);
            workIn = slotIn;
            workIn.y = work.data();
            pending = false;
            l.unlock();
            const auto t0 = std::chrono::steady_clock::now();
            ok = draw(&workIn);
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - t0)
                                    .count();
            l.lock();
            if (!ok) dead = true;
            if (++frames == 1 || frames % 30 == 0) LOG(DEBUG) << "preview frame " << ms << " ms";
        }
    }
};

PreviewSink::PreviewSink(ANativeWindow* window) : mState(std::make_shared<State>()) {
    ANativeWindow_acquire(window);
    mState->window = window;
    // The thread keeps the state alive: the sink may go away while it is blocked in the window.
    std::thread([s = mState] { s->loop(); }).detach();
}

PreviewSink::~PreviewSink() {
    {
        std::lock_guard<std::mutex> l(mState->lock);
        mState->quit = true;
    }
    mState->cond.notify_all();
}

bool PreviewSink::alive() const {
    std::lock_guard<std::mutex> l(mState->lock);
    return !mState->dead;
}

void PreviewSink::submit(const Frame& lit, const PreviewEye eyes[2], int rotation, bool mirror) {
    std::lock_guard<std::mutex> l(mState->lock);
    if (mState->dead || mState->quit || mState->pending) return;  // busy: drop
    mState->slot.assign(lit.y.begin(), lit.y.end());  // reuses the capacity after the first
    PreviewInput& in = mState->slotIn;
    in.width = lit.width;
    in.height = lit.height;
    in.stride = lit.width;
    in.rotation = rotation;
    in.mirror = mirror;
    in.eyes[0] = eyes[0];
    in.eyes[1] = eyes[1];
    mState->pending = true;
    mState->cond.notify_all();
}

}  // namespace cityman::iris
