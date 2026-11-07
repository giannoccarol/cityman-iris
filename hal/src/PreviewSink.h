/*
 * SPDX-FileCopyrightText: 2026 giannoccarol
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <memory>

#include <android/native_window.h>

#include "IrisCamera.h"
#include "Preview.h"

namespace cityman::iris {

// Draws the enrollment preview into the Surface that Settings hands over
// (FaceEnrollOptions.surfacePreview). Rendering runs on its own thread with a one-frame slot:
// a frame submitted while the slot is full is dropped. If the Surface dies (Settings paused or
// gone) drawing stops and enrollment goes on. The thread owns the state, so the sink can be
// destroyed even while the thread is blocked in the window (it then exits on its own).
class PreviewSink {
  public:
    // Takes its own reference on `window`.
    explicit PreviewSink(ANativeWindow* window);
    ~PreviewSink();
    PreviewSink(const PreviewSink&) = delete;
    PreviewSink& operator=(const PreviewSink&) = delete;

    void submit(const Frame& lit, const PreviewEye eyes[2], int rotation, bool mirror);
    bool alive() const;

  private:
    struct State;
    std::shared_ptr<State> mState;
};

}  // namespace cityman::iris
