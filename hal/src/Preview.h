/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Enrollment preview renderer: turns a lit IR frame into the image shown in the Settings face
 * enrollment circle. Pure C++ (no Android dependency): tested on the host (tests/preview_test.cpp).
 */
#pragma once

#include <cstdint>
#include <vector>

namespace cityman::iris {

struct PreviewEye {
    bool found = false;  // the engine found a pupil/iris in this half
    bool good = false;   // the sample was accepted (status OK)
    float irisX = 0, irisY = 0, irisR = 0;  // sensor image coordinates
    float pupilX = 0, pupilY = 0, pupilR = 0;
    float progress = 0;  // collected samples / target, 0..1
};

struct PreviewInput {
    const uint8_t* y = nullptr;  // luma, sensor orientation
    int width = 0;
    int height = 0;
    int stride = 0;
    int rotation = 90;  // clockwise rotation that makes the sensor image upright
    bool mirror = true;  // mirror like a front camera
    PreviewEye eyes[2];
};

// Not thread safe. configure() allocates; render() does not (once the source size is known).
class PreviewRenderer {
  public:
    void configure(int dstWidth, int dstHeight);
    // Renders into RGBA8888 (R, G, B, A bytes), `dstStride` in pixels.
    void render(const PreviewInput& in, uint8_t* rgba, int dstStride);
    // Fills the output with the background (before the first frame).
    void clear(uint8_t* rgba, int dstStride) const;
    int width() const { return mW; }
    int height() const { return mH; }

  private:
    struct Affine {  // sensor = (a*X + b*Y + tx, c*X + d*Y + ty) for display (X, Y)
        float a, b, c, d, tx, ty;
    };
    struct Ring {
        float cx, cy, r, halfWidth;
        uint8_t color[3];
        float alpha;
        float arc;  // fraction of the circle drawn clockwise from the top (>= 1: full)
    };

    void buildHalf(const PreviewInput& in);
    void updateTone(const PreviewInput& in, const Affine& toSensor);
    void drawRing(uint8_t* rgba, int stride, const Ring& ring) const;

    int mW = 0, mH = 0;
    std::vector<uint8_t> mVignette;  // 0..255 per output pixel
    std::vector<uint8_t> mHalf;      // 2x2 box-filtered source (for strong downscaling)
    int mHalfW = 0, mHalfH = 0;
    uint8_t mLut[256][3] = {};

    // Smoothed state (display coordinates of the rotated sensor image).
    bool mHasView = false;
    float mCx = 0, mCy = 0, mSide = 0;
    int mLostFrames = 0;
    float mLo = -1, mHi = -1;
};

}  // namespace cityman::iris
