/*
 * SPDX-FileCopyrightText: 2026 giannoccarol
 * SPDX-License-Identifier: Apache-2.0
 */
#include "Preview.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace cityman::iris {

namespace {

constexpr float kPi = 3.14159265358979f;

// Background (outside the image) and the darkest vignette tone.
constexpr uint8_t kBg[3] = {6, 13, 22};
// Teal when the eye is good, amber when it is found but not usable (far, blurred, dark...).
constexpr uint8_t kTeal[3] = {0x4D, 0xD0, 0xE1};
constexpr uint8_t kAmber[3] = {0xFF, 0xB7, 0x4D};

// Cold monochrome ramp: night blue shadows -> steel blue -> pale cyan -> near white.
struct Stop {
    float t;
    float c[3];
};
constexpr Stop kRamp[] = {
        {0.00f, {11, 26, 42}},    // #0B1A2A
        {0.45f, {46, 88, 120}},   // #2E5878
        {0.80f, {156, 199, 222}}, // #9CC7DE
        {0.93f, {205, 239, 255}}, // #CDEFFF
        {1.00f, {244, 252, 255}}, // #F4FCFF
};

constexpr float kGamma = 0.85f;
constexpr float kSmoothView = 0.35f;   // per lit frame (about 7.5 per second)
constexpr float kSmoothReturn = 0.15f;
constexpr float kSmoothTone = 0.30f;
constexpr int kLostFramesBeforeReturn = 6;

float smoothstep(float e0, float e1, float x) {
    const float t = std::clamp((x - e0) / (e1 - e0), 0.f, 1.f);
    return t * t * (3.f - 2.f * t);
}

}  // namespace

void PreviewRenderer::configure(int w, int h) {
    mW = std::max(1, w);
    mH = std::max(1, h);
    mVignette.assign(size_t(mW) * mH, 255);
    const float half = 0.5f * float(std::min(mW, mH));
    for (int y = 0; y < mH; y++) {
        for (int x = 0; x < mW; x++) {
            const float dx = (x + 0.5f - 0.5f * mW) / half;
            const float dy = (y + 0.5f - 0.5f * mH) / half;
            const float r = std::sqrt(dx * dx + dy * dy);
            const float v = 1.f - 0.62f * smoothstep(0.50f, 1.06f, r);
            mVignette[size_t(y) * mW + x] = uint8_t(std::lround(v * 255.f));
        }
    }
    mHasView = false;
    mLo = mHi = -1;
    mLostFrames = 0;
}

void PreviewRenderer::clear(uint8_t* rgba, int stride) const {
    for (int y = 0; y < mH; y++) {
        uint8_t* p = rgba + size_t(y) * stride * 4;
        const uint8_t* vig = mVignette.data() + size_t(y) * mW;
        for (int x = 0; x < mW; x++, p += 4) {
            const int v = vig[x];
            for (int k = 0; k < 3; k++) {
                const int c0 = kRamp[0].c[k];
                p[k] = uint8_t(kBg[k] + ((c0 - kBg[k]) * v) / 255);
            }
            p[3] = 255;
        }
    }
}

void PreviewRenderer::buildHalf(const PreviewInput& in) {
    const int hw = in.width / 2, hh = in.height / 2;
    if (hw != mHalfW || hh != mHalfH) {
        mHalfW = hw;
        mHalfH = hh;
        mHalf.resize(size_t(hw) * hh);  // only when the source size changes
    }
    for (int y = 0; y < hh; y++) {
        const uint8_t* r0 = in.y + size_t(2 * y) * in.stride;
        const uint8_t* r1 = r0 + in.stride;
        uint8_t* d = mHalf.data() + size_t(y) * hw;
        for (int x = 0; x < hw; x++) {
            d[x] = uint8_t((r0[2 * x] + r0[2 * x + 1] + r1[2 * x] + r1[2 * x + 1] + 2) >> 2);
        }
    }
}

void PreviewRenderer::updateTone(const PreviewInput& in, const Affine& A) {
    // Percentiles over a coarse grid of the visible part of the image.
    uint32_t hist[256] = {};
    uint32_t n = 0;
    const float s = mSide / float(std::min(mW, mH));
    constexpr int kGrid = 48;
    for (int j = 0; j < kGrid; j++) {
        for (int i = 0; i < kGrid; i++) {
            const float X = mCx + ((i + 0.5f) / kGrid - 0.5f) * mW * s;
            const float Y = mCy + ((j + 0.5f) / kGrid - 0.5f) * mH * s;
            const int x = int(A.a * X + A.b * Y + A.tx);
            const int y = int(A.c * X + A.d * Y + A.ty);
            if (x < 0 || y < 0 || x >= in.width || y >= in.height) continue;
            hist[in.y[size_t(y) * in.stride + x]]++;
            n++;
        }
    }
    float lo = 0, hi = 255;
    if (n > 0) {
        const uint32_t pLo = n * 2 / 100, pHi = n * 995 / 1000;
        uint32_t acc = 0;
        int iLo = -1, iHi = 255;
        for (int i = 0; i < 256; i++) {
            acc += hist[i];
            if (iLo < 0 && acc > pLo) iLo = i;
            if (acc > pHi) {
                iHi = i;
                break;
            }
        }
        lo = float(std::max(iLo, 0));
        hi = float(iHi);
    }
    if (hi - lo < 24.f) {
        const float mid = 0.5f * (hi + lo);
        lo = std::max(0.f, mid - 12.f);
        hi = std::min(255.f, lo + 24.f);
    }
    if (mLo < 0) {
        mLo = lo;
        mHi = hi;
    } else {
        mLo += kSmoothTone * (lo - mLo);
        mHi += kSmoothTone * (hi - mHi);
    }
    for (int v = 0; v < 256; v++) {
        float t = std::clamp((v - mLo) / (mHi - mLo), 0.f, 1.f);
        t = std::pow(t, kGamma);
        size_t k = 1;
        while (k + 1 < std::size(kRamp) && t > kRamp[k].t) k++;
        const Stop& a = kRamp[k - 1];
        const Stop& b = kRamp[k];
        const float f = std::clamp((t - a.t) / (b.t - a.t), 0.f, 1.f);
        for (int c = 0; c < 3; c++) {
            mLut[v][c] = uint8_t(std::lround(a.c[c] + (b.c[c] - a.c[c]) * f));
        }
    }
}

void PreviewRenderer::drawRing(uint8_t* rgba, int stride, const Ring& g) const {
    const float ext = g.r + g.halfWidth + 2.f;
    const int x0 = std::max(0, int(std::floor(g.cx - ext)));
    const int x1 = std::min(mW - 1, int(std::ceil(g.cx + ext)));
    const int y0 = std::max(0, int(std::floor(g.cy - ext)));
    const int y1 = std::min(mH - 1, int(std::ceil(g.cy + ext)));
    const float limit = std::clamp(g.arc, 0.f, 1.f) * 2.f * kPi;
    if (limit <= 0.f) return;
    for (int y = y0; y <= y1; y++) {
        uint8_t* p = rgba + (size_t(y) * stride + x0) * 4;
        for (int x = x0; x <= x1; x++, p += 4) {
            const float dx = x - g.cx, dy = y - g.cy;
            const float d = std::sqrt(dx * dx + dy * dy);
            float cov = std::clamp(g.halfWidth + 0.5f - std::fabs(d - g.r), 0.f, 1.f);
            if (cov <= 0.f) continue;
            if (g.arc < 1.f) {
                float th = std::atan2(dx, -dy);  // clockwise from the top
                if (th < 0.f) th += 2.f * kPi;
                const float edge = std::min(th, limit - th) * d;  // arc length to the nearest end
                cov *= std::clamp(edge + 0.5f, 0.f, 1.f);
                if (cov <= 0.f) continue;
            }
            const float a = cov * g.alpha;
            for (int k = 0; k < 3; k++) {
                p[k] = uint8_t(std::lround(p[k] + (g.color[k] - p[k]) * a));
            }
        }
    }
}

void PreviewRenderer::render(const PreviewInput& in, uint8_t* rgba, int stride) {
    // --- geometry: display = sensor rotated clockwise by `rotation`, then mirrored ----------
    const int rot = ((in.rotation % 360) + 360) % 360;
    const bool swap = rot == 90 || rot == 270;
    const int Wd = swap ? in.height : in.width;
    const int Hd = swap ? in.width : in.height;
    const float W1 = float(in.width - 1), H1 = float(in.height - 1);
    Affine A;  // display -> sensor
    switch (rot) {
        case 90:
            A = {0, 1, -1, 0, 0, H1};
            break;
        case 180:
            A = {-1, 0, 0, -1, W1, H1};
            break;
        case 270:
            A = {0, -1, 1, 0, W1, 0};
            break;
        default:
            A = {1, 0, 0, 1, 0, 0};
            break;
    }
    if (in.mirror) {  // X -> Wd - 1 - X
        A.tx += A.a * (Wd - 1);
        A.ty += A.c * (Wd - 1);
        A.a = -A.a;
        A.c = -A.c;
    }
    // sensor -> display (A is orthogonal: the inverse is the transpose).
    auto toDisplay = [&](float x, float y, float* X, float* Y) {
        *X = A.a * (x - A.tx) + A.c * (y - A.ty);
        *Y = A.b * (x - A.tx) + A.d * (y - A.ty);
    };

    // --- view: square crop around the eyes, smoothed ------------------------------------------
    const float minD = float(std::min(Wd, Hd));
    float tx = 0.5f * Wd, ty = 0.5f * Hd, tside = minD;
    float ex[2], ey[2], er[2];
    int found = 0;
    for (const auto& e : in.eyes) {
        if (!e.found || e.irisR <= 0.f) continue;
        toDisplay(e.irisX, e.irisY, &ex[found], &ey[found]);
        er[found] = e.irisR;
        found++;
    }
    if (found == 2) {
        tx = 0.5f * (ex[0] + ex[1]);
        ty = 0.5f * (ey[0] + ey[1]);
        const float dist = std::hypot(ex[0] - ex[1], ey[0] - ey[1]);
        tside = std::max(dist * 2.1f, std::max(er[0], er[1]) * 8.f);
    } else if (found == 1) {
        tx = ex[0];
        ty = ey[0];
        tside = er[0] * 10.f;
    }
    tside = std::clamp(tside, 0.30f * minD, minD);
    float alpha = kSmoothView;
    if (found) {
        mLostFrames = 0;
    } else if (++mLostFrames <= kLostFramesBeforeReturn && mHasView) {
        tx = mCx;
        ty = mCy;
        tside = mSide;
    } else {
        alpha = kSmoothReturn;
    }
    if (!mHasView) {
        mCx = tx;
        mCy = ty;
        mSide = tside;
        mHasView = true;
    } else {
        mCx += alpha * (tx - mCx);
        mCy += alpha * (ty - mCy);
        mSide += alpha * (tside - mSide);
    }
    // Keep the square inside the image when it fits.
    const float halfSide = 0.5f * mSide;
    if (mSide <= Wd) mCx = std::clamp(mCx, halfSide, Wd - halfSide);
    if (mSide <= Hd) mCy = std::clamp(mCy, halfSide, Hd - halfSide);

    updateTone(in, A);

    // --- image ------------------------------------------------------------------------------
    const float s = mSide / float(std::min(mW, mH));  // source pixels per output pixel
    const bool useHalf = s > 1.6f && in.width >= 4 && in.height >= 4;
    const uint8_t* src = in.y;
    int sw = in.width, sh = in.height, sstride = in.stride;
    float scale = 1.f, offset = 0.f;
    if (useHalf) {
        buildHalf(in);
        src = mHalf.data();
        sw = mHalfW;
        sh = mHalfH;
        sstride = mHalfW;
        scale = 0.5f;
        offset = -0.25f;  // half pixel i is centred on sensor 2i + 0.5
    }
    constexpr int kFix = 16;
    const float one = float(1 << kFix);
    const int32_t dfx = int32_t(std::lround(A.a * s * scale * one));
    const int32_t dfy = int32_t(std::lround(A.c * s * scale * one));
    const int32_t maxX = (sw - 1) << kFix, maxY = (sh - 1) << kFix;
    uint8_t bgv[256][3];  // background mixed by vignette value (constant per frame: cheap)
    for (int v = 0; v < 256; v++) {
        for (int k = 0; k < 3; k++) bgv[v][k] = uint8_t(kBg[k] + ((kRamp[0].c[k] - kBg[k]) * v) / 255);
    }
    for (int v = 0; v < mH; v++) {
        const float X = mCx + (0.5f - 0.5f * mW) * s;
        const float Y = mCy + (v + 0.5f - 0.5f * mH) * s;
        int32_t fx = int32_t(std::lround(((A.a * X + A.b * Y + A.tx) * scale + offset) * one));
        int32_t fy = int32_t(std::lround(((A.c * X + A.d * Y + A.ty) * scale + offset) * one));
        uint8_t* p = rgba + size_t(v) * stride * 4;
        const uint8_t* vig = mVignette.data() + size_t(v) * mW;
        for (int u = 0; u < mW; u++, p += 4, fx += dfx, fy += dfy) {
            const int vg = vig[u];
            if (fx < 0 || fy < 0 || fx >= maxX || fy >= maxY) {
                p[0] = bgv[vg][0];
                p[1] = bgv[vg][1];
                p[2] = bgv[vg][2];
                p[3] = 255;
                continue;
            }
            const int ix = fx >> kFix, iy = fy >> kFix;
            const int wx = (fx >> (kFix - 8)) & 255, wy = (fy >> (kFix - 8)) & 255;
            const uint8_t* q = src + size_t(iy) * sstride + ix;
            const int top = q[0] * (256 - wx) + q[1] * wx;
            const int bot = q[sstride] * (256 - wx) + q[sstride + 1] * wx;
            const int lum = (top * (256 - wy) + bot * wy + (1 << 15)) >> 16;
            const uint8_t* c = mLut[lum];
            p[0] = uint8_t(kBg[0] + ((c[0] - kBg[0]) * vg) / 255);
            p[1] = uint8_t(kBg[1] + ((c[1] - kBg[1]) * vg) / 255);
            p[2] = uint8_t(kBg[2] + ((c[2] - kBg[2]) * vg) / 255);
            p[3] = 255;
        }
    }

    // --- overlays ---------------------------------------------------------------------------
    for (const auto& e : in.eyes) {
        if (!e.found || e.irisR <= 0.f) continue;
        float X, Y, PX, PY;
        toDisplay(e.irisX, e.irisY, &X, &Y);
        toDisplay(e.pupilX, e.pupilY, &PX, &PY);
        const float cx = (X - mCx) / s + 0.5f * mW - 0.5f;
        const float cy = (Y - mCy) / s + 0.5f * mH - 0.5f;
        const float pcx = (PX - mCx) / s + 0.5f * mW - 0.5f;
        const float pcy = (PY - mCy) / s + 0.5f * mH - 0.5f;
        const float r = e.irisR / s;
        const float pr = std::max(e.pupilR / s, 2.5f);
        const uint8_t* col = e.good ? kTeal : kAmber;
        auto ring = [&](float x, float y, float rad, float hw, const uint8_t* c, float a,
                        float arc) {
            Ring g{x, y, rad, hw, {c[0], c[1], c[2]}, a, arc};
            drawRing(rgba, stride, g);
        };
        ring(cx, cy, r, 3.0f, col, 0.16f, 1.f);   // soft glow
        ring(cx, cy, r, 0.9f, col, 0.92f, 1.f);   // iris boundary
        ring(pcx, pcy, pr, 0.75f, col, 0.70f, 1.f);  // pupil
        const float ar = r * 1.22f + 7.f;
        ring(cx, cy, ar, 1.6f, kTeal, 0.22f, 1.f);  // progress track
        if (e.progress > 0.f) ring(cx, cy, ar, 1.6f, kTeal, 0.95f, e.progress);
    }
}

}  // namespace cityman::iris
