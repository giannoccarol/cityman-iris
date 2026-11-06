// Input conversion and basic image operations.
#include <algorithm>
#include <cmath>
#include <cstring>

#include "iris_internal.h"

namespace citymaniris {

namespace {

inline int raw10_at(const uint8_t *row, int x) {
    const uint8_t *g = row + (x >> 2) * 5;
    int i = x & 3;
    return (int(g[i]) << 2) | ((g[4] >> (2 * i)) & 3);
}

}  // namespace

bool convert_input(const iris_image &in, Gray8 &out) {
    if (!in.data || in.width <= 0 || in.height <= 0) return false;
    int bits = 8;
    size_t min_stride;
    switch (in.format) {
        case IRIS_PIX_GRAY8: min_stride = size_t(in.width); break;
        case IRIS_PIX_GRAY16:
            bits = in.bits ? in.bits : 10;
            if (bits < 1 || bits > 16) return false;
            min_stride = size_t(in.width) * 2;
            break;
        case IRIS_PIX_RAW10_MIPI:
            bits = 10;
            min_stride = (size_t(in.width) + 3) / 4 * 5;
            break;
        default: return false;
    }
    size_t stride = in.stride ? size_t(in.stride) : min_stride;
    if (in.stride < 0 || stride < min_stride) return false;

    int x0 = 0, y0 = 0, w = in.width, h = in.height;
    if (in.roi_w > 0 && in.roi_h > 0) {
        x0 = std::max(0, in.roi_x);
        y0 = std::max(0, in.roi_y);
        int x1 = std::min(in.width, in.roi_x + in.roi_w);
        int y1 = std::min(in.height, in.roi_y + in.roi_h);
        if (x1 <= x0 || y1 <= y0) return false;
        w = x1 - x0;
        h = y1 - y0;
    }
    const float full = float((1u << bits) - 1u);
    const uint8_t *base = static_cast<const uint8_t *>(in.data);

    // Pass 1: histogram of the normalized value (1024 bins) on a sparse grid.
    std::vector<uint32_t> hist(1024, 0);
    uint64_t count = 0;
    auto value = [&](int x, int y) -> int {
        const uint8_t *row = base + size_t(y) * stride;
        switch (in.format) {
            case IRIS_PIX_GRAY8: return row[x];
            case IRIS_PIX_GRAY16: {
                uint16_t v;
                std::memcpy(&v, row + size_t(x) * 2, 2);
                return std::min<int>(v, int(full));
            }
            default: return raw10_at(row, x);
        }
    };
    const int step = (size_t(w) * size_t(h) > 400000u) ? 2 : 1;
    for (int y = y0; y < y0 + h; y += step)
        for (int x = x0; x < x0 + w; x += step) {
            int b = int(float(value(x, y)) / full * 1023.f + 0.5f);
            hist[size_t(std::min(1023, std::max(0, b)))]++;
            count++;
        }
    uint64_t target = uint64_t(double(count) * 0.995), acc = 0;
    int p995 = 1023;
    for (int b = 0; b < 1024; ++b) {
        acc += hist[size_t(b)];
        if (acc >= target) {
            p995 = b;
            break;
        }
    }
    out.exposure = float(p995) / 1023.f;
    // Map p99.5 to 230; never amplify more than 16x, never compress 8-bit input.
    float g = 230.f / (255.f * std::max(out.exposure, 1e-3f));
    float gmin = (in.format == IRIS_PIX_GRAY8) ? 1.f : 0.5f;
    g = std::min(16.f, std::max(gmin, g));
    out.gain = g;
    out.sat_level = 0.98f * 255.f * g;
    out.off_x = x0;
    out.off_y = y0;
    out.img = ImageU8(w, h);
    const float k = 255.f * g / full;
    for (int y = 0; y < h; ++y) {
        uint8_t *dst = out.img.row(y);
        if (in.format == IRIS_PIX_GRAY8 && g == 1.f) {
            std::memcpy(dst, base + size_t(y + y0) * stride + x0, size_t(w));
            continue;
        }
        for (int x = 0; x < w; ++x) {
            float v = float(value(x + x0, y + y0)) * k + 0.5f;
            dst[x] = uint8_t(v >= 255.f ? 255 : v);
        }
    }
    return true;
}

ImageU8 downsample_box(const ImageU8 &src, int f) {
    if (f <= 1) return src;
    ImageU8 dst(src.w / f, src.h / f);
    const int area = f * f;
    std::vector<uint32_t> acc(static_cast<size_t>(dst.w));
    for (int y = 0; y < dst.h; ++y) {
        std::fill(acc.begin(), acc.end(), 0u);
        for (int dy = 0; dy < f; ++dy) {
            const uint8_t *s = src.row(y * f + dy);
            for (int x = 0; x < dst.w; ++x) {
                uint32_t sum = 0;
                for (int dx = 0; dx < f; ++dx) sum += s[x * f + dx];
                acc[size_t(x)] += sum;
            }
        }
        uint8_t *d = dst.row(y);
        for (int x = 0; x < dst.w; ++x) d[x] = uint8_t((acc[size_t(x)] + uint32_t(area / 2)) / uint32_t(area));
    }
    return dst;
}

ImageF to_float(const ImageU8 &src, int x0, int y0, int w, int h) {
    ImageF dst(w, h);
    for (int y = 0; y < h; ++y) {
        const uint8_t *s = src.row(y0 + y) + x0;
        float *d = dst.row(y);
        for (int x = 0; x < w; ++x) d[x] = float(s[x]);
    }
    return dst;
}

void gaussian_blur(const ImageF &src, ImageF &dst, float sigma) {
    if (sigma <= 0.f) {
        dst = src;
        return;
    }
    int r = std::max(1, int(std::ceil(3.f * sigma)));
    std::vector<float> k(static_cast<size_t>(2 * r + 1));
    float s = 0;
    for (int i = -r; i <= r; ++i) s += (k[size_t(i + r)] = std::exp(-0.5f * float(i * i) / (sigma * sigma)));
    for (auto &v : k) v /= s;
    ImageF tmp(src.w, src.h);
    for (int y = 0; y < src.h; ++y) {
        const float *in = src.row(y);
        float *o = tmp.row(y);
        for (int x = 0; x < src.w; ++x) {
            float acc = 0;
            for (int i = -r; i <= r; ++i) {
                int xx = std::min(src.w - 1, std::max(0, x + i));
                acc += k[size_t(i + r)] * in[xx];
            }
            o[x] = acc;
        }
    }
    dst = ImageF(src.w, src.h);
    std::vector<float> col(static_cast<size_t>(src.w));
    for (int y = 0; y < src.h; ++y) {
        std::fill(col.begin(), col.end(), 0.f);
        for (int i = -r; i <= r; ++i) {
            int yy = std::min(src.h - 1, std::max(0, y + i));
            const float *in = tmp.row(yy);
            float kv = k[size_t(i + r)];
            for (int x = 0; x < src.w; ++x) col[size_t(x)] += kv * in[x];
        }
        std::copy(col.begin(), col.end(), dst.row(y));
    }
}

float bilinear(const ImageF &img, float x, float y, bool *ok) {
    if (!(x >= 0.f && y >= 0.f && x <= float(img.w - 1) && y <= float(img.h - 1))) {
        if (ok) *ok = false;
        return 0.f;
    }
    int x0 = std::min(int(x), img.w - 2), y0 = std::min(int(y), img.h - 2);
    if (img.w < 2 || img.h < 2) {
        if (ok) *ok = false;
        return 0.f;
    }
    float fx = x - float(x0), fy = y - float(y0);
    const float *r0 = img.row(y0), *r1 = img.row(y0 + 1);
    float a = r0[x0] + fx * (r0[x0 + 1] - r0[x0]);
    float b = r1[x0] + fx * (r1[x0 + 1] - r1[x0]);
    if (ok) *ok = true;
    return a + fy * (b - a);
}

float Contour::radius(float t) const {
    float r = a0;
    for (int k = 0; k < order; ++k) r += a[k] * std::cos(float(k + 1) * t) + b[k] * std::sin(float(k + 1) * t);
    return r;
}

float Contour::max_radius() const {
    float m = 0;
    for (int i = 0; i < 64; ++i) m = std::max(m, radius(float(i) * 6.2831853f / 64.f));
    return m;
}

}  // namespace citymaniris
