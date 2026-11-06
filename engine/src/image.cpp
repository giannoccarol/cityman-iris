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

bool prepare_input(const iris_image &in, InputMap &m) {
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
    m.base = base;
    m.stride = stride;
    m.format = in.format;
    m.maxv = int(full);
    m.x0 = x0, m.y0 = y0, m.w = w, m.h = h;

    // Pass 1: histogram (1024 bins of the normalized value) on a sparse grid. Four interleaved
    // histograms keep uniform regions from serializing on a single counter.
    m.h4.assign(4 * 1024, 0);
    uint64_t count = 0;
    const int maxv = m.maxv;
    // bin index for every possible input code (up to 16 bits)
    m.bin_of.resize(size_t(maxv) + 1);
    for (int v = 0; v <= maxv; ++v) m.bin_of[size_t(v)] = uint16_t(std::min(1023, int(float(v) / full * 1023.f + 0.5f)));
    const int step = (size_t(w) * size_t(h) > 400000u) ? 2 : 1;
    uint32_t *h4 = m.h4.data();
    const uint16_t *bin_of = m.bin_of.data();
    for (int y = y0; y < y0 + h; y += step) {
        const uint8_t *row = base + size_t(y) * stride;
        int k = 0;
        switch (in.format) {
            case IRIS_PIX_GRAY8:
                for (int x = x0; x < x0 + w; x += step, ++k) h4[size_t(k & 3) * 1024 + bin_of[row[x]]]++;
                break;
            case IRIS_PIX_GRAY16:
                for (int x = x0; x < x0 + w; x += step, ++k) {
                    uint16_t v;
                    std::memcpy(&v, row + size_t(x) * 2, 2);
                    h4[size_t(k & 3) * 1024 + bin_of[std::min<int>(v, maxv)]]++;
                }
                break;
            default:
                for (int x = x0; x < x0 + w; x += step, ++k) h4[size_t(k & 3) * 1024 + bin_of[raw10_at(row, x)]]++;
                break;
        }
        count += uint64_t(k);
    }
    uint64_t target = uint64_t(double(count) * 0.995), acc = 0;
    int p995 = 1023;
    for (int b = 0; b < 1024; ++b) {
        acc += uint64_t(h4[size_t(b)]) + h4[1024 + size_t(b)] + h4[2048 + size_t(b)] + h4[3072 + size_t(b)];
        if (acc >= target) {
            p995 = b;
            break;
        }
    }
    m.exposure = float(p995) / 1023.f;
    // Map p99.5 to 230; never amplify more than 16x, never compress 8-bit input.
    float g = 230.f / (255.f * std::max(m.exposure, 1e-3f));
    float gmin = (in.format == IRIS_PIX_GRAY8) ? 1.f : 0.5f;
    g = std::min(16.f, std::max(gmin, g));
    m.gain = g;
    m.sat_level = 0.98f * 255.f * g;
    m.identity = in.format == IRIS_PIX_GRAY8 && g == 1.f;
    // Lookup table from input code to 8-bit output.
    const float k = 255.f * g / full;
    m.lut.resize(size_t(maxv) + 1);
    for (int v = 0; v <= maxv; ++v) {
        float o = float(v) * k + 0.5f;
        m.lut[size_t(v)] = uint8_t(o >= 255.f ? 255 : o);
    }
    return true;
}

// Converts row `y` (map coordinates), columns [x, x + n), into dst.
static void convert_row(const InputMap &m, int x, int y, int n, uint8_t *dst) {
    const uint8_t *row = m.base + size_t(y + m.y0) * m.stride;
    const int xs = x + m.x0;
    const uint8_t *lut = m.lut.data();
    if (m.format == IRIS_PIX_GRAY8) {
        if (m.identity) std::memcpy(dst, row + xs, size_t(n));
        else
            for (int i = 0; i < n; ++i) dst[i] = lut[row[xs + i]];
    } else if (m.format == IRIS_PIX_GRAY16) {
        for (int i = 0; i < n; ++i) {
            uint16_t v;
            std::memcpy(&v, row + size_t(xs + i) * 2, 2);
            dst[i] = lut[size_t(std::min<int>(v, m.maxv))];
        }
    } else {
        for (int i = 0; i < n; ++i) dst[i] = lut[size_t(raw10_at(row, xs + i))];
    }
}

void convert_rect(const InputMap &m, int x, int y, int w, int h, ImageU8 &out) {
    if (out.w != w || out.h != h || out.px.size() != size_t(w) * h) {
        out.w = w, out.h = h;
        out.px.resize(size_t(w) * h);
    }
    for (int r = 0; r < h; ++r) convert_row(m, x, y + r, w, out.row(r));
}

void convert_rect_float(const InputMap &m, int x, int y, int w, int h, ImageF &out,
                        std::vector<uint8_t> &tmp) {
    if (out.w != w || out.h != h || out.px.size() != size_t(w) * h) {
        out.w = w, out.h = h;
        out.px.resize(size_t(w) * h);
    }
    tmp.resize(size_t(w));
    for (int r = 0; r < h; ++r) {
        convert_row(m, x, y + r, w, tmp.data());
        float *d = out.row(r);
        for (int i = 0; i < w; ++i) d[i] = float(tmp[size_t(i)]);
    }
}

void convert_coarse(const InputMap &m, int f, int x, int y, int w, int h, ImageU8 &out,
                    std::vector<uint8_t> &rowbuf, std::vector<uint32_t> &acc) {
    // Box mean of the raw codes, then the gain: one table lookup per coarse pixel instead of one
    // per input pixel. Only the candidate search reads this image (the fits use convert_rect).
    // With unit gain on 8-bit input it equals the box mean of the converted image.
    const int cw = w / f, ch = h / f;
    if (out.w != cw || out.h != ch || out.px.size() != size_t(cw) * ch) {
        out.w = cw, out.h = ch;
        out.px.resize(size_t(cw) * ch);
    }
    (void)rowbuf;
    acc.resize(size_t(cw));
    const uint32_t area = uint32_t(f * f);
    const float k = m.gain * 255.f / float(m.maxv) / float(area);
    for (int cy = 0; cy < ch; ++cy) {
        std::fill(acc.begin(), acc.end(), 0u);
        for (int dy = 0; dy < f; ++dy) {
            const uint8_t *row = m.base + size_t(y + cy * f + dy + m.y0) * m.stride;
            const int xs = x + m.x0;
            uint32_t *a = acc.data();
            if (m.format == IRIS_PIX_GRAY8) {
                const uint8_t *s = row + xs;
                if (f == 2) {
                    for (int i = 0; i < cw; ++i) a[i] += uint32_t(s[2 * i]) + s[2 * i + 1];
                } else {
                    for (int i = 0; i < cw; ++i)
                        for (int dx = 0; dx < f; ++dx) a[i] += s[i * f + dx];
                }
            } else if (m.format == IRIS_PIX_GRAY16) {
                for (int i = 0; i < cw; ++i)
                    for (int dx = 0; dx < f; ++dx) {
                        uint16_t v;
                        std::memcpy(&v, row + size_t(xs + i * f + dx) * 2, 2);
                        a[i] += uint32_t(std::min<int>(v, m.maxv));
                    }
            } else {
                for (int i = 0; i < cw; ++i)
                    for (int dx = 0; dx < f; ++dx) a[i] += uint32_t(raw10_at(row, xs + i * f + dx));
            }
        }
        uint8_t *d = out.row(cy);
        if (m.identity) {
            for (int i = 0; i < cw; ++i) d[i] = uint8_t((acc[size_t(i)] + area / 2) / area);
        } else {
            for (int i = 0; i < cw; ++i) {
                const float o = float(acc[size_t(i)]) * k + 0.5f;
                d[i] = uint8_t(o >= 255.f ? 255 : o);
            }
        }
    }
}

bool convert_input(const iris_image &in, Gray8 &out) {
    InputMap m;
    if (!prepare_input(in, m)) return false;
    out.exposure = m.exposure;
    out.gain = m.gain;
    out.sat_level = m.sat_level;
    out.off_x = m.x0;
    out.off_y = m.y0;
    convert_rect(m, 0, 0, m.w, m.h, out.img);
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
    ImageF tmp;
    std::vector<float> k, col;
    gaussian_blur(src, dst, sigma, tmp, k, col);
}

static void resize_like(ImageF &img, int w, int h) {
    if (img.w != w || img.h != h || img.px.size() != size_t(w) * h) {
        img.w = w, img.h = h;
        img.px.resize(size_t(w) * h);
    }
}

// Separable Gaussian with clamped borders. The taps are always summed in the same order
// (i = -r..r) as the plain per-pixel loop, so the result is bit-identical to it; only the
// interior skips the clamping and runs tap-outer so that it vectorizes.
void gaussian_blur(const ImageF &src, ImageF &dst, float sigma, ImageF &tmp, std::vector<float> &k,
                   std::vector<float> &col) {
    if (sigma <= 0.f) {
        dst = src;
        return;
    }
    int r = std::max(1, int(std::ceil(3.f * sigma)));
    k.resize(static_cast<size_t>(2 * r + 1));
    float s = 0;
    for (int i = -r; i <= r; ++i) s += (k[size_t(i + r)] = std::exp(-0.5f * float(i * i) / (sigma * sigma)));
    for (auto &v : k) v /= s;
    resize_like(tmp, src.w, src.h);
    const int w = src.w;
    const int lo = std::min(r, w), hi = std::max(lo, w - r);  // interior: [lo, hi)
    for (int y = 0; y < src.h; ++y) {
        const float *in = src.row(y);
        float *o = tmp.row(y);
        auto clamped = [&](int x) {
            float acc = 0;
            for (int i = -r; i <= r; ++i) {
                int xx = std::min(w - 1, std::max(0, x + i));
                acc += k[size_t(i + r)] * in[xx];
            }
            o[x] = acc;
        };
        for (int x = 0; x < lo; ++x) clamped(x);
        for (int x = lo; x < hi; ++x) o[x] = 0.f;
        for (int i = -r; i <= r; ++i) {
            const float kv = k[size_t(i + r)];
            const float *iin = in + i;
            for (int x = lo; x < hi; ++x) o[x] += kv * iin[x];
        }
        for (int x = hi; x < w; ++x) clamped(x);
    }
    resize_like(dst, src.w, src.h);
    col.resize(static_cast<size_t>(src.w));
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
