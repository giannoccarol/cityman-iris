// Internal C++ interfaces of the cityman-iris engine. Not installed, not ABI-stable.
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

#include "iris_engine.h"

namespace citymaniris {

// ---------------------------------------------------------------- images

// std::lround for |v| < 2^23 (half away from zero), inline: v - trunc(v) is exact in float.
inline int iround(float v) {
    const float t = float(int(v));
    const float d = v - t;
    return int(t) + (d >= 0.5f ? 1 : d <= -0.5f ? -1 : 0);
}

template <typename T>
struct Image {
    int w = 0, h = 0;
    std::vector<T> px;
    Image() = default;
    Image(int w_, int h_, T v = T()) : w(w_), h(h_), px(size_t(w_) * size_t(h_), v) {}
    T &at(int x, int y) { return px[size_t(y) * size_t(w) + size_t(x)]; }
    const T &at(int x, int y) const { return px[size_t(y) * size_t(w) + size_t(x)]; }
    T *row(int y) { return px.data() + size_t(y) * size_t(w); }
    const T *row(int y) const { return px.data() + size_t(y) * size_t(w); }
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w && y < h; }
};
using ImageU8 = Image<uint8_t>;
using ImageF = Image<float>;

// Input converted to 8 bits with an automatic gain (input percentile 99.5 -> 230).
struct Gray8 {
    ImageU8 img;
    int off_x = 0, off_y = 0;   // ROI origin in the input image
    float exposure = 0.f;       // input p99.5 / full scale
    float gain = 1.f;           // 8-bit output per input full-scale unit / 255
    float sat_level = 255.f;    // 8-bit level corresponding to input saturation
};

// Input after the automatic gain, converted lazily: only the coarse search image and the crops
// around the candidates are ever converted (the full-resolution frame never is).
struct InputMap {
    const uint8_t *base = nullptr;
    size_t stride = 0;
    int format = 0, maxv = 0;
    int x0 = 0, y0 = 0, w = 0, h = 0;  // ROI of the input; map coordinates are relative to it
    float exposure = 0.f, gain = 1.f, sat_level = 255.f;
    bool identity = false;             // 8-bit input with unit gain: plain copy
    std::vector<uint8_t> lut;
    std::vector<uint16_t> bin_of;
    std::vector<uint32_t> h4;
};

bool prepare_input(const iris_image &in, InputMap &m);
void convert_rect(const InputMap &m, int x, int y, int w, int h, ImageU8 &out);
void convert_rect_float(const InputMap &m, int x, int y, int w, int h, ImageF &out, std::vector<uint8_t> &tmp);
void convert_coarse(const InputMap &m, int f, int x, int y, int w, int h, ImageU8 &out,
                    std::vector<uint8_t> &rowbuf, std::vector<uint32_t> &acc);
bool convert_input(const iris_image &in, Gray8 &out);
ImageU8 downsample_box(const ImageU8 &src, int f);
ImageF to_float(const ImageU8 &src, int x0, int y0, int w, int h);
void gaussian_blur(const ImageF &src, ImageF &dst, float sigma);
void gaussian_blur(const ImageF &src, ImageF &dst, float sigma, ImageF &tmp, std::vector<float> &k,
                   std::vector<float> &col);
inline float bilinear(const ImageF &img, float x, float y, bool *ok) {
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

// ---------------------------------------------------------------- geometry

// Closed boundary as a function of angle around a centre: r(t) = a0 + sum_k a_k cos kt + b_k sin kt
struct Contour {
    float cx = 0, cy = 0;
    float a0 = 0;
    float a[3] = {0, 0, 0};  // harmonics 1..3
    float b[3] = {0, 0, 0};
    int order = 0;
    float radius(float t) const;
    float mean_radius() const { return a0; }
    float max_radius() const;
    // Upper bound of radius(t) for every t (a0 + sum of |harmonics|).
    float radius_bound() const {
        float r = a0;
        for (int k = 0; k < order; ++k) r += std::fabs(a[k]) + std::fabs(b[k]);
        return r;
    }
};

struct Parabola {
    bool present = false;
    float xv = 0, yv = 0, k = 0;  // y = yv + k * (x - xv)^2
    float y(float x) const { return yv + k * (x - xv) * (x - xv); }
};

struct Segmentation {
    Contour pupil;   // crop coordinates
    Contour iris;
    Parabola upper, lower;  // upper: masked when y < curve; lower: masked when y > curve
    float pupil_level = 0, iris_level = 0, sclera_level = 0;
    float pupil_contrast = 0, limbus_contrast = 0;
    float blur_sigma = 0;
    bool glint = false;
    float candidate_score = 0;
};

// Everything extract() measures; used by the debug tool to draw overlays.
struct ExtractDebug {
    Segmentation seg;
    int crop_x = 0, crop_y = 0;   // crop origin in Gray8 coordinates
    int roi_x = 0, roi_y = 0;     // Gray8 origin in input coordinates
    ImageF norm;                  // rubber sheet
    ImageU8 norm_valid;
    std::vector<float> cand_x, cand_y, cand_r, cand_score;  // input coordinates
    float exposure = 0;
};

// ---------------------------------------------------------------- reusable buffers

struct Run {
    int y, x0, x1;  // x1 exclusive
    int parent;
};
struct Comp {
    long area;
    int bx0, bx1, by0, by1;
    int first_run;
};

struct CropCtx {
    ImageF raw, s1, s2;  // raw, sigma 1, sigma 2
    ImageU8 glint;       // 1 = specular reflection (dilated)
    float glint_level = 255.f;
    bool in_glint(float x, float y) const {
        int xi = iround(x), yi = iround(y);
        return glint.inside(xi, yi) && glint.at(xi, yi);
    }
};

// One eye candidate that passed the geometry fits, with its own crop and noise mask. extract()
// runs the texture/occlusion gates on these in ranked order and keeps the first that is usable.
struct CandResult {
    Segmentation seg;
    int crop_x = 0, crop_y = 0;
    ImageF crop;   // raw crop (owns its buffer; recycled across calls)
    ImageU8 mask;  // noise mask
    int tier = 0;  // 1 = iris >= min radius
    float total = 0;
    iris_status sub_st = IRIS_OK;  // IRIS_Q_OVEREXPOSED when the iris ring is saturated
};

// Everything extract() would otherwise allocate per call. One per thread (iris_context).
struct Scratch {
    InputMap map;
    ImageU8 coarse;
    std::vector<uint8_t> rowbuf, convtmp;
    std::vector<uint32_t> rowacc;
    // pupil candidates
    std::vector<uint8_t> closed, ctmp, row_min, box;
    std::vector<Run> runs;
    std::vector<size_t> row_start;
    std::vector<Comp> comps;
    std::vector<int> comp_of, next_run;
    std::vector<int32_t> stack;
    // crops (current candidate being fitted)
    CropCtx cur;
    ImageU8 raw_g, mask, ring;
    // ranked eye candidates (buffers recycled)
    std::vector<CandResult> cands;
    size_t ncands = 0;
    ImageF blur_tmp;
    std::vector<float> kernel, col, vals;
    // rubber sheet and encoder
    ImageF norm;
    ImageU8 valid;
    struct Encode {
        ImageF sm;
        std::vector<float> kw, rowmean, are, aim, re_all, im_all;
        std::vector<std::complex<float>> spec, resp;
        std::vector<uint8_t> okv;
        std::vector<int> prefix;
    } enc;
};

// Expected iris position (input coordinates) to restrict the eye search; r <= 0: none.
struct Hint {
    float x = 0, y = 0, r = 0;
};

// ---------------------------------------------------------------- codes

struct Code {
    std::vector<uint64_t> bits, mask;  // cols * words_per_col, column-major
    float quality = 0;
    uint32_t valid_bits = 0;
};

struct Layout {
    uint16_t rows = 0, cols = 0;     // code rows / columns
    uint8_t n_filters = 0, words = 0; // words per column
    uint32_t encoder_id = 0;          // hash of everything that changes bit semantics
    bool operator==(const Layout &o) const {
        return rows == o.rows && cols == o.cols && n_filters == o.n_filters && words == o.words &&
               encoder_id == o.encoder_id;
    }
    uint32_t bits_total() const { return uint32_t(rows) * cols * n_filters * 2u; }
};

struct Fft {
    int n = 0;
    std::vector<std::complex<float>> tw;
    std::vector<int> rev;
    void init(int n_);
    void run(std::complex<float> *data, bool inverse) const;
};

}  // namespace citymaniris

struct iris_template {
    citymaniris::Layout layout;
    uint32_t flags = 0;
    std::vector<citymaniris::Code> codes;
};

struct iris_context {
    citymaniris::Scratch scratch;
};

struct iris_engine {
    iris_config cfg;
    citymaniris::Layout layout;
    citymaniris::Fft fft;
    std::vector<float> gabor[2];  // frequency responses, norm_cols each
};

namespace citymaniris {

iris_status extract(const iris_engine &eng, const iris_image &img, iris_template **out,
                    iris_quality *q, ExtractDebug *dbg);
iris_status extract(const iris_engine &eng, Scratch &sc, const iris_image &img, const Hint *hint,
                    iris_template **out, iris_quality *q, ExtractDebug *dbg);
// Fills sc.cands[0..return) with ranked eye candidates (best first). Returns the count; `st` is
// the rejection status to report when it returns 0.
size_t segment(const iris_engine &eng, Scratch &sc, const Hint *hint, iris_status &st,
               ExtractDebug *dbg);
void normalize_iris(const iris_engine &eng, const Segmentation &seg, const ImageF &crop,
                    const ImageU8 &noise_mask, ImageF &norm, ImageU8 &valid);
void encode(const iris_engine &eng, const ImageF &norm, const ImageU8 &valid, Code &code);
void encode(const iris_engine &eng, const ImageF &norm, const ImageU8 &valid, Code &code, Scratch::Encode &b);

struct PairScore {
    float score = 1.f, raw = 1.f;
    uint32_t bits = 0;
    int shift = 0;
};
PairScore compare_codes(const iris_engine &eng, const Code &a, const Code &b);
uint32_t compute_encoder_id(const iris_config &c);

}  // namespace citymaniris
