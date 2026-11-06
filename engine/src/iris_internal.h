// Internal C++ interfaces of the cityman-iris engine. Not installed, not ABI-stable.
#pragma once

#include <complex>
#include <cstdint>
#include <vector>

#include "iris_engine.h"

namespace citymaniris {

// ---------------------------------------------------------------- images

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

bool convert_input(const iris_image &in, Gray8 &out);
ImageU8 downsample_box(const ImageU8 &src, int f);
ImageF to_float(const ImageU8 &src, int x0, int y0, int w, int h);
void gaussian_blur(const ImageF &src, ImageF &dst, float sigma);
float bilinear(const ImageF &img, float x, float y, bool *ok);

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

struct iris_engine {
    iris_config cfg;
    citymaniris::Layout layout;
    citymaniris::Fft fft;
    std::vector<float> gabor[2];  // frequency responses, norm_cols each
};

namespace citymaniris {

iris_status extract(const iris_engine &eng, const iris_image &img, iris_template **out,
                    iris_quality *q, ExtractDebug *dbg);
bool segment(const iris_engine &eng, const Gray8 &g, Segmentation &seg, int &crop_x, int &crop_y,
             ImageF &crop, ImageU8 &noise_mask, iris_status &st, ExtractDebug *dbg);
void normalize_iris(const iris_engine &eng, const Segmentation &seg, const ImageF &crop,
                    const ImageU8 &noise_mask, ImageF &norm, ImageU8 &valid);
void encode(const iris_engine &eng, const ImageF &norm, const ImageU8 &valid, Code &code);

struct PairScore {
    float score = 1.f, raw = 1.f;
    uint32_t bits = 0;
    int shift = 0;
};
PairScore compare_codes(const iris_engine &eng, const Code &a, const Code &b);
uint32_t compute_encoder_id(const iris_config &c);

}  // namespace citymaniris
