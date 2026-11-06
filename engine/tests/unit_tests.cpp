// Host unit tests for the parts of the engine that do not need eye images:
// API contracts, FFT, encoder/matcher on rubber-sheet arrays, serialization, enrollment,
// input conversion, thread safety, timing of encode and match.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <random>
#include <thread>
#include <vector>

#include "eye_frame.h"
#include "iris_engine.h"
#include "iris_internal.h"

using namespace citymaniris;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond)                                                          \
    do {                                                                     \
        if (cond) {                                                          \
            g_pass++;                                                        \
        } else {                                                             \
            g_fail++;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
        }                                                                    \
    } while (0)

namespace {

double now_ms() {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

// Smooth random field on the rubber sheet (periodic in angle), used only to exercise the
// encoder: rows x cols, sum of random low-frequency sinusoids plus white noise.
ImageF random_sheet(int rows, int cols, uint32_t seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> n01(0.f, 1.f);
    std::uniform_real_distribution<float> u01(0.f, 1.f);
    ImageF img(cols, rows, 0.f);
    for (int k = 0; k < 60; ++k) {
        const int fa = 1 + int(u01(rng) * 40.f);
        const float fr = u01(rng) * 3.f, ph = u01(rng) * 6.2831853f, a = n01(rng) / std::sqrt(float(fa));
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                img.at(j, i) += a * std::cos(6.2831853f * (float(fa * j) / float(cols) + fr * float(i) / float(rows)) + ph);
    }
    for (auto &v : img.px) v = 100.f + 20.f * v;
    return img;
}

ImageF shift_sheet(const ImageF &s, int cols_shift) {
    ImageF o(s.w, s.h);
    for (int i = 0; i < s.h; ++i)
        for (int j = 0; j < s.w; ++j) o.at((j + cols_shift + s.w) % s.w, i) = s.at(j, i);
    return o;
}

void add_noise(ImageF &s, float sigma, uint32_t seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> n(0.f, sigma);
    for (auto &v : s.px) v += n(rng);
}

iris_template *make_template(const iris_engine *e, const ImageF &sheet, const ImageU8 *valid = nullptr) {
    ImageU8 ok = valid ? *valid : ImageU8(sheet.w, sheet.h, 1);
    auto *t = new iris_template;
    t->layout = e->layout;
    Code c;
    encode(*e, sheet, ok, c);
    t->codes.push_back(std::move(c));
    return t;
}

void test_api_basics() {
    CHECK(iris_engine_api_version() == IRIS_ENGINE_API_VERSION);
    iris_config c;
    iris_config_default(&c);
    iris_engine *e = nullptr;
    CHECK(iris_engine_create(&c, &e) == IRIS_OK && e);
    iris_config back;
    iris_engine_get_config(e, &back);
    CHECK(std::memcmp(&back, &c, sizeof(c)) == 0);
    iris_engine_destroy(e);
    CHECK(iris_engine_create(nullptr, &e) == IRIS_OK && e);
    iris_engine_destroy(e);
    iris_engine_destroy(nullptr);
    iris_template_destroy(nullptr);

    iris_config bad = c;
    bad.norm_cols = 500;  // not a power of two
    CHECK(iris_engine_create(&bad, &e) == IRIS_E_INVALID_ARGUMENT && !e);
    bad = c;
    bad.code_rows = 7;  // does not divide norm_rows
    CHECK(iris_engine_create(&bad, &e) == IRIS_E_INVALID_ARGUMENT);
    bad = c;
    bad.struct_size = 4;
    CHECK(iris_engine_create(&bad, &e) == IRIS_E_INVALID_ARGUMENT);
    bad = c;
    bad.max_shift = c.code_cols;
    CHECK(iris_engine_create(&bad, &e) == IRIS_E_INVALID_ARGUMENT);
    CHECK(iris_engine_create(&c, nullptr) == IRIS_E_INVALID_ARGUMENT);
    CHECK(std::strcmp(iris_status_string(IRIS_Q_BLURRED), "blurred") == 0);
}

void test_fft() {
    Fft f;
    f.init(64);
    std::vector<std::complex<float>> x(64), y;
    std::mt19937 rng(3);
    std::normal_distribution<float> n;
    for (auto &v : x) v = {n(rng), n(rng)};
    y = x;
    f.run(y.data(), false);
    double err = 0;
    for (int k = 0; k < 64; ++k) {
        std::complex<double> s = 0;
        for (int j = 0; j < 64; ++j) s += std::complex<double>(x[size_t(j)]) * std::polar(1.0, -2.0 * M_PI * j * k / 64.0);
        err = std::max(err, std::abs(s - std::complex<double>(y[size_t(k)])));
    }
    CHECK(err < 1e-3);
    f.run(y.data(), true);
    double back = 0;
    for (int j = 0; j < 64; ++j) back = std::max(back, double(std::abs(y[size_t(j)] - x[size_t(j)])));
    CHECK(back < 1e-4);
}

void test_extract_contracts(const iris_engine *e) {
    iris_template *t = reinterpret_cast<iris_template *>(1);
    iris_quality q;
    q.struct_size = sizeof(q);
    std::vector<uint8_t> img(size_t(320) * 240, 128);
    CHECK(iris_extract(e, nullptr, 320, 240, 0, &t, &q) == IRIS_E_INVALID_ARGUMENT && t == nullptr);
    CHECK(iris_extract(e, img.data(), 320, 240, 100, &t, &q) == IRIS_E_INVALID_ARGUMENT);  // stride < width
    CHECK(iris_extract(e, img.data(), 40, 40, 0, &t, &q) == IRIS_Q_IMAGE_TOO_SMALL);
    std::vector<uint8_t> black(size_t(320) * 240, 3);
    CHECK(iris_extract(e, black.data(), 320, 240, 0, &t, &q) == IRIS_Q_UNDEREXPOSED && q.status == IRIS_Q_UNDEREXPOSED);
    CHECK(iris_extract(e, img.data(), 320, 240, 0, &t, &q) == IRIS_Q_NO_EYE && t == nullptr);
    CHECK(q.eye_found == 0 && q.overall == 0.f);
    iris_quality badq;
    badq.struct_size = 3;
    CHECK(iris_extract(e, img.data(), 320, 240, 0, &t, &badq) == IRIS_E_INVALID_ARGUMENT);
    CHECK(iris_extract(e, img.data(), 320, 240, 0, &t, nullptr) == IRIS_Q_NO_EYE);
}

void test_input_formats() {
    // The same 10-bit content as GRAY16 and RAW10 MIPI must convert identically.
    const int w = 37, h = 11;  // width not a multiple of 4
    std::vector<uint16_t> v(size_t(w) * h);
    for (size_t i = 0; i < v.size(); ++i) v[i] = uint16_t((i * 37u + 11u) % 1024u);
    const int stride10 = (w + 3) / 4 * 5 + 3;  // with row padding
    std::vector<uint8_t> raw(size_t(stride10) * h, 0);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            uint8_t *g = raw.data() + size_t(y) * stride10 + (x / 4) * 5;
            uint16_t p = v[size_t(y) * w + x];
            g[x % 4] = uint8_t(p >> 2);
            g[4] = uint8_t(g[4] | ((p & 3) << (2 * (x % 4))));
        }
    iris_image a{};
    a.data = v.data(), a.width = w, a.height = h, a.format = IRIS_PIX_GRAY16, a.bits = 10;
    iris_image b{};
    b.data = raw.data(), b.width = w, b.height = h, b.stride = stride10, b.format = IRIS_PIX_RAW10_MIPI;
    Gray8 ga, gb;
    CHECK(convert_input(a, ga) && convert_input(b, gb));
    CHECK(ga.img.px == gb.img.px);
    CHECK(std::fabs(ga.exposure - gb.exposure) < 1e-6f);
    // ROI
    a.roi_x = 5, a.roi_y = 2, a.roi_w = 10, a.roi_h = 4;
    Gray8 gr;
    CHECK(convert_input(a, gr) && gr.img.w == 10 && gr.img.h == 4 && gr.off_x == 5 && gr.off_y == 2);
    // Dark 10-bit input is amplified, not truncated to a few levels.
    std::vector<uint16_t> dark(size_t(64) * 64);
    for (size_t i = 0; i < dark.size(); ++i) dark[i] = uint16_t(i % 80);
    iris_image d{};
    d.data = dark.data(), d.width = 64, d.height = 64, d.format = IRIS_PIX_GRAY16, d.bits = 10;
    Gray8 gd;
    CHECK(convert_input(d, gd));
    CHECK(*std::max_element(gd.img.px.begin(), gd.img.px.end()) > 200);
}

// Encoder + matcher on rubber sheets. Returns the score of the same sheet rotated and noisy.
void test_encode_match(const iris_engine *e) {
    const int R = e->cfg.norm_rows, C = e->cfg.norm_cols;
    const int step = C / e->cfg.code_cols;
    ImageF s = random_sheet(R, C, 1);
    iris_template *a = make_template(e, s);
    // Identical
    iris_match_result r;
    CHECK(iris_match_ex(e, a, a, &r) == IRIS_OK && r.raw_hd == 0.f && r.shift == 0);
    // Rotation by 6 code columns is recovered exactly.
    iris_template *b = make_template(e, shift_sheet(s, 6 * step));
    CHECK(iris_match_ex(e, a, b, &r) == IRIS_OK);
    CHECK(r.raw_hd < 0.02f && std::abs(r.shift) == 6);
    // Rotation beyond max_shift is not recovered.
    iris_template *far_rot = make_template(e, shift_sheet(s, (e->cfg.max_shift + 20) * step));
    CHECK(iris_match_ex(e, a, far_rot, &r) == IRIS_OK && r.score > 0.35f);
    iris_template_destroy(b);
    iris_template_destroy(far_rot);
    // Masked half: compares only the commonly valid part.
    ImageU8 half(C, R, 1);
    for (int i = 0; i < R; ++i)
        for (int j = 0; j < C / 2; ++j) half.at(j, i) = 0;
    iris_template *h = make_template(e, s, &half);
    CHECK(iris_match_ex(e, a, h, &r) == IRIS_OK && r.raw_hd < 0.1f);
    CHECK(r.bits_compared < a->codes[0].valid_bits * 6 / 10);
    // No overlap at all.
    ImageU8 none(C, R, 0);
    iris_template *n = make_template(e, s, &none);
    float sc = 0;
    CHECK(n->codes[0].valid_bits == 0);
    CHECK(iris_match(e, a, n, &sc) == IRIS_E_NOT_ENOUGH_OVERLAP && sc == 1.f);
    iris_template_destroy(h);
    iris_template_destroy(n);
    iris_template_destroy(a);
}

struct Dist {
    std::vector<float> v;
    float mean() const {
        double s = 0;
        for (float x : v) s += x;
        return v.empty() ? 0.f : float(s / double(v.size()));
    }
    float sd() const {
        double m = mean(), s = 0;
        for (float x : v) s += (x - m) * (x - m);
        return v.size() < 2 ? 0.f : float(std::sqrt(s / double(v.size() - 1)));
    }
    float minv() const { return *std::min_element(v.begin(), v.end()); }
    float maxv() const { return *std::max_element(v.begin(), v.end()); }
};

// Statistical sanity check of codes on random sheets: rotated + noisy copies vs other sheets,
// for increasing noise (signal std on the sheet is about 35 grey levels).
void test_sheet_statistics(const iris_engine *e) {
    const int R = e->cfg.norm_rows, C = e->cfg.norm_cols, step = C / e->cfg.code_cols;
    const int subjects = 30;
    std::printf("  sheet test (30 sheets, rotation -7..+7 code cols):\n");
    std::printf("  | noise sd | genuine raw HD mean/max | genuine score max | impostor score mean/sd/min |\n");
    for (float noise : {8.f, 20.f, 35.f, 50.f}) {
        std::vector<iris_template *> ref, probe;
        for (int s = 0; s < subjects; ++s) {
            ImageF base = random_sheet(R, C, uint32_t(100 + s));
            ImageF p = shift_sheet(base, ((s % 15) - 7) * step);
            add_noise(p, noise, uint32_t(1000 + s));
            ImageF b2 = base;
            add_noise(b2, noise, uint32_t(2000 + s));
            ref.push_back(make_template(e, b2));
            probe.push_back(make_template(e, p));
        }
        Dist gen, genraw, imp;
        for (int i = 0; i < subjects; ++i)
            for (int j = 0; j < subjects; ++j) {
                iris_match_result r;
                if (iris_match_ex(e, probe[size_t(i)], ref[size_t(j)], &r) != IRIS_OK) continue;
                if (i == j) gen.v.push_back(r.score), genraw.v.push_back(r.raw_hd);
                else imp.v.push_back(r.score);
            }
        std::printf("  | %4.0f | %.3f / %.3f | %.3f | %.3f / %.3f / %.3f |\n", double(noise), double(genraw.mean()),
                    double(genraw.maxv()), double(gen.maxv()), double(imp.mean()), double(imp.sd()), double(imp.minv()));
        if (noise == 8.f) {
            CHECK(gen.maxv() < 0.3f);
            CHECK(imp.mean() > 0.4f && imp.minv() > gen.maxv());
        }
        for (auto *t : ref) iris_template_destroy(t);
        for (auto *t : probe) iris_template_destroy(t);
    }
}

// Geometry phantom: dark disk (r_p) inside a uniform ring (r_i) on a bright background, with a
// small saturated dot in the disk. Checks candidate search, hole filling and the circle fits.
std::vector<uint8_t> phantom(int w, int h, float cx, float cy, float rp, float ri, bool dot) {
    std::vector<uint8_t> img(static_cast<size_t>(w) * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            float d = std::hypot(float(x) - cx, float(y) - cy);
            // 1-px linear ramps for sub-pixel edges
            float v = 170.f;
            float ring = std::min(1.f, std::max(0.f, ri + 0.5f - d));
            v = v * (1 - ring) + 95.f * ring;
            float pup = std::min(1.f, std::max(0.f, rp + 0.5f - d));
            v = v * (1 - pup) + 20.f * pup;
            if (dot && std::hypot(float(x) - cx - 0.3f * rp, float(y) - cy + 0.3f * rp) < 0.15f * rp) v = 255.f;
            img[size_t(y) * w + x] = uint8_t(v + 0.5f);
        }
    return img;
}

void test_phantom(const iris_engine *e) {
    struct Case { float cx, cy, rp, ri; };
    const Case cases[] = {{320, 240, 20, 50}, {300, 200, 30, 75}, {330, 250, 45, 100}, {310, 260, 18, 60}};
    for (const Case &cs : cases) {
        auto img = phantom(640, 480, cs.cx, cs.cy, cs.rp, cs.ri, true);
        iris_image im{};
        im.data = img.data(), im.width = 640, im.height = 480, im.format = IRIS_PIX_GRAY8;
        iris_template *t = nullptr;
        iris_quality q{};
        q.struct_size = sizeof(q);
        ExtractDebug d;
        iris_status st = citymaniris::extract(*e, im, &t, &q, &d);
        const float pe = std::hypot(q.pupil_x - cs.cx, q.pupil_y - cs.cy), ie = std::hypot(q.iris_x - cs.cx, q.iris_y - cs.cy);
        std::printf("  phantom rp %.0f ri %.0f: %s, pupil err %.2f px r %.2f, iris err %.2f px r %.2f, glint %d, blur %.2f, lids %.2f, usable %.2f\n",
                    double(cs.rp), double(cs.ri), iris_status_string(st), double(pe), double(q.pupil_r), double(ie),
                    double(q.iris_r), q.glint_found, double(q.blur_sigma), double(q.eyelid_coverage),
                    double(q.usable_fraction));
        CHECK(q.eye_found == 1);
        CHECK(pe < 1.0f && std::fabs(q.pupil_r - cs.rp) < 1.0f);
        CHECK(ie < 1.5f && std::fabs(q.iris_r - cs.ri) < 1.5f);
        CHECK(q.glint_found == 1);
        CHECK(st == IRIS_Q_LOW_TEXTURE);  // flat ring: geometry fine, nothing to encode
        iris_template_destroy(t);
    }
    // Timing of the full extract (segmentation to the texture gate) on phantoms.
    for (int full = 0; full < 2; ++full) {
        const int w = full ? 1920 : 640, h = full ? 1088 : 480;
        auto img = phantom(w, h, float(w) * 0.55f, float(h) * 0.45f, 26, 65, true);
        double best = 1e9;
        for (int k = 0; k < 7; ++k) {
            iris_template *t = nullptr;
            double a = now_ms();
            iris_extract(e, img.data(), w, h, 0, &t, nullptr);
            best = std::min(best, now_ms() - a);
            iris_template_destroy(t);
        }
        std::printf("  timing: extract on %dx%d phantom (iris r 65): %.1f ms\n", w, h, best);
    }
    // Iris smaller than min_iris_radius (40).
    {
        auto img = phantom(640, 480, 320, 240, 10, 28, false);
        iris_template *t = nullptr;
        iris_status st = iris_extract(e, img.data(), 640, 480, 0, &t, nullptr);
        CHECK(st == IRIS_Q_IRIS_TOO_SMALL || st == IRIS_Q_NO_IRIS);
        CHECK(t == nullptr);
    }
    // Iris crossing the image border.
    {
        auto img = phantom(640, 480, 30, 240, 22, 60, true);
        iris_template *t = nullptr;
        iris_status st = iris_extract(e, img.data(), 640, 480, 0, &t, nullptr);
        std::printf("  phantom at the border: %s\n", iris_status_string(st));
        CHECK(st == IRIS_Q_OUT_OF_FRAME);
    }
    // Blur: the pupil edge estimate follows the applied Gaussian blur.
    for (float sigma : {1.5f, 3.f}) {
        auto img = phantom(640, 480, 320, 240, 30, 80, false);
        ImageF f(640, 480), g;
        for (size_t i = 0; i < img.size(); ++i) f.px[i] = img[i];
        gaussian_blur(f, g, sigma);
        for (size_t i = 0; i < img.size(); ++i) img[i] = uint8_t(g.px[i] + 0.5f);
        iris_template *t = nullptr;
        iris_quality q{};
        q.struct_size = sizeof(q);
        iris_status st = iris_extract(e, img.data(), 640, 480, 0, &t, &q);
        std::printf("  phantom blur sigma %.1f: estimated %.2f px, status %s\n", double(sigma), double(q.blur_sigma),
                    iris_status_string(st));
        CHECK(std::fabs(q.blur_sigma - sigma) < 0.35f * sigma + 0.3f);
        iris_template_destroy(t);
    }
}

void test_serialization(const iris_engine *e) {
    ImageF s = random_sheet(e->cfg.norm_rows, e->cfg.norm_cols, 7);
    iris_template *a = make_template(e, s);
    a->codes[0].quality = 77.5f;
    size_t need = 0;
    CHECK(iris_template_serialize(a, nullptr, 0, &need) == IRIS_OK && need > 32);
    std::vector<uint8_t> buf(need);
    size_t got = 0;
    CHECK(iris_template_serialize(a, buf.data(), 10, &got) == IRIS_E_BUFFER_TOO_SMALL && got == need);
    CHECK(iris_template_serialize(a, buf.data(), buf.size(), &got) == IRIS_OK);
    iris_template *b = nullptr;
    CHECK(iris_template_deserialize(buf.data(), buf.size(), &b) == IRIS_OK && b);
    CHECK(b->codes.size() == 1 && b->codes[0].bits == a->codes[0].bits && b->codes[0].mask == a->codes[0].mask);
    CHECK(b->codes[0].quality == 77.5f && b->layout == a->layout);
    float sc;
    CHECK(iris_match(e, a, b, &sc) == IRIS_OK && sc < 0.01f);
    iris_template_destroy(b);
    std::vector<uint8_t> bad = buf;
    bad[100] ^= 0x10;
    CHECK(iris_template_deserialize(bad.data(), bad.size(), &b) == IRIS_E_BAD_TEMPLATE && !b);
    CHECK(iris_template_deserialize(buf.data(), buf.size() - 1, &b) == IRIS_E_BAD_TEMPLATE);
    bad = buf;
    bad[4] = 9;
    CHECK(iris_template_deserialize(bad.data(), bad.size(), &b) == IRIS_E_TEMPLATE_VERSION);
    bad = buf;
    bad[0] = 'X';
    CHECK(iris_template_deserialize(bad.data(), bad.size(), &b) == IRIS_E_BAD_TEMPLATE);
    // Templates from a different layout are refused by the matcher.
    iris_config c;
    iris_config_default(&c);
    c.wavelength[0] = 24.f;
    iris_engine *e2 = nullptr;
    CHECK(iris_engine_create(&c, &e2) == IRIS_OK);
    CHECK(iris_match(e2, a, a, &sc) == IRIS_E_INCOMPATIBLE);
    iris_engine_destroy(e2);
    iris_template *cl = iris_template_clone(a);
    CHECK(cl && iris_template_code_count(cl) == 1);
    iris_template_destroy(cl);
    iris_template_destroy(a);
}

void test_enroll(const iris_engine *e) {
    const int R = e->cfg.norm_rows, C = e->cfg.norm_cols, step = C / e->cfg.code_cols;
    ImageF base = random_sheet(R, C, 42);
    std::vector<iris_template *> samples;
    for (int k = 0; k < 4; ++k) {
        ImageF p = shift_sheet(base, (k - 2) * step);
        add_noise(p, 6.f, uint32_t(500 + k));
        samples.push_back(make_template(e, p));
    }
    samples.push_back(make_template(e, random_sheet(R, C, 43)));  // wrong subject
    iris_template *t = nullptr;
    iris_enroll_info info;
    CHECK(iris_enroll(e, samples.data(), samples.size(), &t, &info) == IRIS_OK && t);
    CHECK(info.samples_in == 5 && info.samples_used == 4 && info.codes_stored == 4);
    CHECK(info.max_intra_score <= e->cfg.enroll_consistency);
    ImageF probe = shift_sheet(base, 3 * step);
    add_noise(probe, 8.f, 999);
    iris_template *pt = make_template(e, probe), *other = make_template(e, random_sheet(R, C, 44));
    int32_t acc = 0;
    float sc = 0;
    CHECK(iris_verify(e, pt, t, &acc, &sc) == IRIS_OK && acc == 1);
    CHECK(iris_verify(e, other, t, &acc, &sc) == IRIS_OK && acc == 0);
    // Too few consistent samples.
    iris_template *t2 = nullptr;
    std::vector<iris_template *> two = {samples[0], samples[4]};
    CHECK(iris_enroll(e, two.data(), two.size(), &t2, &info) == IRIS_E_ENROLL_INCONSISTENT && !t2);
    iris_template_destroy(pt);
    iris_template_destroy(other);
    iris_template_destroy(t);
    for (auto *s : samples) iris_template_destroy(s);
}

void test_threads(const iris_engine *e) {
    const int R = e->cfg.norm_rows, C = e->cfg.norm_cols;
    std::vector<ImageF> sheets;
    for (int i = 0; i < 8; ++i) sheets.push_back(random_sheet(R, C, uint32_t(300 + i)));
    std::vector<std::vector<uint64_t>> serial(8), par(8);
    for (int i = 0; i < 8; ++i) {
        iris_template *t = make_template(e, sheets[size_t(i)]);
        serial[size_t(i)] = t->codes[0].bits;
        iris_template_destroy(t);
    }
    std::vector<std::thread> th;
    for (int i = 0; i < 8; ++i)
        th.emplace_back([&, i] {
            for (int rep = 0; rep < 5; ++rep) {
                iris_template *t = make_template(e, sheets[size_t(i)]);
                float sc;
                iris_match(e, t, t, &sc);
                par[size_t(i)] = t->codes[0].bits;
                iris_template_destroy(t);
            }
        });
    for (auto &t : th) t.join();
    CHECK(serial == par);
}

void bench(const iris_engine *e) {
    const int R = e->cfg.norm_rows, C = e->cfg.norm_cols;
    ImageF s = random_sheet(R, C, 5);
    const int n = 200;
    double t0 = now_ms();
    iris_template *t = nullptr;
    for (int i = 0; i < n; ++i) {
        iris_template_destroy(t);
        t = make_template(e, s);
    }
    double t_enc = (now_ms() - t0) / n;
    iris_template *u = make_template(e, random_sheet(R, C, 6));
    float sc;
    t0 = now_ms();
    const int m = 20000;
    for (int i = 0; i < m; ++i) iris_match(e, t, u, &sc);
    double t_match = (now_ms() - t0) / m;
    std::printf("  timing (host, 1 thread): encode %.3f ms, match 1x1 code with +/-%d shifts %.4f ms\n", t_enc,
                int(e->cfg.max_shift), t_match);
    iris_template_destroy(t);
    iris_template_destroy(u);
    // Full-frame front end on a 1920x1088 frame without an eye (conversion, downsampling,
    // multi-threshold components): the part of extract() that scales with the frame size.
    std::mt19937 rng(9);
    std::normal_distribution<float> pix(110.f, 25.f);
    std::vector<uint8_t> frame(size_t(1920) * 1088);
    for (auto &v : frame) v = uint8_t(std::min(255.f, std::max(0.f, pix(rng))));
    for (int y = 0; y < 1088; ++y)  // dark background band, typical with a near-field LED
        for (int x = 0; x < 300; ++x) frame[size_t(y) * 1920 + size_t(x)] = 6;
    for (int fmt = 0; fmt < 2; ++fmt) {
        std::vector<uint16_t> f16(frame.size());
        for (size_t i = 0; i < frame.size(); ++i) f16[i] = uint16_t(frame[i] * 4);
        iris_image img{};
        img.width = 1920, img.height = 1088;
        if (fmt == 0) img.data = frame.data(), img.format = IRIS_PIX_GRAY8;
        else img.data = f16.data(), img.format = IRIS_PIX_GRAY16, img.bits = 10;
        iris_template *x = nullptr;
        double best = 1e9;
        for (int k = 0; k < 5; ++k) {
            double a = now_ms();
            iris_extract_image(e, &img, &x, nullptr);
            best = std::min(best, now_ms() - a);
        }
        std::printf("  timing: 1920x1088 %s frame without eye (front end only): %.1f ms\n",
                    fmt ? "GRAY16/10-bit" : "GRAY8", best);
    }
}

// API 2: iris_extract_ex with a context and a search hint gives the template of the full search.
std::vector<uint8_t> serialized(const iris_template *t) {
    size_t n = 0;
    iris_template_serialize(t, nullptr, 0, &n);
    std::vector<uint8_t> b(n);
    if (iris_template_serialize(t, b.data(), b.size(), &n) != IRIS_OK) b.clear();
    return b;
}

void test_context_hint(const iris_engine *e) {
    CHECK(iris_engine_api_version() >= 2);
    iris_context *ctx = nullptr;
    CHECK(iris_context_create(nullptr, &ctx) == IRIS_E_INVALID_ARGUMENT && !ctx);
    CHECK(iris_context_create(e, &ctx) == IRIS_OK && ctx);
    eyeframe::Params p;  // 1920x1080, iris r 85 at (900, 520)
    const auto frame = eyeframe::render(p);
    eyeframe::Params p2 = p;
    p2.cx = 1350.f, p2.cy = 380.f, p2.iris_r = 70.f, p2.pupil_r = 26.f, p2.texture_seed = 7;
    p2.sclera_a = 200.f, p2.sclera_b = 100.f;
    const auto frame2 = eyeframe::render(p2);
    iris_image img{};
    img.data = frame.data(), img.width = p.width, img.height = p.height, img.format = IRIS_PIX_GRAY8;

    iris_template *full = nullptr;
    iris_quality qf{};
    qf.struct_size = sizeof(qf);
    CHECK(iris_extract_ex(e, nullptr, &img, nullptr, &full, &qf) == IRIS_OK && full);
    CHECK(std::hypot(qf.iris_x - p.cx, qf.iris_y - p.cy) < 1.5f && std::fabs(qf.iris_r - p.iris_r) < 1.5f);
    // Same as the API 1 entry point, byte for byte.
    iris_template *v1 = nullptr;
    CHECK(iris_extract_image(e, &img, &v1, nullptr) == IRIS_OK && v1);
    CHECK(serialized(full) == serialized(v1));

    // Hint near the eye (as from the previous frame, a few pixels off).
    const iris_search_hint near_hint{qf.iris_x + 9.f, qf.iris_y - 6.f, qf.iris_r};
    iris_template *hinted = nullptr;
    iris_quality qh{};
    qh.struct_size = sizeof(qh);
    CHECK(iris_extract_ex(e, ctx, &img, &near_hint, &hinted, &qh) == IRIS_OK && hinted);
    float score = 1.f;
    CHECK(hinted && iris_match(e, full, hinted, &score) == IRIS_OK);
    std::printf("  hint near the eye: score vs full search %.4f, iris (%.2f, %.2f) vs (%.2f, %.2f)\n", double(score),
                double(qh.iris_x), double(qh.iris_y), double(qf.iris_x), double(qf.iris_y));
    CHECK(score <= 0.05f);
    CHECK(std::hypot(qh.iris_x - qf.iris_x, qh.iris_y - qf.iris_y) < 0.5f);

    // Hint far from the eye: falls back to the whole frame.
    const iris_search_hint far_hint{1700.f, 900.f, 80.f};
    iris_template *fallback = nullptr;
    CHECK(iris_extract_ex(e, ctx, &img, &far_hint, &fallback, nullptr) == IRIS_OK && fallback);
    CHECK(fallback && iris_match(e, full, fallback, &score) == IRIS_OK && score <= 0.05f);

    // A reused context gives the same bytes as a fresh one, also after another frame.
    img.data = frame2.data();
    iris_template *other = nullptr, *other_fresh = nullptr;
    CHECK(iris_extract_ex(e, ctx, &img, nullptr, &other, nullptr) == IRIS_OK);
    CHECK(iris_extract_ex(e, nullptr, &img, nullptr, &other_fresh, nullptr) == IRIS_OK);
    CHECK(other && other_fresh && serialized(other) == serialized(other_fresh));
    img.data = frame.data();
    iris_template *again = nullptr;
    CHECK(iris_extract_ex(e, ctx, &img, nullptr, &again, nullptr) == IRIS_OK);
    CHECK(again && serialized(again) == serialized(full));

    // Micro-benchmark on the 1920x1080 frame (host, 1 thread).
    auto best_of = [&](const iris_search_hint *h, iris_context *c, const std::vector<uint8_t> &data) {
        img.data = data.data();
        double best = 1e9;
        for (int k = 0; k < 9; ++k) {
            iris_template *t = nullptr;
            double a = now_ms();
            iris_extract_ex(e, c, &img, h, &t, nullptr);
            best = std::min(best, now_ms() - a);
            iris_template_destroy(t);
        }
        return best;
    };
    std::vector<uint8_t> empty(frame.size());
    for (size_t i = 0; i < empty.size(); ++i) empty[i] = uint8_t(90 + (i * 2654435761u >> 28));
    std::printf("  timing 1920x1080 eye frame: no context %.1f ms, context %.1f ms, context + hint %.1f ms; "
                "no eye %.1f ms\n",
                best_of(nullptr, nullptr, frame), best_of(nullptr, ctx, frame), best_of(&near_hint, ctx, frame),
                best_of(nullptr, ctx, empty));

    for (iris_template *t : {full, v1, hinted, fallback, other, other_fresh, again}) iris_template_destroy(t);
    iris_context_destroy(ctx);
    iris_context_destroy(nullptr);
}

}  // namespace

int main() {
    iris_engine *e = nullptr;
    if (iris_engine_create(nullptr, &e) != IRIS_OK) return 2;
    test_api_basics();
    test_fft();
    test_extract_contracts(e);
    test_input_formats();
    test_phantom(e);
    test_encode_match(e);
    test_sheet_statistics(e);
    test_serialization(e);
    test_enroll(e);
    test_threads(e);
    test_context_hint(e);
    bench(e);
    iris_engine_destroy(e);
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
