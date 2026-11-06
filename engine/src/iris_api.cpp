// C API: configuration, extraction pipeline, matching, enrollment and template serialization.
#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>

#include "iris_internal.h"

#if defined(__cpp_exceptions) || defined(__EXCEPTIONS)
#define IRIS_TRY try
#define IRIS_CATCH_ALL                                \
    catch (const std::bad_alloc &) {                  \
        return IRIS_E_NO_MEMORY;                      \
    }                                                 \
    catch (...) {                                     \
        return IRIS_E_INTERNAL;                       \
    }
#else
#define IRIS_TRY if (true)
#define IRIS_CATCH_ALL else {}
#endif

using namespace citymaniris;

namespace {

bool is_pow2(unsigned v) { return v && !(v & (v - 1)); }

bool config_valid(const iris_config &c) {
    if (c.struct_size != sizeof(iris_config)) return false;
    if (!(c.min_iris_radius >= 8.f) || !(c.max_iris_radius > c.min_iris_radius)) return false;
    if (c.norm_rows < 4 || c.norm_cols < 32 || !is_pow2(c.norm_cols) || c.norm_cols > 4096) return false;
    if (c.code_rows < 1 || c.code_cols < 8 || c.norm_rows % c.code_rows || c.norm_cols % c.code_cols) return false;
    if (c.n_filters < 1 || c.n_filters > 2) return false;
    if (unsigned(c.code_rows) * c.n_filters * 2u > 64u * 255u) return false;
    for (int i = 0; i < c.n_filters; ++i)
        if (!(c.wavelength[i] >= 3.f && c.wavelength[i] <= float(c.norm_cols))) return false;
    if (!(c.sigma_on_f > 0.1f && c.sigma_on_f < 0.95f)) return false;
    if (!(c.radial_sigma >= 0.f) || !(c.fragile_fraction >= 0.f && c.fragile_fraction < 0.9f)) return false;
    if (c.max_shift >= c.code_cols / 2) return false;
    if (!(c.min_overlap_fraction >= 0.f && c.min_overlap_fraction < 1.f)) return false;
    if (std::isnan(c.reference_bits) || !(c.match_threshold >= 0.f && c.match_threshold <= 1.f)) return false;
    if (c.enroll_min_samples < 1 || c.enroll_max_codes < 1 || c.enroll_max_codes > 64) return false;
    return true;
}

void fill_quality_geometry(iris_quality *q, const Segmentation &s, int ox, int oy) {
    q->pupil_x = s.pupil.cx + float(ox);
    q->pupil_y = s.pupil.cy + float(oy);
    q->pupil_r = s.pupil.a0;
    q->iris_x = s.iris.cx + float(ox);
    q->iris_y = s.iris.cy + float(oy);
    q->iris_r = s.iris.a0;
    q->pupil_iris_ratio = s.iris.a0 > 0 ? s.pupil.a0 / s.iris.a0 : 0.f;
    q->limbus_contrast = s.limbus_contrast;
    q->pupil_contrast = s.pupil_contrast;
    q->glint_found = s.glint ? 1 : 0;
    q->eye_found = 1;
    q->blur_sigma = s.blur_sigma;
}

// CRC-32 (IEEE 802.3, reflected), bitwise: templates are small.
uint32_t crc32(const uint8_t *p, size_t n) {
    uint32_t c = 0xffffffffu;
    for (size_t i = 0; i < n; ++i) {
        c ^= p[i];
        for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1u)));
    }
    return ~c;
}

void put16(uint8_t *p, uint16_t v) { p[0] = uint8_t(v), p[1] = uint8_t(v >> 8); }
void put32(uint8_t *p, uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i));
}
void put64(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; ++i) p[i] = uint8_t(v >> (8 * i));
}
uint16_t get16(const uint8_t *p) { return uint16_t(p[0] | (p[1] << 8)); }
uint32_t get32(const uint8_t *p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }
uint64_t get64(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= uint64_t(p[i]) << (8 * i);
    return v;
}

constexpr uint8_t kMagic[4] = {'C', 'I', 'R', 'T'};
constexpr size_t kHeader = 32;

size_t serialized_size(const iris_template &t) {
    const size_t words = size_t(t.layout.cols) * t.layout.words;
    return kHeader + t.codes.size() * (8 + 16 * words) + 4;
}

}  // namespace

namespace citymaniris {

iris_status extract(const iris_engine &eng, const iris_image &img, iris_template **out, iris_quality *q,
                    ExtractDebug *dbg) {
    *out = nullptr;
    const iris_config &cfg = eng.cfg;
    Gray8 g;
    if (!convert_input(img, g)) return IRIS_E_INVALID_ARGUMENT;
    if (q) q->exposure = g.exposure;
    if (dbg) dbg->exposure = g.exposure, dbg->roi_x = g.off_x, dbg->roi_y = g.off_y;
    if (g.img.w < int(2.f * cfg.min_iris_radius) + 4 || g.img.h < int(2.f * cfg.min_iris_radius) + 4)
        return IRIS_Q_IMAGE_TOO_SMALL;
    if (g.exposure < 0.04f) return IRIS_Q_UNDEREXPOSED;

    Segmentation seg;
    int cx = 0, cy = 0;
    ImageF crop;
    ImageU8 nmask;
    iris_status st = IRIS_OK;
    bool found = segment(eng, g, seg, cx, cy, crop, nmask, st, dbg);
    if (dbg) {
        dbg->seg = seg;
        dbg->crop_x = cx, dbg->crop_y = cy;
    }
    if (!found) return st;
    if (q) fill_quality_geometry(q, seg, cx + g.off_x, cy + g.off_y);
    if (st != IRIS_OK) return st;
    if (seg.iris.a0 < cfg.min_iris_radius) return IRIS_Q_IRIS_TOO_SMALL;

    // Iris circle outside the image (crop is clamped to the image/ROI).
    {
        int outside = 0;
        for (int i = 0; i < 64; ++i) {
            float t = float(i) * 6.2831853f / 64.f;
            float x = seg.iris.cx + seg.iris.a0 * std::cos(t), y = seg.iris.cy + seg.iris.a0 * std::sin(t);
            int gx = int(x) + cx, gy = int(y) + cy;
            if (gx < 0 || gy < 0 || gx >= g.img.w || gy >= g.img.h) outside++;
        }
        if (outside > 16) return IRIS_Q_OUT_OF_FRAME;
    }
    // Focus: blur relative to the shortest encoded wavelength at mid-iris.
    float lambda_px = 1e9f;
    for (int i = 0; i < cfg.n_filters; ++i) {
        const float rmid = 0.5f * (seg.pupil.a0 + seg.iris.a0);
        lambda_px = std::min(lambda_px, 6.2831853f * rmid * cfg.wavelength[i] / float(cfg.norm_cols));
    }
    const float blur = seg.blur_sigma < 0.f ? 0.f : seg.blur_sigma;
    const float ratio = blur / lambda_px;
    const float focus = 100.f / (1.f + (ratio / cfg.max_blur_ratio) * (ratio / cfg.max_blur_ratio));
    if (q) q->focus = focus;
    if (ratio > cfg.max_blur_ratio) return IRIS_Q_BLURRED;

    ImageF norm;
    ImageU8 valid;
    normalize_iris(eng, seg, crop, nmask, norm, valid);
    // eyelid coverage on the rubber sheet (geometry only)
    {
        int lid = 0;
        const int R = cfg.norm_rows, C = cfg.norm_cols;
        for (int j = 0; j < C; j += 2) {
            const float t = 6.2831853f * float(j) / float(C), ct = std::cos(t), st2 = std::sin(t);
            const float rp = seg.pupil.radius(t);
            for (int i = 0; i < R; i += 2) {
                const float rho = (float(i) + 0.5f) / float(R);
                const float px = seg.pupil.cx + rp * ct, py = seg.pupil.cy + rp * st2;
                const float x = px + rho * (seg.iris.cx + seg.iris.a0 * ct - px);
                const float y = py + rho * (seg.iris.cy + seg.iris.a0 * st2 - py);
                if ((seg.upper.present && y < seg.upper.y(x)) || (seg.lower.present && y > seg.lower.y(x))) lid++;
            }
        }
        if (q) q->eyelid_coverage = float(lid) / float((R / 2) * (C / 2));
    }
    Code code;
    encode(eng, norm, valid, code);
    if (dbg) dbg->norm = norm, dbg->norm_valid = valid;
    const float usable = float(code.valid_bits) / float(eng.layout.bits_total());
    // The fragile-bit mask removes a fixed share of the bits; compare what is left with the
    // geometric validity to tell occlusion from missing texture.
    const float usable_geo = usable / std::max(0.1f, 1.f - cfg.fragile_fraction);
    size_t nvalid_px = 0;
    for (uint8_t v : valid.px) nvalid_px += v;
    const float geo = float(nvalid_px) / float(std::max<size_t>(1, valid.px.size()));
    if (q) q->usable_fraction = usable;
    if (geo < cfg.min_usable_fraction) return IRIS_Q_OCCLUDED;
    if (usable_geo < cfg.min_usable_fraction) return IRIS_Q_LOW_TEXTURE;

    const float size_term = std::min(1.f, seg.iris.a0 / (1.5f * cfg.min_iris_radius));
    code.quality = focus * std::min(1.f, usable_geo / 0.8f) * (0.5f + 0.5f * size_term);
    if (q) q->overall = code.quality;
    iris_template *t = new iris_template;
    t->layout = eng.layout;
    t->flags = cfg.fragile_fraction > 0.f ? 1u : 0u;
    t->codes.push_back(std::move(code));
    *out = t;
    return IRIS_OK;
}

}  // namespace citymaniris

extern "C" {

int iris_engine_api_version(void) { return IRIS_ENGINE_API_VERSION; }

const char *iris_status_string(iris_status s) {
    switch (s) {
        case IRIS_OK: return "ok";
        case IRIS_E_INVALID_ARGUMENT: return "invalid argument";
        case IRIS_E_NO_MEMORY: return "out of memory";
        case IRIS_E_BUFFER_TOO_SMALL: return "buffer too small";
        case IRIS_E_BAD_TEMPLATE: return "corrupt template";
        case IRIS_E_TEMPLATE_VERSION: return "unsupported template version";
        case IRIS_E_INCOMPATIBLE: return "incompatible code layout";
        case IRIS_E_INTERNAL: return "internal error";
        case IRIS_Q_IMAGE_TOO_SMALL: return "image too small";
        case IRIS_Q_UNDEREXPOSED: return "underexposed";
        case IRIS_Q_OVEREXPOSED: return "overexposed";
        case IRIS_Q_NO_EYE: return "no eye found";
        case IRIS_Q_NO_IRIS: return "iris boundary not found";
        case IRIS_Q_BAD_GEOMETRY: return "implausible eye geometry";
        case IRIS_Q_IRIS_TOO_SMALL: return "iris too small (move closer)";
        case IRIS_Q_OUT_OF_FRAME: return "iris out of frame";
        case IRIS_Q_BLURRED: return "blurred";
        case IRIS_Q_OCCLUDED: return "iris occluded";
        case IRIS_Q_LOW_TEXTURE: return "no iris texture";
        case IRIS_E_NOT_ENOUGH_OVERLAP: return "not enough overlapping bits";
        case IRIS_E_ENROLL_INCONSISTENT: return "inconsistent enrollment samples";
    }
    return "unknown";
}

void iris_config_default(iris_config *c) {
    if (!c) return;
    std::memset(c, 0, sizeof(*c));
    c->struct_size = sizeof(iris_config);
    c->min_iris_radius = 40.f;
    c->max_iris_radius = 130.f;
    c->norm_rows = 64;
    c->norm_cols = 512;
    c->code_rows = 16;
    c->code_cols = 256;
    c->n_filters = 2;
    c->wavelength[0] = 32.f;
    c->wavelength[1] = 64.f;
    c->sigma_on_f = 0.5f;
    c->radial_sigma = 2.f;
    c->fragile_fraction = 0.15f;
    c->max_blur_ratio = 0.30f;
    c->min_usable_fraction = 0.35f;
    c->min_limbus_contrast = 3.f;
    c->max_shift = 10;
    c->match_threshold = 0.36f;
    c->reference_bits = 0.f;
    c->min_overlap_fraction = 0.15f;
    c->enroll_min_samples = 3;
    c->enroll_max_codes = 5;
    c->enroll_consistency = 0.32f;
}

iris_status iris_engine_create(const iris_config *config, iris_engine **engine_out) {
    if (!engine_out) return IRIS_E_INVALID_ARGUMENT;
    *engine_out = nullptr;
    iris_config c;
    if (config) c = *config;
    else iris_config_default(&c);
    if (!config_valid(c)) return IRIS_E_INVALID_ARGUMENT;
    IRIS_TRY {
        iris_engine *e = new iris_engine;
        e->cfg = c;
        e->layout.rows = c.code_rows;
        e->layout.cols = c.code_cols;
        e->layout.n_filters = uint8_t(c.n_filters);
        e->layout.words = uint8_t((unsigned(c.code_rows) * c.n_filters * 2u + 63u) / 64u);
        e->layout.encoder_id = compute_encoder_id(c);
        e->fft.init(c.norm_cols);
        const float ls = std::log(c.sigma_on_f);
        for (int fi = 0; fi < c.n_filters; ++fi) {
            e->gabor[fi].assign(c.norm_cols, 0.f);
            const float f0 = 1.f / c.wavelength[fi];
            for (int k = 1; k <= c.norm_cols / 2; ++k) {
                const float fr = float(k) / float(c.norm_cols);
                const float l = std::log(fr / f0);
                e->gabor[fi][size_t(k)] = std::exp(-(l * l) / (2.f * ls * ls));
            }
        }
        *engine_out = e;
        return IRIS_OK;
    }
    IRIS_CATCH_ALL
    return IRIS_E_INTERNAL;
}

void iris_engine_destroy(iris_engine *e) { delete e; }

void iris_engine_get_config(const iris_engine *e, iris_config *out) {
    if (e && out) *out = e->cfg;
}

iris_status iris_extract_image(const iris_engine *e, const iris_image *img, iris_template **out,
                               iris_quality *q) {
    if (out) *out = nullptr;
    if (!e || !img || !out) return IRIS_E_INVALID_ARGUMENT;
    if (q) {
        if (q->struct_size != sizeof(iris_quality)) return IRIS_E_INVALID_ARGUMENT;
        std::memset(q, 0, sizeof(*q));
        q->struct_size = sizeof(iris_quality);
    }
    iris_status st = IRIS_E_INTERNAL;
    IRIS_TRY { st = citymaniris::extract(*e, *img, out, q, nullptr); }
    IRIS_CATCH_ALL
    if (q) {
        q->status = st;
        if (st != IRIS_OK) q->overall = 0.f;
    }
    return st;
}

iris_status iris_extract(const iris_engine *e, const uint8_t *gray, int32_t w, int32_t h, int32_t stride,
                         iris_template **out, iris_quality *q) {
    iris_image img;
    std::memset(&img, 0, sizeof(img));
    img.data = gray;
    img.width = w;
    img.height = h;
    img.stride = stride;
    img.format = IRIS_PIX_GRAY8;
    return iris_extract_image(e, &img, out, q);
}

iris_status iris_match_ex(const iris_engine *e, const iris_template *a, const iris_template *b,
                          iris_match_result *res) {
    if (!e || !a || !b || !res) return IRIS_E_INVALID_ARGUMENT;
    std::memset(res, 0, sizeof(*res));
    res->score = 1.f;
    res->raw_hd = 1.f;
    if (!(a->layout == e->layout) || !(b->layout == e->layout)) return IRIS_E_INCOMPATIBLE;
    PairScore best;
    bool any = false;
    for (size_t i = 0; i < a->codes.size(); ++i)
        for (size_t j = 0; j < b->codes.size(); ++j) {
            PairScore p = compare_codes(*e, a->codes[i], b->codes[j]);
            if (p.bits == 0) continue;
            if (!any || p.score < best.score) {
                best = p;
                any = true;
                res->probe_index = int32_t(i);
                res->gallery_index = int32_t(j);
            }
        }
    if (!any) return IRIS_E_NOT_ENOUGH_OVERLAP;
    res->score = best.score;
    res->raw_hd = best.raw;
    res->bits_compared = best.bits;
    res->shift = best.shift;
    return IRIS_OK;
}

iris_status iris_match(const iris_engine *e, const iris_template *a, const iris_template *b, float *score) {
    if (!score) return IRIS_E_INVALID_ARGUMENT;
    iris_match_result r;
    iris_status st = iris_match_ex(e, a, b, &r);
    *score = r.score;
    return st;
}

iris_status iris_verify(const iris_engine *e, const iris_template *a, const iris_template *b, int32_t *acc,
                        float *score) {
    if (!acc) return IRIS_E_INVALID_ARGUMENT;
    *acc = 0;
    iris_match_result r;
    iris_status st = iris_match_ex(e, a, b, &r);
    if (score) *score = r.score;
    if (st == IRIS_OK) *acc = r.score <= e->cfg.match_threshold ? 1 : 0;
    return st;
}

iris_status iris_enroll(const iris_engine *e, const iris_template *const *samples, size_t n,
                        iris_template **out, iris_enroll_info *info) {
    if (out) *out = nullptr;
    if (!e || !samples || !out || n == 0) return IRIS_E_INVALID_ARGUMENT;
    if (info) std::memset(info, 0, sizeof(*info));
    IRIS_TRY {
        std::vector<const Code *> codes;
        for (size_t i = 0; i < n; ++i) {
            if (!samples[i]) return IRIS_E_INVALID_ARGUMENT;
            if (!(samples[i]->layout == e->layout)) return IRIS_E_INCOMPATIBLE;
            for (const Code &c : samples[i]->codes) codes.push_back(&c);
        }
        const size_t m = codes.size();
        if (info) info->samples_in = uint32_t(m);
        std::vector<float> d(m * m, 0.f);
        for (size_t i = 0; i < m; ++i)
            for (size_t j = i + 1; j < m; ++j) {
                PairScore p = compare_codes(*e, *codes[i], *codes[j]);
                d[i * m + j] = d[j * m + i] = p.bits ? p.score : 1.f;
            }
        size_t med = 0;
        float bestsum = 1e30f;
        for (size_t i = 0; i < m; ++i) {
            float s = 0;
            for (size_t j = 0; j < m; ++j) s += d[i * m + j];
            if (s < bestsum) bestsum = s, med = i;
        }
        std::vector<size_t> used;
        for (size_t i = 0; i < m; ++i)
            if (i == med || d[med * m + i] <= e->cfg.enroll_consistency) used.push_back(i);
        float mean = 0, mx = 0;
        int np = 0;
        for (size_t a = 0; a < used.size(); ++a)
            for (size_t b = a + 1; b < used.size(); ++b) {
                float v = d[used[a] * m + used[b]];
                mean += v, mx = std::max(mx, v), np++;
            }
        if (np) mean /= float(np);
        if (info) {
            info->samples_used = uint32_t(used.size());
            info->mean_intra_score = mean;
            info->max_intra_score = mx;
        }
        if (used.size() < e->cfg.enroll_min_samples) return IRIS_E_ENROLL_INCONSISTENT;
        // Keep the most central codes (lowest mean distance to the consistent set), medoid first.
        std::vector<std::pair<float, size_t>> order;
        for (size_t i : used) {
            float s = 0;
            for (size_t j : used) s += d[i * m + j];
            order.push_back({i == med ? -1.f : s, i});
        }
        std::sort(order.begin(), order.end());
        iris_template *t = new iris_template;
        t->layout = e->layout;
        t->flags = samples[0]->flags;
        for (size_t k = 0; k < order.size() && k < e->cfg.enroll_max_codes; ++k)
            t->codes.push_back(*codes[order[k].second]);
        if (info) info->codes_stored = uint32_t(t->codes.size());
        *out = t;
        return IRIS_OK;
    }
    IRIS_CATCH_ALL
    return IRIS_E_INTERNAL;
}

size_t iris_template_code_count(const iris_template *t) { return t ? t->codes.size() : 0; }

iris_template *iris_template_clone(const iris_template *t) {
    if (!t) return nullptr;
    IRIS_TRY { return new iris_template(*t); }
#if defined(__cpp_exceptions) || defined(__EXCEPTIONS)
    catch (...) {
        return nullptr;
    }
#else
    else {}
#endif
    return nullptr;
}

void iris_template_destroy(iris_template *t) { delete t; }

/*
 * Format v1 (little-endian):
 *   0  "CIRT"            4  u16 version     6  u16 header size (32)
 *   8  u16 code_rows    10  u16 code_cols  12  u8 n_filters  13 u8 words/column
 *  14  u16 n_codes      16  u32 encoder_id 20  u32 flags     24 u32 payload bytes  28 u32 0
 *  32  per code: f32 quality, u32 valid_bits, bits[cols*words] u64, mask[cols*words] u64
 * end  u32 CRC-32 of all preceding bytes
 */
iris_status iris_template_serialize(const iris_template *t, uint8_t *buf, size_t cap, size_t *size_out) {
    if (!t || !size_out) return IRIS_E_INVALID_ARGUMENT;
    const size_t need = serialized_size(*t);
    *size_out = need;
    if (!buf) return IRIS_OK;
    if (cap < need) return IRIS_E_BUFFER_TOO_SMALL;
    if (t->codes.size() > 0xffffu) return IRIS_E_INVALID_ARGUMENT;
    const size_t words = size_t(t->layout.cols) * t->layout.words;
    std::memcpy(buf, kMagic, 4);
    put16(buf + 4, IRIS_TEMPLATE_FORMAT_VERSION);
    put16(buf + 6, uint16_t(kHeader));
    put16(buf + 8, t->layout.rows);
    put16(buf + 10, t->layout.cols);
    buf[12] = t->layout.n_filters;
    buf[13] = t->layout.words;
    put16(buf + 14, uint16_t(t->codes.size()));
    put32(buf + 16, t->layout.encoder_id);
    put32(buf + 20, t->flags);
    put32(buf + 24, uint32_t(need - kHeader - 4));
    put32(buf + 28, 0);
    uint8_t *p = buf + kHeader;
    for (const Code &c : t->codes) {
        if (c.bits.size() != words || c.mask.size() != words) return IRIS_E_INTERNAL;
        uint32_t qb;
        std::memcpy(&qb, &c.quality, 4);
        put32(p, qb);
        put32(p + 4, c.valid_bits);
        p += 8;
        for (size_t i = 0; i < words; ++i, p += 8) put64(p, c.bits[i]);
        for (size_t i = 0; i < words; ++i, p += 8) put64(p, c.mask[i]);
    }
    put32(p, crc32(buf, size_t(p - buf)));
    return IRIS_OK;
}

iris_status iris_template_deserialize(const uint8_t *buf, size_t size, iris_template **out) {
    if (out) *out = nullptr;
    if (!buf || !out) return IRIS_E_INVALID_ARGUMENT;
    if (size < kHeader + 4 || std::memcmp(buf, kMagic, 4) != 0) return IRIS_E_BAD_TEMPLATE;
    const uint16_t ver = get16(buf + 4);
    if (ver != IRIS_TEMPLATE_FORMAT_VERSION) return IRIS_E_TEMPLATE_VERSION;
    if (get16(buf + 6) != kHeader) return IRIS_E_BAD_TEMPLATE;
    IRIS_TRY {
        iris_template t;
        t.layout.rows = get16(buf + 8);
        t.layout.cols = get16(buf + 10);
        t.layout.n_filters = buf[12];
        t.layout.words = buf[13];
        const uint16_t nc = get16(buf + 14);
        t.layout.encoder_id = get32(buf + 16);
        t.flags = get32(buf + 20);
        if (t.layout.rows == 0 || t.layout.cols == 0 || t.layout.n_filters == 0 || t.layout.n_filters > 2 ||
            t.layout.words != (unsigned(t.layout.rows) * t.layout.n_filters * 2u + 63u) / 64u || nc == 0)
            return IRIS_E_BAD_TEMPLATE;
        const size_t words = size_t(t.layout.cols) * t.layout.words;
        const size_t need = kHeader + size_t(nc) * (8 + 16 * words) + 4;
        if (size != need || get32(buf + 24) != need - kHeader - 4) return IRIS_E_BAD_TEMPLATE;
        if (crc32(buf, need - 4) != get32(buf + need - 4)) return IRIS_E_BAD_TEMPLATE;
        const uint8_t *p = buf + kHeader;
        t.codes.resize(nc);
        for (Code &c : t.codes) {
            uint32_t qb = get32(p);
            std::memcpy(&c.quality, &qb, 4);
            c.valid_bits = get32(p + 4);
            p += 8;
            c.bits.resize(words);
            c.mask.resize(words);
            for (size_t i = 0; i < words; ++i, p += 8) c.bits[i] = get64(p);
            for (size_t i = 0; i < words; ++i, p += 8) c.mask[i] = get64(p);
        }
        *out = new iris_template(std::move(t));
        return IRIS_OK;
    }
    IRIS_CATCH_ALL
    return IRIS_E_INTERNAL;
}

double iris_binomial_far(double thr, double dof) {
    if (!(dof > 0) || !(thr >= 0)) return 0.0;
    if (thr >= 1.0) return 1.0;
    // P(X <= floor(thr*N)), X ~ Binomial(N, 0.5), with N = round(dof).
    const long N = std::max(1L, std::lround(dof));
    const long kmax = long(std::floor(thr * double(N)));
    double sum = 0.0;
    const double lgN = std::lgamma(double(N) + 1.0);
    for (long k = 0; k <= kmax; ++k)
        sum += std::exp(lgN - std::lgamma(double(k) + 1.0) - std::lgamma(double(N - k) + 1.0) - double(N) * std::log(2.0));
    return std::min(1.0, sum);
}

}  // extern "C"
