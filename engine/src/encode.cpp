// Rubber-sheet normalization, 1D log-Gabor phase encoding and masked Hamming comparison.
//
// Normalization: Daugman's homogeneous rubber sheet, x(r,t) = (1-r) P(t) + r L(t), between the
// pupil contour P (Fourier contour around its own centre) and the limbus circle L.
// Encoding: each normalized row is filtered with 1D log-Gabor filters (Field 1987, as used by
// Masek 2003) along the angular direction after Gaussian smoothing across rows; the phase of the
// complex response is quantized to 2 bits (Daugman). Masks: invalid pixels (eyelids, lashes,
// reflections, out of frame) and "fragile bits" whose response component is close to zero
// (Hollingsworth, Bowyer, Flynn 2009).
// Matching: fractional Hamming distance over commonly valid bits, minimized over circular shifts,
// with Daugman's normalization HDn = 0.5 - (0.5 - HD) sqrt(n / n_ref) (Daugman 2004).
#include <algorithm>
#include <cmath>

#include "iris_internal.h"

namespace citymaniris {

namespace {
constexpr float k2Pi = 6.28318530717959f;
constexpr float kRhoLo = 0.02f, kRhoHi = 0.92f;
constexpr uint32_t kAlgorithmRevision = 1;

inline int popc(uint64_t v) { return __builtin_popcountll(v); }
}  // namespace

void Fft::init(int n_) {
    n = n_;
    tw.resize(size_t(n / 2));
    for (int k = 0; k < n / 2; ++k) {
        double a = -2.0 * 3.14159265358979323846 * double(k) / double(n);
        tw[size_t(k)] = std::complex<float>(float(std::cos(a)), float(std::sin(a)));
    }
    rev.resize(size_t(n));
    int lg = 0;
    while ((1 << lg) < n) lg++;
    for (int i = 0; i < n; ++i) {
        int r = 0;
        for (int b = 0; b < lg; ++b)
            if (i & (1 << b)) r |= 1 << (lg - 1 - b);
        rev[size_t(i)] = r;
    }
}

void Fft::run(std::complex<float> *d, bool inverse) const {
    for (int i = 0; i < n; ++i)
        if (i < rev[size_t(i)]) std::swap(d[i], d[rev[size_t(i)]]);
    for (int len = 2; len <= n; len <<= 1) {
        const int half = len >> 1, step = n / len;
        for (int i = 0; i < n; i += len)
            for (int k = 0; k < half; ++k) {
                std::complex<float> w = tw[size_t(k * step)];
                if (inverse) w = std::conj(w);
                std::complex<float> u = d[i + k], v = d[i + k + half] * w;
                d[i + k] = u + v;
                d[i + k + half] = u - v;
            }
    }
    if (inverse) {
        const float s = 1.f / float(n);
        for (int i = 0; i < n; ++i) d[i] *= s;
    }
}

uint32_t compute_encoder_id(const iris_config &c) {
    uint32_t h = 2166136261u;
    auto mix = [&](uint32_t v) {
        for (int i = 0; i < 4; ++i) {
            h ^= (v >> (8 * i)) & 0xffu;
            h *= 16777619u;
        }
    };
    auto mixf = [&](float f) { mix(uint32_t(std::lround(double(f) * 1000.0))); };
    mix(kAlgorithmRevision);
    mix(c.norm_rows), mix(c.norm_cols), mix(c.code_rows), mix(c.code_cols), mix(c.n_filters);
    for (int i = 0; i < c.n_filters; ++i) mixf(c.wavelength[i]);
    mixf(c.sigma_on_f), mixf(c.radial_sigma), mixf(kRhoLo), mixf(kRhoHi);
    return h;
}

void normalize_iris(const iris_engine &eng, const Segmentation &seg, const ImageF &crop,
                    const ImageU8 &noise_mask, ImageF &norm, ImageU8 &valid) {
    const int R = eng.cfg.norm_rows, C = eng.cfg.norm_cols;
    norm.w = C, norm.h = R;
    norm.px.assign(size_t(C) * R, 0.f);  // keeps the capacity across calls
    valid.w = C, valid.h = R;
    valid.px.assign(size_t(C) * R, 0);
    for (int j = 0; j < C; ++j) {
        const float t = k2Pi * float(j) / float(C), ct = std::cos(t), st = std::sin(t);
        const float rp = seg.pupil.radius(t);
        const float px = seg.pupil.cx + rp * ct, py = seg.pupil.cy + rp * st;
        const float lx = seg.iris.cx + seg.iris.a0 * ct, ly = seg.iris.cy + seg.iris.a0 * st;
        for (int i = 0; i < R; ++i) {
            const float rho = kRhoLo + (float(i) + 0.5f) / float(R) * (kRhoHi - kRhoLo);
            const float x = px + rho * (lx - px), y = py + rho * (ly - py);
            bool ok;
            float v = bilinear(crop, x, y, &ok);
            if (ok) {
                int xi = iround(x), yi = iround(y);
                if (noise_mask.inside(xi, yi) && noise_mask.at(xi, yi)) ok = false;
                if (seg.upper.present && y < seg.upper.y(x)) ok = false;
                if (seg.lower.present && y > seg.lower.y(x)) ok = false;
            }
            norm.at(j, i) = v;
            valid.at(j, i) = ok ? 1 : 0;
        }
    }
}

void encode(const iris_engine &eng, const ImageF &norm, const ImageU8 &valid, Code &code) {
    Scratch::Encode buf;
    encode(eng, norm, valid, code, buf);
}

void encode(const iris_engine &eng, const ImageF &norm, const ImageU8 &valid, Code &code, Scratch::Encode &buf) {
    const iris_config &cfg = eng.cfg;
    const int R = cfg.norm_rows, C = cfg.norm_cols, CR = cfg.code_rows, CC = cfg.code_cols;
    const int rstep = R / CR, cstep = C / CC, W = eng.layout.words, NF = cfg.n_filters;

    // Fill invalid samples, then smooth across rows with a normalized (mask-aware) Gaussian.
    double gs = 0;
    long gn = 0;
    for (size_t i = 0; i < norm.px.size(); ++i)
        if (valid.px[i]) gs += norm.px[i], gn++;
    const float gmean = gn ? float(gs / double(gn)) : 0.f;
    ImageF &sm = buf.sm;
    sm.w = C, sm.h = R;
    sm.px.assign(size_t(C) * R, 0.f);
    const float sg = cfg.radial_sigma;
    const int kr = sg > 0.f ? std::max(1, int(std::ceil(2.5f * sg))) : 0;
    std::vector<float> &kw = buf.kw;
    kw.assign(size_t(2 * kr + 1), 1.f);
    for (int d = -kr; d <= kr; ++d) kw[size_t(d + kr)] = sg > 0.f ? std::exp(-0.5f * float(d * d) / (sg * sg)) : 1.f;
    std::vector<float> &rowmean = buf.rowmean;
    rowmean.assign(size_t(R), gmean);
    for (int i = 0; i < R; ++i) {
        double s = 0;
        int n = 0;
        for (int j = 0; j < C; ++j)
            if (valid.at(j, i)) s += norm.at(j, i), n++;
        if (n > C / 10) rowmean[size_t(i)] = float(s / n);
    }
    // Only the rows sampled by the code (k * rstep + rstep / 2) are ever read: smooth those.
    for (int i = rstep / 2; i < R; i += rstep)
        for (int j = 0; j < C; ++j) {
            float s = 0, w = 0;
            for (int d = -kr; d <= kr; ++d) {
                int ii = i + d;
                if (ii < 0 || ii >= R || !valid.at(j, ii)) continue;
                s += kw[size_t(d + kr)] * norm.at(j, ii);
                w += kw[size_t(d + kr)];
            }
            sm.at(j, i) = w > 0.2f ? s / w : rowmean[size_t(i)];
        }

    code.bits.assign(size_t(CC) * size_t(W), 0);
    code.mask.assign(size_t(CC) * size_t(W), 0);
    std::vector<std::complex<float>> &spec = buf.spec, &resp = buf.resp;
    spec.resize(size_t(C)), resp.resize(size_t(C));
    // Per filter: responses of all (row, col) for the fragile-bit quantiles.
    std::vector<float> &are = buf.are, &aim = buf.aim;
    std::vector<uint8_t> &okv = buf.okv;
    okv.assign(size_t(CR) * size_t(CC), 0);
    std::vector<float> &re_all = buf.re_all, &im_all = buf.im_all;
    re_all.assign(size_t(NF) * CR * CC, 0.f), im_all.assign(size_t(NF) * CR * CC, 0.f);
    std::vector<int> &prefix = buf.prefix;
    prefix.assign(size_t(C) + 1, 0);
    int prefix_r0 = -1, prefix_r1 = -1;
    for (int k = 0; k < CR; ++k) {
        const int i = k * rstep + rstep / 2;
        for (int j = 0; j < C; ++j) spec[size_t(j)] = std::complex<float>(sm.at(j, i) - rowmean[size_t(i)], 0.f);
        eng.fft.run(spec.data(), false);
        for (int fi = 0; fi < NF; ++fi) {
            const std::vector<float> &G = eng.gabor[fi];
            for (int j = 0; j < C; ++j) resp[size_t(j)] = spec[size_t(j)] * G[size_t(j)];
            eng.fft.run(resp.data(), true);
            const int hw = std::max(1, int(cfg.wavelength[fi] / 4.f));
            const int r0 = std::max(0, i - rstep / 2), r1 = std::min(R - 1, i + (rstep - 1) / 2);
            // Valid samples in the (rows r0..r1) x (columns j-hw..j+hw, circular) window, from
            // prefix sums of the per-column counts (the same integers as the direct count).
            if (fi == 0 || r0 != prefix_r0 || r1 != prefix_r1) {
                prefix_r0 = r0, prefix_r1 = r1;
                prefix[0] = 0;
                for (int jj = 0; jj < C; ++jj) {
                    int cnt = 0;
                    for (int ii = r0; ii <= r1; ++ii) cnt += valid.at(jj, ii);
                    prefix[size_t(jj) + 1] = prefix[size_t(jj)] + cnt;
                }
            }
            auto colsum = [&](int a, int b) {  // columns a..b inclusive, 0 <= a <= b < C
                return prefix[size_t(b) + 1] - prefix[size_t(a)];
            };
            for (int jc = 0; jc < CC; ++jc) {
                const int j = jc * cstep;
                const int nt = (r1 - r0 + 1) * (2 * hw + 1);
                int nv = 0;
                if (2 * hw + 1 >= C) {
                    for (int d = -hw; d <= hw; ++d) nv += colsum((j + d + C) % C, (j + d + C) % C);
                } else {
                    const int a = j - hw, b = j + hw;
                    if (a >= 0 && b < C) nv = colsum(a, b);
                    else if (a < 0) nv = colsum(a + C, C - 1) + colsum(0, b);
                    else nv = colsum(a, C - 1) + colsum(0, b - C);
                }
                const bool ok = valid.at(j, i) && nv * 4 >= nt * 3;
                const size_t idx = (size_t(fi) * CR + size_t(k)) * CC + size_t(jc);
                re_all[idx] = resp[size_t(j)].real();
                im_all[idx] = resp[size_t(j)].imag();
                if (fi == 0) okv[size_t(k) * CC + size_t(jc)] = ok;
                else okv[size_t(k) * CC + size_t(jc)] &= ok ? 1 : 0;
            }
        }
    }
    // Fragile-bit thresholds per filter and component.
    std::vector<float> thr_re(size_t(NF), 0.f), thr_im(size_t(NF), 0.f);
    if (cfg.fragile_fraction > 0.f) {
        for (int fi = 0; fi < NF; ++fi) {
            are.clear(), aim.clear();
            for (int k = 0; k < CR; ++k)
                for (int jc = 0; jc < CC; ++jc) {
                    if (!okv[size_t(k) * CC + size_t(jc)]) continue;
                    const size_t idx = (size_t(fi) * CR + size_t(k)) * CC + size_t(jc);
                    are.push_back(std::fabs(re_all[idx]));
                    aim.push_back(std::fabs(im_all[idx]));
                }
            if (are.empty()) continue;
            const size_t q = std::min(are.size() - 1, size_t(float(are.size()) * cfg.fragile_fraction));
            std::nth_element(are.begin(), are.begin() + long(q), are.end());
            std::nth_element(aim.begin(), aim.begin() + long(q), aim.end());
            thr_re[size_t(fi)] = are[q];
            thr_im[size_t(fi)] = aim[q];
        }
    }
    uint32_t nvalid = 0;
    for (int fi = 0; fi < NF; ++fi)
        for (int k = 0; k < CR; ++k)
            for (int jc = 0; jc < CC; ++jc) {
                const size_t idx = (size_t(fi) * CR + size_t(k)) * CC + size_t(jc);
                const bool ok = okv[size_t(k) * CC + size_t(jc)] != 0;
                const int b = (fi * CR + k) * 2;
                uint64_t *bw = &code.bits[size_t(jc) * W], *mw = &code.mask[size_t(jc) * W];
                const float re = re_all[idx], im = im_all[idx];
                if (re > 0.f) bw[b >> 6] |= uint64_t(1) << (b & 63);
                if (im > 0.f) bw[(b + 1) >> 6] |= uint64_t(1) << ((b + 1) & 63);
                if (ok && std::fabs(re) > thr_re[size_t(fi)]) {
                    mw[b >> 6] |= uint64_t(1) << (b & 63);
                    nvalid++;
                }
                if (ok && std::fabs(im) > thr_im[size_t(fi)]) {
                    mw[(b + 1) >> 6] |= uint64_t(1) << ((b + 1) & 63);
                    nvalid++;
                }
            }
    code.valid_bits = nvalid;
}

PairScore compare_codes(const iris_engine &eng, const Code &a, const Code &b) {
    const int CC = eng.layout.cols, W = eng.layout.words, S = eng.cfg.max_shift;
    const uint32_t total = eng.layout.bits_total();
    const uint32_t min_bits = std::max<uint32_t>(16u, uint32_t(eng.cfg.min_overlap_fraction * float(total)));
    const float ref = eng.cfg.reference_bits == 0.f ? 0.7f * float(total) : eng.cfg.reference_bits;
    PairScore best;
    best.score = 2.f;
    for (int s = -S; s <= S; ++s) {
        uint32_t n = 0, d = 0;
        for (int j = 0; j < CC; ++j) {
            int jb = j + s;
            if (jb < 0) jb += CC;
            else if (jb >= CC) jb -= CC;
            const uint64_t *ab = &a.bits[size_t(j) * W], *am = &a.mask[size_t(j) * W];
            const uint64_t *bb = &b.bits[size_t(jb) * W], *bm = &b.mask[size_t(jb) * W];
            for (int w = 0; w < W; ++w) {
                uint64_t m = am[w] & bm[w];
                n += uint32_t(popc(m));
                d += uint32_t(popc((ab[w] ^ bb[w]) & m));
            }
        }
        if (n < min_bits) continue;
        float raw = float(d) / float(n);
        float sc = ref > 0.f ? 0.5f - (0.5f - raw) * std::sqrt(float(n) / ref) : raw;
        sc = std::min(1.f, std::max(0.f, sc));
        if (sc < best.score || (sc == best.score && raw < best.raw)) {
            best.score = sc;
            best.raw = raw;
            best.bits = n;
            best.shift = s;
        }
    }
    if (best.score > 1.f) best = PairScore();  // no shift with enough overlap
    return best;
}

}  // namespace citymaniris
