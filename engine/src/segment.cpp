// Eye localization and iris segmentation.
//
// 1. Pupil candidates: multi-level dark thresholds on a box-downsampled frame, morphological
//    opening, connected components with hole filling (the LED glint makes a hole in the pupil),
//    scored by circularity, contrast against the surrounding ring and presence of a glint.
// 2. Pupil boundary: radial edge search from the candidate centre at full resolution, robust
//    circle fit (Kasa) and then a low-order Fourier contour (Daugman 2007, "active contours").
// 3. Limbus: integro-differential operator (Daugman 1993) restricted to the lateral sectors
//    that eyelids rarely cover, with the radius bounded by the pupil radius.
// 4. Eyelids: parabolic integro-differential search above and below the pupil.
// 5. Noise mask: specular reflections, eyelashes (dark outliers), out-of-frame pixels.
#include <algorithm>
#include <cmath>
#include <cstring>

#include "iris_internal.h"

namespace citymaniris {

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float k2Pi = 6.28318530717959f;

struct Cand {
    float x, y, r;  // full-resolution (Gray8) coordinates
    float score;
    float in_level, ring_level;
    bool glint;
};

// Solve a small dense linear system in place (Gaussian elimination, partial pivoting).
bool solve(int n, double *A, double *b) {
    for (int c = 0; c < n; ++c) {
        int p = c;
        for (int r = c + 1; r < n; ++r)
            if (std::fabs(A[r * n + c]) > std::fabs(A[p * n + c])) p = r;
        if (std::fabs(A[p * n + c]) < 1e-12) return false;
        if (p != c) {
            for (int k = 0; k < n; ++k) std::swap(A[c * n + k], A[p * n + k]);
            std::swap(b[c], b[p]);
        }
        for (int r = c + 1; r < n; ++r) {
            double f = A[r * n + c] / A[c * n + c];
            for (int k = c; k < n; ++k) A[r * n + k] -= f * A[c * n + k];
            b[r] -= f * b[c];
        }
    }
    for (int c = n - 1; c >= 0; --c) {
        double s = b[c];
        for (int k = c + 1; k < n; ++k) s -= A[c * n + k] * b[k];
        b[c] = s / A[c * n + c];
    }
    return true;
}

// Median without a copy; reorders v (the value does not depend on the order).
float median_inplace(std::vector<float> &v) {
    if (v.empty()) return 0.f;
    size_t m = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + long(m), v.end());
    return v[m];
}

float median_of(std::vector<float> v) {
    if (v.empty()) return 0.f;
    size_t m = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + long(m), v.end());
    return v[m];
}

// Kasa algebraic circle fit with iterative rejection of outliers.
bool fit_circle(const std::vector<float> &xs, const std::vector<float> &ys, std::vector<uint8_t> &inl,
                float &cx, float &cy, float &r) {
    size_t n = xs.size();
    inl.assign(n, 1);
    for (int it = 0; it < 4; ++it) {
        double A[9] = {0}, b[3] = {0};
        size_t used = 0;
        for (size_t i = 0; i < n; ++i) {
            if (!inl[i]) continue;
            double x = xs[i], y = ys[i], z = -(x * x + y * y);
            double v[3] = {x, y, 1.0};
            for (int p = 0; p < 3; ++p) {
                for (int q = 0; q < 3; ++q) A[p * 3 + q] += v[p] * v[q];
                b[p] += v[p] * z;
            }
            used++;
        }
        if (used < 5 || !solve(3, A, b)) return false;
        cx = float(-b[0] / 2), cy = float(-b[1] / 2);
        double rr = double(cx) * cx + double(cy) * cy - b[2];
        if (rr <= 0) return false;
        r = float(std::sqrt(rr));
        std::vector<float> res;
        for (size_t i = 0; i < n; ++i)
            if (inl[i]) res.push_back(std::fabs(std::hypot(xs[i] - cx, ys[i] - cy) - r));
        float sig = 1.4826f * median_of(res);
        float lim = std::max(1.0f, 3.f * sig);
        bool changed = false;
        for (size_t i = 0; i < n; ++i) {
            bool in = std::fabs(std::hypot(xs[i] - cx, ys[i] - cy) - r) <= lim;
            if (in != bool(inl[i])) changed = true;
            inl[i] = in;
        }
        if (!changed && it > 0) break;
    }
    return true;
}

// Least-squares Fourier contour r(t) around (cx, cy) through the inlier points.
bool fit_fourier(const std::vector<float> &xs, const std::vector<float> &ys, std::vector<uint8_t> &inl,
                 float cx, float cy, int order, Contour &out) {
    const int np = 1 + 2 * order;
    std::vector<float> t(xs.size()), rho(xs.size());
    for (size_t i = 0; i < xs.size(); ++i) {
        t[i] = std::atan2(ys[i] - cy, xs[i] - cx);
        rho[i] = std::hypot(xs[i] - cx, ys[i] - cy);
    }
    Contour c;
    c.cx = cx;
    c.cy = cy;
    c.order = order;
    for (int it = 0; it < 3; ++it) {
        double A[49] = {0}, b[7] = {0};
        size_t used = 0;
        for (size_t i = 0; i < xs.size(); ++i) {
            if (!inl[i]) continue;
            double v[7];
            v[0] = 1;
            for (int k = 0; k < order; ++k) {
                v[1 + 2 * k] = std::cos(double(k + 1) * double(t[i]));
                v[2 + 2 * k] = std::sin(double(k + 1) * double(t[i]));
            }
            for (int p = 0; p < np; ++p) {
                for (int q = 0; q < np; ++q) A[p * np + q] += v[p] * v[q];
                b[p] += v[p] * rho[i];
            }
            used++;
        }
        if (int(used) < 2 * np + 2) return false;
        // Ridge on the harmonics keeps the contour round where there are angular gaps.
        for (int p = 1; p < np; ++p) A[p * np + p] += 0.05 * double(used);
        if (!solve(np, A, b)) return false;
        c.a0 = float(b[0]);
        for (int k = 0; k < order; ++k) {
            c.a[k] = float(b[1 + 2 * k]);
            c.b[k] = float(b[2 + 2 * k]);
        }
        std::vector<float> res;
        for (size_t i = 0; i < xs.size(); ++i)
            if (inl[i]) res.push_back(std::fabs(rho[i] - c.radius(t[i])));
        float lim = std::max(1.0f, 3.f * 1.4826f * median_of(res));
        for (size_t i = 0; i < xs.size(); ++i) inl[i] = std::fabs(rho[i] - c.radius(t[i])) <= lim;
    }
    out = c;
    return true;
}

// ---------------------------------------------------------------- pupil candidates

// Grey-level 3x3 closing (max filter then min filter), separable, branch-free inner loops.
// Outside the image is treated as bright, so dark sets touching the border are eroded like the
// binary opening with a background border.
template <bool kMax>
void hpass3(const uint8_t *src, uint8_t *dst, int w, int h) {
    for (int y = 0; y < h; ++y) {
        const uint8_t *s = src + size_t(y) * w;
        uint8_t *d = dst + size_t(y) * w;
        if (w < 2) {
            d[0] = kMax ? 255 : s[0];
            continue;
        }
        d[0] = kMax ? 255 : std::min(s[0], s[1]);
        for (int x = 1; x + 1 < w; ++x)
            d[x] = kMax ? std::max(std::max(s[x - 1], s[x]), s[x + 1]) : std::min(std::min(s[x - 1], s[x]), s[x + 1]);
        d[w - 1] = kMax ? 255 : std::min(s[w - 2], s[w - 1]);
    }
}

template <bool kMax>
void vpass3(const uint8_t *src, uint8_t *dst, int w, int h) {
    for (int y = 0; y < h; ++y) {
        uint8_t *d = dst + size_t(y) * w;
        const uint8_t *c = src + size_t(y) * w;
        if (y == 0 || y == h - 1) {
            const uint8_t *o = y == 0 ? (h > 1 ? c + w : nullptr) : c - w;
            for (int x = 0; x < w; ++x) d[x] = kMax ? 255 : std::min(c[x], o ? o[x] : c[x]);
            continue;
        }
        const uint8_t *u = c - w, *o = c + w;
        for (int x = 0; x < w; ++x)
            d[x] = kMax ? std::max(std::max(u[x], c[x]), o[x]) : std::min(std::min(u[x], c[x]), o[x]);
    }
}

void grey_close3(std::vector<uint8_t> &b, std::vector<uint8_t> &t, int w, int h) {
    hpass3<true>(b.data(), t.data(), w, h);
    vpass3<true>(t.data(), b.data(), w, h);
    hpass3<false>(b.data(), t.data(), w, h);
    vpass3<false>(t.data(), b.data(), w, h);
}

std::vector<Cand> pupil_candidates(const ImageU8 &c, int f, float rmin_full, float rmax_full, Scratch &sc) {
    const int w = c.w, h = c.h;
    std::vector<Cand> out;
    if (w < 8 || h < 8) return out;
    // Four interleaved histograms: uniform regions would otherwise serialize on one counter.
    uint32_t h4[4][256] = {{0}};
    const size_t np = c.px.size();
    size_t hi = 0;
    for (; hi + 4 <= np; hi += 4) {
        h4[0][c.px[hi]]++;
        h4[1][c.px[hi + 1]]++;
        h4[2][c.px[hi + 2]]++;
        h4[3][c.px[hi + 3]]++;
    }
    for (; hi < np; ++hi) h4[0][c.px[hi]]++;
    uint32_t hist[256];
    for (int k = 0; k < 256; ++k) hist[k] = h4[0][k] + h4[1][k] + h4[2][k] + h4[3][k];
    auto pct = [&](double q) {
        uint64_t tgt = uint64_t(q * double(c.px.size())), acc = 0;
        for (int i = 0; i < 256; ++i) {
            acc += hist[i];
            if (acc > tgt) return i;
        }
        return 255;
    };
    const int pmin = pct(0.001), p50 = pct(0.5);
    std::vector<int> levels;
    for (double q : {0.004, 0.015, 0.04, 0.1}) levels.push_back(pct(q) + 1);
    for (int d : {12, 26, 44, 64}) levels.push_back(pmin + d);
    std::sort(levels.begin(), levels.end());
    std::vector<int> th;
    for (int l : levels) {
        if (l <= pmin + 2 || l > p50) continue;
        if (!th.empty() && l - th.back() < 6) continue;
        th.push_back(l);
    }

    const float rmin = rmin_full / float(f), rmax = rmax_full / float(f);
    const float amin = kPi * rmin * rmin * 0.5f, amax = kPi * rmax * rmax * 1.2f;
    std::vector<uint8_t> &tmp = sc.ctmp;
    tmp.resize(static_cast<size_t>(w) * h);
    std::vector<Run> &runs = sc.runs;
    std::vector<size_t> &row_start = sc.row_start;
    std::vector<Comp> &comps = sc.comps;
    std::vector<int> &comp_of = sc.comp_of, &next_run = sc.next_run;
    std::vector<int32_t> &stack = sc.stack;
    std::vector<uint8_t> &box = sc.box;

    // Threshold decomposition: the binary 3x3 opening of {c < T} equals {closing(c) < T} with the
    // grey-level closing (max then min filter), so one closing serves every threshold.
    std::vector<uint8_t> &closed = sc.closed;
    closed.assign(c.px.begin(), c.px.end());
    if (rmin >= 2.f) grey_close3(closed, tmp, w, h);
    std::vector<uint8_t> &row_min = sc.row_min;
    row_min.assign(static_cast<size_t>(h), 255);
    for (int y = 0; y < h; ++y) {
        const uint8_t *r = closed.data() + size_t(y) * w;
        uint8_t m = 255;
        for (int x = 0; x < w; ++x) m = std::min(m, r[x]);
        row_min[size_t(y)] = m;
    }
    for (int T : th) {
        const uint8_t tt = uint8_t(T);
        // Run-length connected components (8-connectivity), union-find on runs: the cost follows
        // the number of runs, not the area, so large dark backgrounds stay cheap.
        runs.clear();
        row_start.assign(size_t(h) + 1, 0);
        for (int y = 0; y < h; ++y) {
            row_start[size_t(y)] = runs.size();
            if (row_min[size_t(y)] >= tt) continue;
            const uint8_t *r = closed.data() + size_t(y) * w;
            int x = 0;
            while (x < w) {
                while (x < w && r[x] >= tt) ++x;
                if (x >= w) break;
                const int x_start = x;
                while (x < w && r[x] < tt) ++x;
                runs.push_back(Run{y, x_start, x, int(runs.size())});
            }
        }
        row_start[size_t(h)] = runs.size();
        auto find = [&](int i) {
            while (runs[size_t(i)].parent != i) {
                runs[size_t(i)].parent = runs[size_t(runs[size_t(i)].parent)].parent;
                i = runs[size_t(i)].parent;
            }
            return i;
        };
        for (int y = 1; y < h; ++y) {
            size_t pq = row_start[size_t(y - 1)];
            const size_t pe = row_start[size_t(y)];
            for (size_t r = row_start[size_t(y)]; r < row_start[size_t(y + 1)]; ++r) {
                while (pq < pe && runs[pq].x1 < runs[r].x0) ++pq;
                for (size_t q = pq; q < pe && runs[q].x0 <= runs[r].x1; ++q) {
                    int ra = find(int(q)), rb = find(int(r));
                    if (ra != rb) runs[size_t(std::max(ra, rb))].parent = std::min(ra, rb);
                }
            }
        }
        comps.clear();
        comp_of.assign(runs.size(), -1);
        next_run.assign(runs.size(), -1);
        for (size_t i = 0; i < runs.size(); ++i) {
            const int root = find(int(i));
            int ci = comp_of[size_t(root)];
            if (ci < 0) {
                ci = comp_of[size_t(root)] = int(comps.size());
                comps.push_back(Comp{0, runs[i].x0, runs[i].x1 - 1, runs[i].y, runs[i].y, -1});
            }
            Comp &cp = comps[size_t(ci)];
            cp.area += runs[i].x1 - runs[i].x0;
            cp.bx0 = std::min(cp.bx0, runs[i].x0), cp.bx1 = std::max(cp.bx1, runs[i].x1 - 1);
            cp.by0 = std::min(cp.by0, runs[i].y), cp.by1 = std::max(cp.by1, runs[i].y);
            next_run[i] = cp.first_run;
            cp.first_run = int(i);
        }
        for (const Comp &cp : comps) {
            {
                const long area = cp.area;
                const int bx0 = cp.bx0, bx1 = cp.bx1, by0 = cp.by0, by1 = cp.by1;
                const int bw = bx1 - bx0 + 1, bh = by1 - by0 + 1;
                if (float(area) > amax || float(std::max(bw, bh)) < 2.f * rmin) continue;
                if (float(std::max(bw, bh)) > 2.6f * rmax) continue;
                if (std::max(bw, bh) > 2 * std::min(bw, bh)) continue;
                // Fill holes (glints): flood the background of the padded bounding box.
                const int pw = bw + 2, ph = bh + 2;
                box.assign(size_t(pw) * ph, 0);
                for (int ri = cp.first_run; ri >= 0; ri = next_run[size_t(ri)]) {
                    const Run &rn = runs[size_t(ri)];
                    std::memset(&box[size_t(rn.y - by0 + 1) * pw + size_t(rn.x0 - bx0 + 1)], 1, size_t(rn.x1 - rn.x0));
                }
                stack.clear();
                stack.push_back(0);
                box[0] = 2;
                while (!stack.empty()) {
                    int32_t i = stack.back();
                    stack.pop_back();
                    int x = i % pw, y = i / pw;
                    const int nx[4] = {x - 1, x + 1, x, x}, ny[4] = {y, y, y - 1, y + 1};
                    for (int k = 0; k < 4; ++k) {
                        if (nx[k] < 0 || ny[k] < 0 || nx[k] >= pw || ny[k] >= ph) continue;
                        size_t j = size_t(ny[k]) * pw + nx[k];
                        if (!box[j]) {
                            box[j] = 2;
                            stack.push_back(int32_t(j));
                        }
                    }
                }
                double sx = 0, sy = 0, sl = 0;
                long af = 0, ain = 0;
                for (int y = 0; y < bh; ++y)
                    for (int x = 0; x < bw; ++x) {
                        uint8_t v = box[size_t(y + 1) * pw + x + 1];
                        if (v == 2) continue;
                        af++;
                        sx += x + bx0;
                        sy += y + by0;
                        if (v == 1) {
                            sl += c.at(x + bx0, y + by0);
                            ain++;
                        }
                    }
                if (float(af) < amin) continue;
                const float req = std::sqrt(float(af) / kPi);
                const float half = 0.5f * float(std::max(bw, bh));
                const float circ = float(af) / (kPi * half * half);
                if (circ < 0.55f) continue;
                const float cx = float(sx / double(af)), cy = float(sy / double(af));
                const float in_level = float(sl / double(std::max(1L, ain)));
                // Ring around the blob: must be clearly brighter and mostly not dark.
                double rs = 0;
                int rn = 0, rdark = 0, rtot = 0;
                for (float k : {1.5f, 1.9f, 2.3f})
                    for (int a = 0; a < 48; ++a) {
                        float t = float(a) * k2Pi / 48.f;
                        int x = int(std::lround(cx + k * req * std::cos(t)));
                        int y = int(std::lround(cy + k * req * std::sin(t)));
                        rtot++;
                        if (!c.inside(x, y)) continue;
                        int v = c.at(x, y);
                        rs += v;
                        rn++;
                        if (v < T) rdark++;
                    }
                if (rn < rtot / 2) continue;
                const float ring = float(rs / rn);
                const float dark_frac = float(rdark) / float(rn);
                const float contrast = (ring - in_level) / (ring + in_level + 1.f);
                if (contrast < 0.12f || dark_frac > 0.4f) continue;
                // Glint: a bright spot inside or right next to the blob.
                const float glint_level = std::max(235.f, ring + 0.4f * (255.f - ring));  // specular: near full scale
                bool glint = false;
                const int gr = int(std::ceil(1.25f * req + 1.f));
                for (int y = int(cy) - gr; y <= int(cy) + gr && !glint; ++y)
                    for (int x = int(cx) - gr; x <= int(cx) + gr; ++x)
                        if (c.inside(x, y) && c.at(x, y) >= glint_level &&
                            (float(x) - cx) * (float(x) - cx) + (float(y) - cy) * (float(y) - cy) <=
                                float(gr * gr)) {
                            glint = true;
                            break;
                        }
                float score = circ * circ * std::min(1.f, contrast / 0.5f) * (1.f - dark_frac) * (1.f - dark_frac);
                if (glint) score *= 1.5f;
                Cand cd;
                cd.x = (cx + 0.5f) * float(f) - 0.5f;
                cd.y = (cy + 0.5f) * float(f) - 0.5f;
                cd.r = req * float(f);
                cd.score = score;
                cd.in_level = in_level;
                cd.ring_level = ring;
                cd.glint = glint;
                out.push_back(cd);
            }
        }
    }
    std::sort(out.begin(), out.end(), [](const Cand &a, const Cand &b) { return a.score > b.score; });
    std::vector<Cand> kept;
    for (const Cand &cd : out) {
        bool dup = false;
        for (const Cand &k : kept)
            // same blob seen at another threshold; a concentric blob of very different size
            // (the whole iris below a high threshold) is kept as a separate hypothesis
            if (std::hypot(cd.x - k.x, cd.y - k.y) < std::min(cd.r, k.r) &&
                std::max(cd.r, k.r) < 1.5f * std::min(cd.r, k.r)) {
                dup = true;
                break;
            }
        if (!dup) kept.push_back(cd);
        if (kept.size() >= 4) break;
    }
    return kept;
}

// ---------------------------------------------------------------- full-resolution fits

struct PupilFit {
    Contour c;
    float edge = 0, valid_frac = 0, blur_sigma = 0;
};

bool fit_pupil(const CropCtx &cc, float cx0, float cy0, float r0, float pupil_level, float ring_level,
               PupilFit &pf) {
    const float mid = 0.5f * (pupil_level + ring_level);
    const int N = std::min(128, std::max(32, int(k2Pi * r0 / 1.5f)));
    Contour cur;
    cur.cx = cx0;
    cur.cy = cy0;
    cur.a0 = r0;
    std::vector<float> xs, ys, ds, prof;
    std::vector<uint8_t> inl;
    for (int it = 0; it < 3; ++it) {
        const float wfrac = it == 0 ? 0.5f : 0.25f;
        xs.clear(), ys.clear(), ds.clear();
        for (int k = 0; k < N; ++k) {
            const float t = float(k) * k2Pi / float(N), ct = std::cos(t), st = std::sin(t);
            const float rp = cur.radius(t);
            const float ra = std::max(1.f, rp * (1.f - wfrac)), rb = rp * (1.f + wfrac) + 2.f;
            const int ns = int((rb - ra) / 0.5f) + 1;
            prof.resize(size_t(ns));
            bool ok = true;
            for (int j = 0; j < ns && ok; ++j)
                prof[size_t(j)] = bilinear(cc.s1, cur.cx + (ra + 0.5f * float(j)) * ct,
                                           cur.cy + (ra + 0.5f * float(j)) * st, &ok);
            if (!ok) continue;
            float best = 0;
            int bj = -1;
            for (int j = 3; j + 3 < ns; ++j) {
                float d = 0.5f * (prof[size_t(j + 2)] - prof[size_t(j - 2)]);
                if (d > best && prof[size_t(j - 3)] < mid) {
                    best = d;
                    bj = j;
                }
            }
            if (bj < 0 || best < 0.04f * (ring_level - pupil_level)) continue;
            const float r = ra + 0.5f * float(bj);
            bool gl = false;
            for (float dr = -3.f; dr <= 3.f && !gl; dr += 1.f)
                gl = cc.in_glint(cur.cx + (r + dr) * ct, cur.cy + (r + dr) * st);
            if (gl) continue;
            xs.push_back(cur.cx + r * ct);
            ys.push_back(cur.cy + r * st);
            ds.push_back(best);
        }
        if (xs.size() < size_t(0.4f * float(N))) return false;
        float ccx, ccy, cr;
        if (!fit_circle(xs, ys, inl, ccx, ccy, cr)) return false;
        Contour fc;
        if (xs.size() >= 16 && fit_fourier(xs, ys, inl, ccx, ccy, 2, fc)) {
            cur = fc;
        } else {
            cur = Contour();
            cur.cx = ccx, cur.cy = ccy, cur.a0 = cr;
        }
        if (!(cur.a0 > 1.f)) return false;
    }
    std::vector<float> e;
    size_t ninl = 0;
    for (size_t i = 0; i < xs.size(); ++i)
        if (inl[i]) e.push_back(ds[i]), ninl++;
    pf.c = cur;
    pf.edge = median_of(e);
    pf.valid_frac = float(ninl) / float(N);

    // Blur: mean edge profile across the boundary on the raw image; for a Gaussian-blurred step
    // of height H the peak slope is H / (sigma * sqrt(2 pi)).
    const float T = std::min(10.f, std::max(3.f, 0.6f * cur.a0));
    const int ns = int(2.f * T / 0.25f) + 1;
    std::vector<double> acc(size_t(ns), 0.0);
    int nr = 0;
    for (int k = 0; k < N; ++k) {
        const float t = float(k) * k2Pi / float(N), ct = std::cos(t), st = std::sin(t);
        const float rb = cur.radius(t);
        bool ok = true;
        std::vector<float> p(static_cast<size_t>(ns));
        for (int j = 0; j < ns && ok; ++j) {
            float r = rb - T + 0.25f * float(j);
            float x = cur.cx + r * ct, y = cur.cy + r * st;
            if (cc.in_glint(x, y)) ok = false;
            else p[size_t(j)] = bilinear(cc.raw, x, y, &ok);
        }
        if (!ok) continue;
        for (int j = 0; j < ns; ++j) acc[size_t(j)] += p[size_t(j)];
        nr++;
    }
    if (nr >= 8) {
        for (auto &v : acc) v /= nr;
        double lo = 0, hi = 0;
        for (int j = 0; j < 4; ++j) lo += acc[size_t(j)], hi += acc[size_t(ns - 1 - j)];
        const double H = (hi - lo) / 4.0;
        double dmax = 0;
        for (int j = 2; j + 2 < ns; ++j) dmax = std::max(dmax, (acc[size_t(j + 2)] - acc[size_t(j - 2)]));
        if (H > 1.0 && dmax > 1e-3) {
            double s = H / (dmax * std::sqrt(2.0 * double(kPi)));
            pf.blur_sigma = float(std::sqrt(std::max(0.0, s * s - 0.45 * 0.45)));
        } else {
            pf.blur_sigma = T;
        }
    } else {
        pf.blur_sigma = -1.f;  // unknown
    }
    return true;
}

struct LimbusFit {
    float cx = 0, cy = 0, r = 0, contrast = 0, sclera = 0;
};

// Mean of `img` along the lateral arcs of the circle; returns false when too few samples.
struct Arcs {
    std::vector<float> c, s;
    int n_right = 0;
};

bool arc_means(const CropCtx &cc, const Arcs &arcs, float cx, float cy, float r, float &mr, float &ml) {
    double sr = 0, sl = 0;
    int nr = 0, nl = 0;
    const int n = int(arcs.c.size());
    for (int i = 0; i < n; ++i) {
        float x = cx + r * arcs.c[size_t(i)], y = cy + r * arcs.s[size_t(i)];
        bool ok;
        float v = bilinear(cc.s2, x, y, &ok);
        if (!ok || cc.in_glint(x, y)) continue;
        if (i < arcs.n_right) sr += v, nr++;
        else sl += v, nl++;
    }
    const int half_r = arcs.n_right * 6 / 10, half_l = (n - arcs.n_right) * 6 / 10;
    mr = nr >= half_r && nr > 0 ? float(sr / nr) : NAN;
    ml = nl >= half_l && nl > 0 ? float(sl / nl) : NAN;
    return !(std::isnan(mr) && std::isnan(ml));
}

float side_score(float a1, float a0, float b1, float b0) {
    // a: right side (outer, inner), b: left side.
    bool av = !std::isnan(a1) && !std::isnan(a0), bv = !std::isnan(b1) && !std::isnan(b0);
    if (av && bv) return (a1 - a0) + (b1 - b0);
    if (av) return 2.f * (a1 - a0) * 0.8f;  // single side: slight penalty
    if (bv) return 2.f * (b1 - b0) * 0.8f;
    return -1e9f;
}

bool fit_limbus(const CropCtx &cc, const Contour &pupil, float r_lo, float r_hi, LimbusFit &lf) {
    if (r_hi <= r_lo + 2.f) return false;
    Arcs arcs;
    const int per = 32;
    for (int i = 0; i < per; ++i) {  // right: -30..+45 degrees (y grows downwards)
        float t = (-30.f + 75.f * (float(i) + 0.5f) / float(per)) * kPi / 180.f;
        arcs.c.push_back(std::cos(t)), arcs.s.push_back(std::sin(t));
    }
    arcs.n_right = per;
    for (int i = 0; i < per; ++i) {  // left: 135..210 degrees
        float t = (135.f + 75.f * (float(i) + 0.5f) / float(per)) * kPi / 180.f;
        arcs.c.push_back(std::cos(t)), arcs.s.push_back(std::sin(t));
    }
    const float pupil_max = pupil.max_radius();  // 64 trig evaluations: once, not per probe
    const float dc = std::max(2.f, std::min(0.08f * r_hi, 0.5f * pupil.a0 + 2.f));
    const float cs = dc > 6.f ? 2.f : 1.f;
    // Stage 1: radial profile around the pupil centre -> up to 3 candidate radii (local maxima).
    const int nrad = int(r_hi - r_lo) + 7;
    std::vector<float> MR(static_cast<size_t>(nrad)), ML(static_cast<size_t>(nrad)), D(static_cast<size_t>(nrad), -1e9f);
    for (int i = 0; i < nrad; ++i) {
        float mr, ml;
        arc_means(cc, arcs, pupil.cx, pupil.cy, r_lo - 3.f + float(i), mr, ml);
        MR[size_t(i)] = mr, ML[size_t(i)] = ml;
    }
    for (int i = 3; i + 3 < nrad; ++i) {
        const float r = r_lo - 3.f + float(i);
        if (r < r_lo || r > r_hi) continue;
        D[size_t(i)] = side_score(MR[size_t(i + 2)], MR[size_t(i - 2)], ML[size_t(i + 2)], ML[size_t(i - 2)]);
    }
    std::vector<std::pair<float, float>> peaks;  // (score, radius)
    for (int i = 3; i + 3 < nrad; ++i) {
        const float v = D[size_t(i)];
        if (v <= -1e8f) continue;
        bool is_max = true;
        for (int k = std::max(0, i - 3); k <= std::min(nrad - 1, i + 3) && is_max; ++k)
            if (k != i && D[size_t(k)] > v) is_max = false;
        if (is_max) peaks.push_back({v, r_lo - 3.f + float(i)});
    }
    std::sort(peaks.begin(), peaks.end(), [](const std::pair<float, float> &x, const std::pair<float, float> &y) {
        return x.first > y.first;
    });
    while (peaks.size() > 1 && (peaks.size() > 3 || peaks.back().first < 0.5f * peaks.front().first)) peaks.pop_back();
    // Stage 2: centre search around each candidate radius (the limbus centre is offset from the
    // pupil centre by a few percent of the iris radius).
    float best = -1e9f, bx = pupil.cx, by = pupil.cy, br = 0;
    const float rspan = std::min(5.f, dc + 1.f);
    const float cstep = std::max(cs, dc / 4.f);
    for (const auto &pk : peaks)
        for (float dy = -dc; dy <= dc + 1e-3f; dy += cstep)
            for (float dx = -dc; dx <= dc + 1e-3f; dx += cstep) {
                const float cx = pupil.cx + dx, cy = pupil.cy + dy;
                for (float r = std::max(r_lo, pk.second - rspan); r <= std::min(r_hi, pk.second + rspan) + 1e-3f; r += 1.f) {
                    if (std::hypot(dx, dy) + pupil_max > r - 9.f) continue;  // arcs clear of the pupil edge
                    float o_r, o_l, i_r, i_l;
                    arc_means(cc, arcs, cx, cy, r + 2.f, o_r, o_l);
                    arc_means(cc, arcs, cx, cy, r - 2.f, i_r, i_l);
                    const float sc = side_score(o_r, i_r, o_l, i_l);
                    if (sc > best) best = sc, bx = cx, by = cy, br = r;
                }
            }
    if (best <= -1e8f) return false;
    // Refine with half-pixel steps.
    // Refine: coordinate descent with shrinking steps (cstep/2 ... 0.25 px).
    for (float stp = 0.5f * cstep; stp >= 0.24f; stp *= 0.5f)
        for (int pass = 0; pass < 3; ++pass) {
            const float cx0p = bx, cy0p = by, r0p = br;
            for (int k = 0; k < 6; ++k) {
                const float cx = cx0p + (k == 0 ? stp : k == 1 ? -stp : 0.f);
                const float cy = cy0p + (k == 2 ? stp : k == 3 ? -stp : 0.f);
                const float r = r0p + (k == 4 ? stp : k == 5 ? -stp : 0.f);
                if (std::hypot(cx - pupil.cx, cy - pupil.cy) + pupil_max > r - 9.f) continue;
                float o_r, o_l, i_r, i_l;
                arc_means(cc, arcs, cx, cy, r + 2.f, o_r, o_l);
                arc_means(cc, arcs, cx, cy, r - 2.f, i_r, i_l);
                float sc = side_score(o_r, i_r, o_l, i_l);
                if (sc > best) best = sc, bx = cx, by = cy, br = r;
            }
            if (bx == cx0p && by == cy0p && br == r0p) break;
        }
    lf.cx = bx, lf.cy = by, lf.r = br;
    lf.contrast = 0.5f * best;
    float o_r, o_l;
    arc_means(cc, arcs, bx, by, br + 4.f, o_r, o_l);
    lf.sclera = std::isnan(o_r) ? o_l : (std::isnan(o_l) ? o_r : 0.5f * (o_r + o_l));
    return true;
}

// Parabolic eyelid search. upper: y = yv + k (x-xv)^2 with brighter skin above.
Parabola fit_lid(const CropCtx &cc, const Contour &pupil, const LimbusFit &lf, bool upper, float min_score) {
    Parabola best;
    const ImageF &im = cc.s2;
    const float R = lf.r;
    // keep the 6-px probes clear of the (blurred) pupil and limbus edges
    const float pr = pupil.max_radius() + 7.f;
    float best_s = min_score;
    const float ystart = upper ? lf.cy - R : pupil.cy, yend = upper ? pupil.cy : lf.cy + R;
    const int need = std::max(6, int(0.3f * R));
    for (float kap : {0.f, 0.15f, 0.3f, 0.45f, 0.6f, 0.8f}) {
        const float k = (upper ? kap : -kap) / R;
        for (float xo : {-0.25f, 0.f, 0.25f}) {
            const float xv = lf.cx + xo * R;
            for (float yv = ystart; yv <= yend; yv += 1.f) {
                double s = 0;
                int n = 0;
                for (float x = lf.cx - R; x <= lf.cx + R; x += 2.f) {
                    const float dxl = (x - lf.cx) * (x - lf.cx);
                    // The full test below is >= round(dx^2) whatever the FMA contraction: skip early.
                    if (dxl > (R - 6.f) * (R - 6.f)) continue;
                    const float y = yv + k * (x - xv) * (x - xv);
                    if ((x - lf.cx) * (x - lf.cx) + (y - lf.cy) * (y - lf.cy) > (R - 6.f) * (R - 6.f)) continue;
                    if ((x - pupil.cx) * (x - pupil.cx) + (y - pupil.cy) * (y - pupil.cy) < pr * pr) continue;
                    const int xi = iround(x), ya = iround(y - 3.f), yb = iround(y + 3.f);
                    if (!im.inside(xi, ya) || !im.inside(xi, yb)) continue;
                    if (cc.glint.at(xi, ya) || cc.glint.at(xi, yb)) continue;
                    float g = im.at(xi, ya) - im.at(xi, yb);
                    s += upper ? g : -g;
                    n++;
                }
                if (n < need) continue;
                float sc = float(s / n);
                if (sc > best_s) {
                    best_s = sc;
                    best.present = true;
                    best.xv = xv, best.yv = yv, best.k = k;
                }
            }
        }
    }
    return best;
}

}  // namespace

namespace {

template <typename T>
void reset_image(Image<T> &img, int w, int h, T v) {
    img.w = w, img.h = h;
    img.px.assign(size_t(w) * size_t(h), v);
}

}  // namespace

bool segment(const iris_engine &eng, Scratch &sc, const Hint *hint, Segmentation &seg, int &crop_x,
             int &crop_y, iris_status &st, ExtractDebug *dbg) {
    const iris_config &cfg = eng.cfg;
    const InputMap &m = sc.map;
    const float rmin_iris = cfg.min_iris_radius, rmax_iris = cfg.max_iris_radius;
    const float rp_min = std::max(3.f, 0.12f * rmin_iris);
    const float rp_max = 0.8f * rmax_iris;
    // Coarse factor of the candidate search: the smallest pupil spans about 2.4 coarse pixels.
    // (The historic int(rp_min / 2.4) gave 1 for the default config, because 4.8f / 2.4f rounds
    // below 2: the search ran at full resolution. A factor of 4 loses small pupils.)
    const int f = std::max(1, std::min(8, int(rp_min / 2.4f + 1e-3f)));
    auto find = [&](int x, int y, int w, int h) {
        std::vector<Cand> cs;
        if (w / f < 8 || h / f < 8) return cs;
        convert_coarse(m, f, x, y, w, h, sc.coarse, sc.rowbuf, sc.rowacc);
        cs = pupil_candidates(sc.coarse, f, rp_min, rp_max, sc);
        for (Cand &c : cs) c.x += float(x), c.y += float(y);
        return cs;
    };

    bool have = false;
    iris_status fail = IRIS_Q_NO_EYE, best_st = IRIS_OK;
    float best_total = -1.f;
    // Full-resolution fits of the best candidates; keeps the best segmentation in sc.best.
    auto fit_candidates = [&](const std::vector<Cand> &cands) {
        have = false;
        fail = IRIS_Q_NO_EYE;
        best_st = IRIS_OK;
        best_total = -1.f;
        if (dbg) {
            dbg->cand_x.clear(), dbg->cand_y.clear(), dbg->cand_r.clear(), dbg->cand_score.clear();
            for (const Cand &c : cands) {
                dbg->cand_x.push_back(c.x + float(m.x0));
                dbg->cand_y.push_back(c.y + float(m.y0));
                dbg->cand_r.push_back(c.r);
                dbg->cand_score.push_back(c.score);
            }
        }
        const size_t ntry = std::min<size_t>(3, cands.size());
        for (size_t ci = 0; ci < ntry; ++ci) {
            const Cand &cd = cands[ci];
            if (ci > 0 && cd.score < 0.25f * cands[0].score) break;
            // another hypothesis inside an eye already segmented: same eye, skip
            if (have && std::hypot(cd.x - float(crop_x) - seg.iris.cx, cd.y - float(crop_y) - seg.iris.cy) < seg.iris.a0) continue;
            const float half = std::min(rmax_iris * 1.15f, std::max(cd.r / 0.12f, rmin_iris)) + 10.f;
            const int x0 = std::max(0, int(cd.x - half)), y0 = std::max(0, int(cd.y - half));
            const int x1 = std::min(m.w, int(cd.x + half) + 1), y1 = std::min(m.h, int(cd.y + half) + 1);
            if (x1 - x0 < 8 || y1 - y0 < 8) continue;
            CropCtx &cc = sc.cur;
            convert_rect_float(m, x0, y0, x1 - x0, y1 - y0, cc.raw, sc.convtmp);
            gaussian_blur(cc.raw, cc.s1, 1.0f, sc.blur_tmp, sc.kernel, sc.col);
            gaussian_blur(cc.raw, cc.s2, 2.0f, sc.blur_tmp, sc.kernel, sc.col);
            cc.glint_level = std::max(245.f, cd.ring_level + 0.6f * (255.f - cd.ring_level));  // sclera/skin reach ~230 after auto-gain
            reset_image<uint8_t>(cc.glint, cc.raw.w, cc.raw.h, 0);
            {
                ImageU8 &raw_g = sc.raw_g;
                reset_image<uint8_t>(raw_g, cc.raw.w, cc.raw.h, 0);
                for (size_t i = 0; i < raw_g.px.size(); ++i) raw_g.px[i] = cc.raw.px[i] >= cc.glint_level;
                for (int y = 0; y < raw_g.h; ++y)
                    for (int x = 0; x < raw_g.w; ++x) {
                        if (!raw_g.at(x, y)) continue;
                        for (int dy = -2; dy <= 2; ++dy)
                            for (int dx = -2; dx <= 2; ++dx)
                                if (cc.glint.inside(x + dx, y + dy)) cc.glint.at(x + dx, y + dy) = 1;
                    }
            }
            PupilFit pf;
            const float lcx = cd.x - float(x0), lcy = cd.y - float(y0);
            if (!fit_pupil(cc, lcx, lcy, cd.r, cd.in_level, cd.ring_level, pf)) continue;
            if (fail == IRIS_Q_NO_EYE) fail = IRIS_Q_NO_IRIS;
            const float rpm = pf.c.a0, rpx = pf.c.max_radius();
            const float r_lo = std::max(0.6f * rmin_iris, std::max(1.2f * rpm, rpx + 9.f));  // clear of the blurred pupil edge
            const float r_hi = std::min(rmax_iris * 1.1f, rpm / 0.12f);
            LimbusFit lf;
            if (!fit_limbus(cc, pf.c, r_lo, r_hi, lf)) continue;
            if (lf.contrast < cfg.min_limbus_contrast) continue;
            const float ratio = rpm / lf.r;
            const float offc = std::hypot(lf.cx - pf.c.cx, lf.cy - pf.c.cy);
            if (ratio < 0.1f || ratio > 0.85f || offc + rpx > lf.r - 1.f || lf.r >= r_hi - 0.5f) {
                if (fail == IRIS_Q_NO_IRIS) fail = IRIS_Q_BAD_GEOMETRY;
                continue;
            }
            const float total = cd.score * std::min(1.f, lf.contrast / 10.f) * pf.valid_frac;
            if (total <= best_total) continue;
            best_total = total;
            have = true;
            best_st = IRIS_OK;

            Segmentation s;
            s.pupil = pf.c;
            s.iris = Contour();
            s.iris.cx = lf.cx, s.iris.cy = lf.cy, s.iris.a0 = lf.r;
            s.pupil_level = cd.in_level;
            s.limbus_contrast = lf.contrast;
            s.sclera_level = lf.sclera;
            s.pupil_contrast = cd.ring_level - cd.in_level;
            s.blur_sigma = pf.blur_sigma;
            s.candidate_score = cd.score;
            // glint near the pupil
            const float gr = pf.c.max_radius() * 1.5f;
            for (int y = int(pf.c.cy - gr); y <= int(pf.c.cy + gr) && !s.glint; ++y)
                for (int x = int(pf.c.cx - gr); x <= int(pf.c.cx + gr); ++x)
                    if (cc.raw.inside(x, y) && cc.raw.at(x, y) >= cc.glint_level) {
                        s.glint = true;
                        break;
                    }
            const float lid_min = std::max(2.5f, 0.5f * lf.contrast);
            s.upper = fit_lid(cc, pf.c, lf, true, lid_min);
            s.lower = fit_lid(cc, pf.c, lf, false, lid_min);

            // Iris statistics (raw) on the unoccluded ring.
            std::vector<float> &vals = sc.vals;
            vals.clear();
            ImageU8 &mask = sc.mask;
            reset_image<uint8_t>(mask, cc.raw.w, cc.raw.h, 0);
            // Same predicate as before, cheapest tests first; the pupil contour (atan2 + trig) is
            // only evaluated within its upper bound.
            const float pupil_bound = pf.c.radius_bound() + 1.5f;
            auto in_ring = [&](float x, float y) {
                if (std::hypot(x - lf.cx, y - lf.cy) > lf.r - 1.5f) return false;
                if (s.upper.present && y < s.upper.y(x)) return false;
                if (s.lower.present && y > s.lower.y(x)) return false;
                float dp = std::hypot(x - pf.c.cx, y - pf.c.cy);
                if (dp >= pupil_bound) return true;
                float t = std::atan2(y - pf.c.cy, x - pf.c.cx);
                if (dp < pf.c.radius(t) + 1.5f) return false;
                return true;
            };
            const int bx0 = std::max(0, int(lf.cx - lf.r)), bx1 = std::min(cc.raw.w - 1, int(lf.cx + lf.r) + 1);
            const int by0 = std::max(0, int(lf.cy - lf.r)), by1 = std::min(cc.raw.h - 1, int(lf.cy + lf.r) + 1);
            // in_ring once per pixel of the bounding box (it is used by both passes below)
            ImageU8 &ring = sc.ring;
            const int rw = bx1 - bx0 + 1, rh = by1 - by0 + 1;
            reset_image<uint8_t>(ring, std::max(0, rw), std::max(0, rh), 0);
            for (int y = by0; y <= by1; ++y)
                for (int x = bx0; x <= bx1; ++x) ring.at(x - bx0, y - by0) = in_ring(float(x), float(y)) ? 1 : 0;
            for (int y = by0; y <= by1; ++y)
                for (int x = bx0; x <= bx1; ++x)
                    if (!cc.glint.at(x, y) && ring.at(x - bx0, y - by0)) vals.push_back(cc.raw.at(x, y));
            float med = median_inplace(vals);
            for (auto &v : vals) v = std::fabs(v - med);
            float sd = 1.4826f * median_inplace(vals);
            s.iris_level = med;
            const float t_lash = std::min(med - 3.f * sd, 0.5f * (med + s.pupil_level));
            const float t_refl = med + 0.5f * (cc.glint_level - med);
            int nsat = 0, nring = 0;
            for (int y = by0; y <= by1; ++y)
                for (int x = bx0; x <= bx1; ++x) {
                    if (!ring.at(x - bx0, y - by0)) continue;
                    nring++;
                    const float v = cc.raw.at(x, y);
                    if (v >= m.sat_level) nsat++;
                    if (v < t_lash || v > t_refl) {
                        for (int dy = -1; dy <= 1; ++dy)
                            for (int dx = -1; dx <= 1; ++dx)
                                if (mask.inside(x + dx, y + dy)) mask.at(x + dx, y + dy) = 1;
                    }
                }
            for (size_t i = 0; i < mask.px.size(); ++i) mask.px[i] |= cc.glint.px[i];
            if (nring > 0 && float(nsat) > 0.2f * float(nring)) best_st = IRIS_Q_OVEREXPOSED;

            seg = s;
            crop_x = x0, crop_y = y0;
            std::swap(sc.cur, sc.best);
            std::swap(sc.mask, sc.best_mask);
        }
        return have;
    };

    // 1. Around the hint (the eye position in the previous frame), when given.
    if (hint && hint->r > 0.f) {
        const float hx = hint->x - float(m.x0), hy = hint->y - float(m.y0);
        const float hw = 2.2f * std::max(hint->r, rmin_iris) + 16.f;
        const int x0 = std::max(0, int(hx - hw)), y0 = std::max(0, int(hy - hw));
        const int x1 = std::min(m.w, int(hx + hw) + 1), y1 = std::min(m.h, int(hy + hw) + 1);
        if (x1 > x0 && y1 > y0) {
            std::vector<Cand> cands = find(x0, y0, x1 - x0, y1 - y0);
            if (!cands.empty() && fit_candidates(cands)) {
                st = best_st;
                return true;
            }
        }
    }
    // 2. Whole frame (or input ROI).
    std::vector<Cand> cands = find(0, 0, m.w, m.h);
    if (cands.empty()) {
        if (dbg) dbg->cand_x.clear(), dbg->cand_y.clear(), dbg->cand_r.clear(), dbg->cand_score.clear();
        st = IRIS_Q_NO_EYE;
        return false;
    }
    fit_candidates(cands);
    st = have ? best_st : fail;
    return have;
}

}  // namespace citymaniris
