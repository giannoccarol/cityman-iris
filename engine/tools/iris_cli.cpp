// iris_cli: run the engine on captured frames and compare templates.
//
//   iris_cli extract IMAGE [--raw10 W H [STRIDE] | --raw16 W H [BITS]] [--roi X Y W H]
//                          [-o OUT.tpl] [--debug OUT.ppm] [--min-r R] [--max-r R]
//                          [--hint X Y R | --hint-auto] [--bench N]
//   iris_cli match A.tpl B.tpl
//   iris_cli enroll OUT.tpl IN1.tpl IN2.tpl ...
//
// IMAGE is a binary PGM (P5, 8 or 16 bit big-endian) unless --raw10/--raw16 is given
// (headerless dump, e.g. from the RDI path). --debug writes the crop around the eye with the
// segmentation drawn (green pupil, red limbus, blue eyelids, purple masked pixels) and the
// rubber sheet below it.
// --hint X Y R searches around an iris at (X, Y) with radius R first (as the HAL does with the
// position of the previous frame); --hint-auto takes it from a first run without hint. --bench N
// then times N extractions without context, with a context and with context + hint
// (median / min / mean), for the timing on the phone (taskset to pick a core).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "iris_engine.h"
#include "iris_internal.h"

using namespace citymaniris;

namespace {

bool read_file(const char *path, std::vector<uint8_t> &out) {
    FILE *f = std::fopen(path, "rb");
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (n < 0) {
        std::fclose(f);
        return false;
    }
    out.resize(size_t(n));
    bool ok = std::fread(out.data(), 1, out.size(), f) == out.size();
    std::fclose(f);
    return ok;
}

bool write_file(const char *path, const uint8_t *d, size_t n) {
    FILE *f = std::fopen(path, "wb");
    if (!f) return false;
    bool ok = std::fwrite(d, 1, n, f) == n;
    return std::fclose(f) == 0 && ok;
}

// Parses P5; 16-bit samples are converted to host order in `px16`.
bool parse_pgm(const std::vector<uint8_t> &f, int &w, int &h, int &maxval, size_t &offset) {
    size_t p = 0;
    auto skip = [&] {
        while (p < f.size()) {
            if (f[p] == '#') {
                while (p < f.size() && f[p] != '\n') p++;
            } else if (f[p] == ' ' || f[p] == '\n' || f[p] == '\r' || f[p] == '\t') {
                p++;
            } else {
                break;
            }
        }
    };
    auto num = [&](int &v) {
        skip();
        if (p >= f.size() || f[p] < '0' || f[p] > '9') return false;
        long x = 0;
        while (p < f.size() && f[p] >= '0' && f[p] <= '9' && x < 1000000) x = x * 10 + (f[p++] - '0');
        v = int(x);
        return true;
    };
    if (f.size() < 2 || f[0] != 'P' || f[1] != '5') return false;
    p = 2;
    if (!num(w) || !num(h) || !num(maxval)) return false;
    p++;  // single whitespace
    offset = p;
    if (w <= 0 || h <= 0 || maxval <= 0 || maxval > 65535) return false;
    size_t bpp = maxval > 255 ? 2 : 1;
    return f.size() >= offset + size_t(w) * size_t(h) * bpp;
}

void put_rgb(std::vector<uint8_t> &img, int w, int h, int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    uint8_t *p = &img[(size_t(y) * w + x) * 3];
    p[0] = r, p[1] = g, p[2] = b;
}

void write_debug(const char *path, const iris_image &src, const ExtractDebug &d) {
    Gray8 g;
    if (!convert_input(src, g)) return;
    const Segmentation &s = d.seg;
    const float R = s.iris.a0 > 0 ? s.iris.a0 : 80.f;
    const float cxg = (s.iris.a0 > 0 ? s.iris.cx : s.pupil.cx) + float(d.crop_x);
    const float cyg = (s.iris.a0 > 0 ? s.iris.cy : s.pupil.cy) + float(d.crop_y);
    int x0 = std::max(0, int(cxg - 1.6f * R)), y0 = std::max(0, int(cyg - 1.6f * R));
    int x1 = std::min(g.img.w, int(cxg + 1.6f * R)), y1 = std::min(g.img.h, int(cyg + 1.6f * R));
    if (s.iris.a0 <= 0 && s.pupil.a0 <= 0) x0 = 0, y0 = 0, x1 = g.img.w, y1 = g.img.h;
    const int cw = x1 - x0, ch = y1 - y0;
    const int nw = d.norm.w, nh = d.norm.h;
    const int W = std::max(cw, nw), H = ch + (nh ? nh + 4 : 0);
    std::vector<uint8_t> img(size_t(W) * H * 3, 0);
    for (int y = 0; y < ch; ++y)
        for (int x = 0; x < cw; ++x) {
            uint8_t v = g.img.at(x + x0, y + y0);
            put_rgb(img, W, H, x, y, v, v, v);
        }
    auto to_local = [&](float cx, float cy, float &x, float &y) {
        x = cx + float(d.crop_x - x0), y = cy + float(d.crop_y - y0);
    };
    for (int i = 0; i < 720; ++i) {
        float t = float(i) * 6.2831853f / 720.f, x, y;
        if (s.pupil.a0 > 0) {
            to_local(s.pupil.cx + s.pupil.radius(t) * std::cos(t), s.pupil.cy + s.pupil.radius(t) * std::sin(t), x, y);
            put_rgb(img, W, H, int(std::lround(x)), int(std::lround(y)), 0, 255, 0);
        }
        if (s.iris.a0 > 0) {
            to_local(s.iris.cx + s.iris.a0 * std::cos(t), s.iris.cy + s.iris.a0 * std::sin(t), x, y);
            put_rgb(img, W, H, int(std::lround(x)), int(std::lround(y)), 255, 0, 0);
        }
    }
    for (const Parabola *p : {&s.upper, &s.lower}) {
        if (!p->present) continue;
        for (float xc = s.iris.cx - s.iris.a0; xc <= s.iris.cx + s.iris.a0; xc += 0.5f) {
            float x, y;
            to_local(xc, p->y(xc), x, y);
            put_rgb(img, W, H, int(std::lround(x)), int(std::lround(y)), 60, 120, 255);
        }
    }
    for (size_t i = 0; i < d.cand_x.size(); ++i) {
        float cx = d.cand_x[i] - float(d.roi_x + x0), cy = d.cand_y[i] - float(d.roi_y + y0);
        for (int k = -3; k <= 3; ++k) {
            put_rgb(img, W, H, int(cx) + k, int(cy), 255, 255, 0);
            put_rgb(img, W, H, int(cx), int(cy) + k, 255, 255, 0);
        }
    }
    if (nw) {
        float lo = 1e9f, hi = -1e9f;
        for (float v : d.norm.px) lo = std::min(lo, v), hi = std::max(hi, v);
        for (int y = 0; y < nh; ++y)
            for (int x = 0; x < nw; ++x) {
                uint8_t v = uint8_t(255.f * (d.norm.at(x, y) - lo) / std::max(1.f, hi - lo));
                if (d.norm_valid.at(x, y)) put_rgb(img, W, H, x, ch + 4 + y, v, v, v);
                else put_rgb(img, W, H, x, ch + 4 + y, uint8_t(v / 2 + 80), 0, uint8_t(v / 2 + 80));
            }
    }
    char hdr[64];
    int n = std::snprintf(hdr, sizeof(hdr), "P6\n%d %d\n255\n", W, H);
    std::vector<uint8_t> out(hdr, hdr + n);
    out.insert(out.end(), img.begin(), img.end());
    if (!write_file(path, out.data(), out.size())) std::fprintf(stderr, "cannot write %s\n", path);
}

iris_template *load_template(const char *path) {
    std::vector<uint8_t> b;
    if (!read_file(path, b)) {
        std::fprintf(stderr, "cannot read %s\n", path);
        return nullptr;
    }
    iris_template *t = nullptr;
    iris_status st = iris_template_deserialize(b.data(), b.size(), &t);
    if (st != IRIS_OK) std::fprintf(stderr, "%s: %s\n", path, iris_status_string(st));
    return t;
}

bool save_template(const char *path, const iris_template *t) {
    size_t n = 0;
    iris_template_serialize(t, nullptr, 0, &n);
    std::vector<uint8_t> b(n);
    if (iris_template_serialize(t, b.data(), b.size(), &n) != IRIS_OK) return false;
    return write_file(path, b.data(), n);
}

int usage() {
    std::fprintf(stderr,
                 "usage:\n"
                 "  iris_cli extract IMAGE [--raw10 W H [STRIDE] | --raw16 W H [BITS]] [--roi X Y W H]\n"
                 "                         [-o OUT.tpl] [--debug OUT.ppm] [--min-r R] [--max-r R]\n"
                 "                         [--hint X Y R | --hint-auto] [--bench N]\n"
                 "  iris_cli match A.tpl B.tpl\n"
                 "  iris_cli enroll OUT.tpl IN1.tpl IN2.tpl ...\n");
    return 2;
}

int cmd_extract(int argc, char **argv) {
    if (argc < 3) return usage();
    const char *in = argv[2], *out = nullptr, *dbg = nullptr;
    iris_image img{};
    int raw = 0, rw = 0, rh = 0, rs = 0, bits = 10;
    iris_config cfg;
    iris_config_default(&cfg);
    Hint hint;
    bool hint_auto = false;
    int bench = 0;
    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        auto need = [&](int k) { return i + k < argc; };
        if ((a == "--raw10" || a == "--raw16") && need(2)) {
            raw = a == "--raw10" ? 10 : 16;
            rw = std::atoi(argv[++i]);
            rh = std::atoi(argv[++i]);
            if (need(1) && argv[i + 1][0] != '-') {
                if (raw == 10) rs = std::atoi(argv[++i]);
                else bits = std::atoi(argv[++i]);
            }
        } else if (a == "--roi" && need(4)) {
            img.roi_x = std::atoi(argv[++i]);
            img.roi_y = std::atoi(argv[++i]);
            img.roi_w = std::atoi(argv[++i]);
            img.roi_h = std::atoi(argv[++i]);
        } else if (a == "-o" && need(1)) {
            out = argv[++i];
        } else if (a == "--debug" && need(1)) {
            dbg = argv[++i];
        } else if (a == "--min-r" && need(1)) {
            cfg.min_iris_radius = float(std::atof(argv[++i]));
        } else if (a == "--max-r" && need(1)) {
            cfg.max_iris_radius = float(std::atof(argv[++i]));
        } else if (a == "--hint" && need(3)) {
            hint.x = float(std::atof(argv[++i]));
            hint.y = float(std::atof(argv[++i]));
            hint.r = float(std::atof(argv[++i]));
        } else if (a == "--hint-auto") {
            hint_auto = true;
        } else if (a == "--bench" && need(1)) {
            bench = std::max(0, std::atoi(argv[++i]));
        } else {
            return usage();
        }
    }
    std::vector<uint8_t> file;
    if (!read_file(in, file)) {
        std::fprintf(stderr, "cannot read %s\n", in);
        return 1;
    }
    std::vector<uint16_t> px16;
    if (raw == 10) {
        img.data = file.data(), img.width = rw, img.height = rh, img.stride = rs, img.format = IRIS_PIX_RAW10_MIPI;
        size_t st = rs ? size_t(rs) : (size_t(rw) + 3) / 4 * 5;
        if (file.size() < st * size_t(rh)) return std::fprintf(stderr, "file too short\n"), 1;
    } else if (raw == 16) {
        if (file.size() < size_t(rw) * size_t(rh) * 2) return std::fprintf(stderr, "file too short\n"), 1;
        px16.resize(size_t(rw) * rh);
        std::memcpy(px16.data(), file.data(), px16.size() * 2);
        img.data = px16.data(), img.width = rw, img.height = rh, img.format = IRIS_PIX_GRAY16, img.bits = bits;
    } else {
        int w, h, mv;
        size_t off;
        if (!parse_pgm(file, w, h, mv, off)) return std::fprintf(stderr, "%s: not a binary PGM\n", in), 1;
        img.width = w, img.height = h;
        if (mv > 255) {
            px16.resize(size_t(w) * h);
            for (size_t i = 0; i < px16.size(); ++i) px16[i] = uint16_t(file[off + 2 * i] << 8 | file[off + 2 * i + 1]);
            int b = 1;
            while ((1 << b) - 1 < mv) b++;
            img.data = px16.data(), img.format = IRIS_PIX_GRAY16, img.bits = b;
        } else {
            img.data = file.data() + off, img.format = IRIS_PIX_GRAY8;
        }
    }
    iris_engine *e = nullptr;
    if (iris_engine_create(&cfg, &e) != IRIS_OK) return std::fprintf(stderr, "bad config\n"), 1;
    iris_template *t = nullptr;
    ExtractDebug d;
    auto t0 = std::chrono::steady_clock::now();
    iris_quality q;
    std::memset(&q, 0, sizeof(q));
    q.struct_size = sizeof(q);
    Scratch sc;
    iris_status st = citymaniris::extract(*e, sc, img, hint.r > 0.f ? &hint : nullptr, &t, &q, &d);
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("status: %s (%d), %.1f ms\n", iris_status_string(st), int(st), ms);
    std::printf("exposure %.3f  eye %d  glint %d\n", double(q.exposure), q.eye_found, q.glint_found);
    std::printf("pupil (%.1f, %.1f) r %.1f   iris (%.1f, %.1f) r %.1f   ratio %.2f\n", double(q.pupil_x),
                double(q.pupil_y), double(q.pupil_r), double(q.iris_x), double(q.iris_y), double(q.iris_r),
                double(q.pupil_iris_ratio));
    std::printf("limbus contrast %.1f  pupil contrast %.1f  blur sigma %.2f px  focus %.0f\n",
                double(q.limbus_contrast), double(q.pupil_contrast), double(q.blur_sigma), double(q.focus));
    std::printf("eyelid coverage %.2f  usable bits %.2f  overall %.0f\n", double(q.eyelid_coverage),
                double(q.usable_fraction), double(q.overall));
    if (dbg) write_debug(dbg, img, d);
    if (bench > 0) {
        iris_search_hint h{hint.x, hint.y, hint.r};
        if (hint_auto && q.eye_found) h = iris_search_hint{q.iris_x, q.iris_y, q.iris_r};
        iris_context *ctx = nullptr;
        iris_context_create(e, &ctx);
        auto timeit = [&](const char *name, iris_context *c, const iris_search_hint *hp) {
            std::vector<double> v;
            iris_status s2 = IRIS_OK;
            for (int k = 0; k < bench; ++k) {
                iris_template *tt = nullptr;
                const auto a0 = std::chrono::steady_clock::now();
                s2 = iris_extract_ex(e, c, &img, hp, &tt, nullptr);
                v.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a0).count());
                iris_template_destroy(tt);
            }
            std::sort(v.begin(), v.end());
            double sum = 0;
            for (double x : v) sum += x;
            std::printf("bench %-16s %d runs: median %.1f ms  min %.1f ms  mean %.1f ms  (%s)\n", name, bench,
                        v[v.size() / 2], v.front(), sum / double(v.size()), iris_status_string(s2));
        };
        timeit("no context", nullptr, nullptr);
        timeit("context", ctx, nullptr);
        if (h.r > 0.f) {
            std::printf("hint (%.1f, %.1f) r %.1f\n", double(h.x), double(h.y), double(h.r));
            timeit("context + hint", ctx, &h);
        }
        iris_context_destroy(ctx);
    }
    int rc = st == IRIS_OK ? 0 : 1;
    if (t && out && !save_template(out, t)) {
        std::fprintf(stderr, "cannot write %s\n", out);
        rc = 1;
    }
    iris_template_destroy(t);
    iris_engine_destroy(e);
    return rc;
}

int cmd_match(int argc, char **argv) {
    if (argc != 4) return usage();
    iris_template *a = load_template(argv[2]), *b = load_template(argv[3]);
    iris_engine *e = nullptr;
    int rc = 1;
    if (a && b && iris_engine_create(nullptr, &e) == IRIS_OK) {
        iris_match_result r;
        auto t0 = std::chrono::steady_clock::now();
        iris_status st = iris_match_ex(e, a, b, &r);
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        iris_config c;
        iris_engine_get_config(e, &c);
        std::printf("status: %s  score %.4f  raw HD %.4f  bits %u  shift %d  (%.3f ms)  -> %s (threshold %.2f)\n",
                    iris_status_string(st), double(r.score), double(r.raw_hd), r.bits_compared, r.shift, ms,
                    st == IRIS_OK && r.score <= c.match_threshold ? "MATCH" : "no match", double(c.match_threshold));
        rc = st == IRIS_OK ? 0 : 1;
    }
    iris_engine_destroy(e);
    iris_template_destroy(a);
    iris_template_destroy(b);
    return rc;
}

int cmd_enroll(int argc, char **argv) {
    if (argc < 4) return usage();
    std::vector<iris_template *> in;
    for (int i = 3; i < argc; ++i) {
        iris_template *t = load_template(argv[i]);
        if (!t) {
            for (auto *x : in) iris_template_destroy(x);
            return 1;
        }
        in.push_back(t);
    }
    iris_engine *e = nullptr;
    iris_engine_create(nullptr, &e);
    iris_template *out = nullptr;
    iris_enroll_info info;
    iris_status st = iris_enroll(e, in.data(), in.size(), &out, &info);
    std::printf("status: %s  used %u/%u  codes %u  intra mean %.3f max %.3f\n", iris_status_string(st),
                info.samples_used, info.samples_in, info.codes_stored, double(info.mean_intra_score),
                double(info.max_intra_score));
    int rc = st == IRIS_OK && save_template(argv[2], out) ? 0 : 1;
    iris_template_destroy(out);
    for (auto *x : in) iris_template_destroy(x);
    iris_engine_destroy(e);
    return rc;
}

}  // namespace

int main(int argc, char **argv) {
    if (argc < 2) return usage();
    std::string c = argv[1];
    if (c == "extract") return cmd_extract(argc, argv);
    if (c == "match") return cmd_match(argc, argv);
    if (c == "enroll") return cmd_enroll(argc, argv);
    return usage();
}
