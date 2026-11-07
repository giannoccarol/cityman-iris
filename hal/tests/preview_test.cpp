// SPDX-License-Identifier: Apache-2.0
// Host test of the enrollment preview renderer: a synthetic two-eye IR frame (sensor
// orientation, eyes in the top and bottom halves as with the sensor mounted at 90 degrees) and,
// optionally, a real Y frame from the phone (binary PGM). Writes PPM files and prints timings.
//   g++ -std=c++17 -O2 -I../src preview_test.cpp ../src/Preview.cpp -o preview_test
//   ./preview_test out [real_y.pgm]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "Preview.h"

using namespace cityman::iris;

namespace {

constexpr int W = 1920, H = 1080;

struct Eye {
    float x, y, r, pr;
};

std::vector<uint8_t> synthetic(const Eye eyes[2], uint32_t seed) {
    std::vector<uint8_t> img(size_t(W) * H);
    std::mt19937 rng(seed);
    std::normal_distribution<float> noise(0.f, 3.f);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            // Skin lit by the IR LED: bright centre, darker edges (vignetting).
            const float dx = (x - W * 0.5f) / W, dy = (y - H * 0.5f) / H;
            float v = 150.f - 160.f * (dx * dx + dy * dy) + 6.f * std::sin(x * 0.013f + y * 0.007f);
            for (int e = 0; e < 2; e++) {
                const Eye& E = eyes[e];
                const float ex = x - E.x, ey = y - E.y;
                const float d = std::sqrt(ex * ex + ey * ey);
                // Almond-shaped eye opening, wide along x (face upright).
                const float lid = (ex * ex) / (2.3f * E.r * 2.3f * E.r) +
                                  (ey * ey) / (1.25f * E.r * 1.25f * E.r);
                if (lid < 1.f) {
                    v = 175.f;  // sclera
                    if (d < E.r) {
                        const float th = std::atan2(ey, ex);
                        // Iris texture: radial fibres and crypts.
                        v = 95.f + 18.f * std::sin(th * 37.f + d * 0.21f) +
                            10.f * std::sin(th * 11.f - d * 0.35f) - 18.f * (d / E.r);
                        if (d < E.pr) v = 18.f;  // pupil
                    }
                    if (lid > 0.85f) v *= 0.7f;  // lashes shadow
                }
                // Corneal glint of the LED.
                const float gx = ex - E.pr * 0.4f, gy = ey + E.pr * 0.3f;
                if (gx * gx + gy * gy < 25.f) v = 250.f;
            }
            img[size_t(y) * W + x] = uint8_t(std::clamp(v + noise(rng), 0.f, 255.f));
        }
    }
    return img;
}

bool writePpm(const std::string& path, const std::vector<uint8_t>& rgba, int w, int h) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (size_t i = 0; i < size_t(w) * h; i++) std::fwrite(&rgba[i * 4], 1, 3, f);
    return std::fclose(f) == 0;
}

bool readPgm(const char* path, std::vector<uint8_t>* out, int* w, int* h) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return false;
    int maxv = 0;
    if (std::fscanf(f, "P5 %d %d %d", w, h, &maxv) != 3 || maxv != 255) return false;
    std::fgetc(f);
    out->resize(size_t(*w) * *h);
    const bool ok = std::fread(out->data(), 1, out->size(), f) == out->size();
    std::fclose(f);
    return ok;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string outDir = argc > 1 ? argv[1] : "out";
    const int S = 720;
    PreviewRenderer r;
    r.configure(S, S);
    std::vector<uint8_t> rgba(size_t(S) * S * 4);
    r.clear(rgba.data(), S);
    writePpm(outDir + "/preview_clear.ppm", rgba, S, S);

    // Eyes in sensor coordinates: same column, top and bottom halves (portrait use, mount 90).
    Eye eyes[2] = {{620, 520, 64, 23}, {1180, 500, 62, 21}};  // side by side, face upright
    const auto img = synthetic(eyes, 1);
    PreviewInput in;
    in.y = img.data();
    in.width = W;
    in.height = H;
    in.stride = W;
    in.rotation = 0;
    in.mirror = true;
    auto setEyes = [&](float p0, float p1, bool good1) {
        for (int e = 0; e < 2; e++) {
            PreviewEye& pe = in.eyes[e];
            pe.found = true;
            pe.good = e == 0 ? true : good1;
            pe.irisX = eyes[e].x;
            pe.irisY = eyes[e].y;
            pe.irisR = eyes[e].r;
            pe.pupilX = eyes[e].x;
            pe.pupilY = eyes[e].y;
            pe.pupilR = eyes[e].pr;
            pe.progress = e == 0 ? p0 : p1;
        }
    };

    // First frame: no eyes yet (full width view), then the eyes appear: the view zooms in over a
    // few frames (exponential smoothing).
    in.eyes[0] = in.eyes[1] = PreviewEye{};
    r.render(in, rgba.data(), S);
    writePpm(outDir + "/preview_synth_noeyes.ppm", rgba, S, S);
    double worst = 0, total = 0;
    const int frames = 12;
    for (int i = 0; i < frames; i++) {
        setEyes(std::min(1.f, (i + 1) / 6.f), std::min(1.f, i / 9.f), i > 5);
        const auto t0 = std::chrono::steady_clock::now();
        r.render(in, rgba.data(), S);
        const double ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
                        .count();
        worst = std::max(worst, ms);
        total += ms;
        if (i == 3) writePpm(outDir + "/preview_synth_zooming.ppm", rgba, S, S);
    }
    writePpm(outDir + "/preview_synth.ppm", rgba, S, S);
    // A mid-enrollment state: left eye good 4/6, right eye amber (e.g. too far) 1/6.
    setEyes(4.f / 6.f, 1.f / 6.f, false);
    for (int i = 0; i < 8; i++) r.render(in, rgba.data(), S);
    writePpm(outDir + "/preview_synth_mid.ppm", rgba, S, S);
    std::printf("synthetic: %d frames, mean %.2f ms, worst %.2f ms (host)\n", frames,
                total / frames, worst);

    if (argc > 2) {
        std::vector<uint8_t> real;
        int rw = 0, rh = 0;
        if (!readPgm(argv[2], &real, &rw, &rh)) {
            std::printf("cannot read %s\n", argv[2]);
            return 1;
        }
        PreviewRenderer r2;
        r2.configure(S, S);
        PreviewInput ri;
        ri.y = real.data();
        ri.width = rw;
        ri.height = rh;
        ri.stride = rw;
        const auto t0 = std::chrono::steady_clock::now();
        r2.render(ri, rgba.data(), S);
        const double ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
                        .count();
        writePpm(outDir + "/preview_real.ppm", rgba, S, S);
        std::printf("real %dx%d: %.2f ms (host, half-res path)\n", rw, rh, ms);
    }
    return 0;
}
