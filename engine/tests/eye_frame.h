// Deterministic synthetic IR eye frame for tests and benchmarks: skin with an illumination
// falloff, almond sclera opening, textured iris (radial fibres, crypts, collarette), dark pupil,
// LED glint, eyelid shadow and sensor noise. Header-only, no dependency on the engine.
#pragma once

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace eyeframe {

struct Params {
    int width = 1920, height = 1080;
    float cx = 900.f, cy = 520.f;  // iris centre
    float iris_r = 85.f, pupil_r = 32.f;
    float sclera_a = 230.f, sclera_b = 120.f;  // semi-axes of the eye opening (x, y)
    float pupil_dx = 1.5f, pupil_dy = 0.5f;    // pupil offset from the iris centre
    uint32_t texture_seed = 1, noise_seed = 1;
    float noise = 2.5f;
    float rotation = 0.f;  // iris torsion, radians
};

inline std::vector<uint8_t> render(const Params &p) {
    std::vector<uint8_t> img(size_t(p.width) * p.height);
    std::mt19937 tr(p.texture_seed);
    std::uniform_real_distribution<float> u(0.f, 1.f);
    // Iris texture: sum of angular/radial waves with random phases (seeded per subject).
    constexpr int kWaves = 14;
    float fa[kWaves], fr[kWaves], ph[kWaves], am[kWaves];
    for (int i = 0; i < kWaves; ++i) {
        fa[i] = float(int(4 + 60 * u(tr)));
        fr[i] = 0.05f + 0.5f * u(tr);
        ph[i] = 6.2831853f * u(tr);
        am[i] = 4.f + 8.f * u(tr);
    }
    constexpr int kCrypts = 12;
    float ct[kCrypts], cr[kCrypts], cs[kCrypts];
    for (int i = 0; i < kCrypts; ++i) {
        ct[i] = 6.2831853f * u(tr);
        cr[i] = 0.3f + 0.6f * u(tr);
        cs[i] = 0.06f + 0.08f * u(tr);
    }
    std::mt19937 nr(p.noise_seed);
    std::normal_distribution<float> noise(0.f, p.noise);
    const float pcx = p.cx + p.pupil_dx, pcy = p.cy + p.pupil_dy;
    const float W = float(p.width), H = float(p.height);
    for (int yi = 0; yi < p.height; ++yi) {
        for (int xi = 0; xi < p.width; ++xi) {
            const float x = float(xi), y = float(yi);
            const float nx = (x - W * 0.5f) / W, ny = (y - H * 0.5f) / H;
            float v = 165.f - 150.f * (nx * nx + ny * ny) + 5.f * std::sin(x * 0.011f + y * 0.006f);
            const float ex = x - p.cx, ey = y - p.cy;
            // almond opening: two arcs, wider than tall
            const float lid = (ex * ex) / (p.sclera_a * p.sclera_a) +
                              (ey * ey) / (p.sclera_b * p.sclera_b);
            if (lid < 1.f) {
                v = 200.f - 25.f * lid;  // sclera, slightly darker towards the corners
                const float d = std::sqrt(ex * ex + ey * ey);
                if (d < p.iris_r + 0.5f) {
                    const float t = std::atan2(ey, ex) + p.rotation;
                    const float rr = (d - p.pupil_r) / (p.iris_r - p.pupil_r);
                    float iv = 92.f - 14.f * rr;
                    for (int i = 0; i < kWaves; ++i)
                        iv += am[i] * std::sin(fa[i] * t + fr[i] * d + ph[i]) * 0.5f;
                    for (int i = 0; i < kCrypts; ++i) {
                        float dt = std::remainder(t - ct[i], 6.2831853f);
                        float dr = rr - cr[i];
                        if (dt * dt + dr * dr < cs[i] * cs[i]) iv -= 22.f;
                    }
                    if (std::fabs(rr - 0.3f) < 0.03f) iv += 12.f;  // collarette
                    const float edge = std::fmin(1.f, std::fmax(0.f, p.iris_r + 0.5f - d));
                    v = v * (1.f - edge) + iv * edge;
                }
                const float dp = std::hypot(x - pcx, y - pcy);
                const float pe = std::fmin(1.f, std::fmax(0.f, p.pupil_r + 0.5f - dp));
                v = v * (1.f - pe) + 14.f * pe;
                if (lid > 0.8f) v *= 0.75f;  // lashes / lid shadow
            }
            // LED glint inside the pupil
            const float gx = x - (pcx + 0.35f * p.pupil_r), gy = y - (pcy - 0.3f * p.pupil_r);
            if (gx * gx + gy * gy < 20.f) v = 252.f;
            v += noise(nr);
            img[size_t(yi) * size_t(p.width) + size_t(xi)] = uint8_t(v < 0.f ? 0.f : v > 255.f ? 255.f : v);
        }
    }
    return img;
}

}  // namespace eyeframe
