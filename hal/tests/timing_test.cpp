// SPDX-License-Identifier: Apache-2.0
// Host test of the frame-locked pulse timing with the values measured on the phone (6/11/2026):
// frame 33.44 ms, rolling shutter skew 29.81 ms, SENSOR_TIMESTAMP = start of exposure of the
// first row, frame callback 55-68 ms after the timestamp, sysfs write 0.37 ms (max 1.3 ms).
//   g++ -std=c++17 -I../src timing_test.cpp ../src/Timing.cpp -o timing_test && ./timing_test
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "Config.h"
#include "Timing.h"

using namespace cityman::iris;

namespace {
int failures = 0;
#define CHECK(c)                                                    \
    do {                                                            \
        if (!(c)) {                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            failures++;                                             \
        }                                                           \
    } while (0)

constexpr int64_t ms = 1000000;

// Same rule as IrLed::minGapNs.
int64_t minGap(int64_t pulse) {
    int64_t g = pulse * (1000 - kLedMaxDutyPermille) / kLedMaxDutyPermille;
    return g > kLedMinGapMs * ms ? g : kLedMinGapMs * ms;
}

void simulate(int64_t exposure, int period, int64_t writeLatency, int64_t callbackLatency) {
    const int64_t fd = 33440000, skew = 29810000, margin = kLedMarginUs * 1000LL;
    const int64_t t0 = 1000 * ms;
    std::vector<LedPulse> pulses;
    int64_t target = 0, prevTarget = 0;
    int lit = 0, targets = 0, darkFrames = 0, partial = 0, firstLit = -1;
    double onTotal = 0, maxDuty = 0;
    const int frames = 300;
    for (int n = 0; n < frames; n++) {
        const int64_t ts = t0 + n * fd;
        const int64_t now = ts + callbackLatency;
        const bool fully = frameFullyLit(ts, skew, exposure, 0, pulses);
        const bool dark = frameDark(ts, skew, exposure, pulses);
        const bool isTarget = target && llabs(ts - target) <= fd / 4;
        if (isTarget) {
            CHECK(fully);
            lit += fully;
            if (firstLit < 0) firstLit = n;
            target = 0;
        } else {
            CHECK(!fully);
            if (!dark) partial++;
        }
        darkFrames += dark;
        // A pulse is "pending" until it ends.
        const bool pending = !pulses.empty() && pulses.back().offNs > now;
        if (target == 0 && !pending) {
            const int64_t len = skew + exposure + 2 * margin;
            int64_t earliest = 0;
            if (!pulses.empty()) {
                const auto& l = pulses.back();
                earliest = l.offNs + std::max(minGap(l.offNs - l.onNs), minGap(len));
            }
            const int64_t t = nextTarget(ts, fd, margin, now, prevTarget, period, earliest);
            CHECK(t - margin >= now + ms);
            CHECK(len <= kLedMaxPulseMs * ms);
            if (!pulses.empty()) {
                const auto& l = pulses.back();
                const double duty = double(l.offNs - l.onNs) /
                                    double(t - margin + writeLatency - l.onNs);
                if (duty > maxDuty) maxDuty = duty;
            }
            pulses.push_back({t - margin + writeLatency, t + skew + exposure + margin + writeLatency,
                              140});
            onTotal += double(len) / ms;
            target = prevTarget = t;
            targets++;
        }
    }
    // The first pulse goes on the first frame it can still cover once frame 0 has arrived
    // (frame 0 is seen callbackLatency after its timestamp), not after a full cadence.
    int expectFirst = 1;
    while (t0 + expectFirst * fd - margin < t0 + callbackLatency + ms) expectFirst++;
    CHECK(firstLit == expectFirst);
    const double seconds = frames * 33.44 / 1000;
    std::printf("exp %2lld ms K=%d write %.2f ms cb %lld ms: first lit frame %d (%.0f ms), %d/%d targets lit, "
                "%.1f lit/s, dark %d, partial %d, duty max %.0f %%, avg on %.0f %%\n",
                (long long)(exposure / ms), period, writeLatency / 1e6,
                (long long)(callbackLatency / ms), firstLit, firstLit * 33.44, lit, targets, lit / seconds, darkFrames, partial,
                maxDuty * 100, onTotal / (seconds * 1000) * 100);
    CHECK(lit == targets || lit == targets - 1);  // the last one may be still in flight
    CHECK(maxDuty <= kLedMaxDutyPermille / 1000.0 + 1e-9);
}
}  // namespace

int main() {
    for (int64_t exp : {5 * ms, 8 * ms, 10 * ms, 20 * ms}) {
        for (int64_t cb : {55 * ms, 68 * ms}) {
            simulate(exp, kLedFramePeriod, 370000, cb);
            simulate(exp, kLedFramePeriod, 1300000, cb);
        }
    }
    // Free-running pulses never light a whole frame: a pulse shorter than skew + exposure.
    std::vector<LedPulse> p = {{0, 30 * ms, 100}};
    CHECK(!frameFullyLit(1 * ms, 29810000, 5 * ms, 0, p));
    std::printf(failures ? "%d FAILURES\n" : "all timing checks passed\n", failures);
    return failures ? 1 : 0;
}
