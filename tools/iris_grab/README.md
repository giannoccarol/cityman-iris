# iris_grab

Captures IR frames from the Lumia 950 XL iris camera (camera id 2, Toshiba 0x0861,
1920x1088 RAW10) through the camera2 NDK, the path the iris HAL uses. It is a test tool and
the way to collect real frames for the engine.

## Build (host NDK, no Android tree needed)

```bash
NDK=~/Android/Sdk/ndk/android-ndk-r27c
$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android29-clang++ -std=c++17 -O2 \
  -Wall -Wextra -static-libstdc++ -o iris_grab iris_grab.cpp \
  -lcamera2ndk -lmediandk -landroid -llog -ldl
adb push iris_grab /data/local/tmp/
```

It links the system `libcamera2ndk.so` and `libmediandk.so` and runs as root (`adb root`):
the LED sysfs file is root only, and the camera service accepts uid 0 as a client.
It starts a binder thread pool through `libbinder_ndk` (loaded at run time): without it the
camera service callbacks and the image buffers never reach the process.

## Use

```bash
adb shell /data/local/tmp/iris_grab -n 3 -e 10000 -i 800 -l 200 -o /data/local/tmp/irisgrab/t1
```

| Option | Meaning | Default |
| --- | --- | --- |
| `-c ID` | camera id | 2 |
| `-n N` | frames in the burst (1-30) | 5 |
| `-e US` | exposure in µs, 0 = auto exposure | 0 |
| `-i ISO` | sensitivity, 0 = auto (the HAL caps manual ISO at 800) | 0 |
| `-l MA` | IR LED current (`led:torch_0`), 0-200 mA, on only while streaming | 0 |
| `-w N` | preview frames before the burst | 15 |
| `-o DIR` | output directory | `/data/local/tmp/iris_grab` |
| `-x` | keep the colour pipeline (no `CONTROL_EFFECT_MODE_MONO`) | |

Output per frame: `frame_N.raw16` (W*H little-endian uint16, 10-bit, black 64),
`frame_N.pgm` (8-bit view, black removed, 1st-99.5th percentile stretch), `frame_N_y.pgm`
(Y plane of the YUV output), and `meta.txt` (exposure, ISO, timestamp per frame).
The LED is switched off on exit, on errors and on SIGINT/SIGTERM/SIGHUP; waits are bounded
(4 s warm-up, 5 s burst), so the LED stays on about 1-2 s per run.

Feed a frame to the engine:

```bash
iris_cli extract frame_0.raw16 --raw16 1920 1088 10 --debug dbg.pgm -o eye.tpl
```

## Measured on the phone (6 Nov 2026, test kernel `test/cityman-iris`)

- RAW16 1920x1088 and YUV 1920x1080 in the same session; a 3-frame burst takes about 1-1.7 s
  with the LED on.
- Manual exposure is applied exactly (10 ms, 30 ms); manual ISO stops at 800.
- With `CONTROL_EFFECT_MODE_MONO` the U/V planes are exactly 128: no colour tint.
- The four CFA sites of the RAW frame have the same mean within 3 %: the sensor has no
  colour filter, the BGGR order the HAL reports is only a label.
- Dark room, LED on a near surface, 10 ms at ISO 800: 18.5 counts above black at 100 mA,
  38.9 at 200 mA; 30 ms at 200 mA: 124. Signal is linear in current and exposure.
