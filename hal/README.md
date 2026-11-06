# cityman iris face HAL

`android.hardware.biometrics.face-service.cityman` implements `android.hardware.biometrics.face`
**V4** (`IFace/default`, NDK backend) for the Lumia 950 XL ("cityman"). Android has no iris
biometric interface any more, so the iris scanner is presented as a **weak (class 2) IR face
sensor**: Settings offers "Face unlock", the lock screen uses it; it cannot unlock keystore keys
(no payments, no banking apps).

## Architecture

```
FaceService (system_server)
   |  IFace / ISession (binder)
android.hardware.biometrics.face-service.cityman  (vendor, user system)
   |- Session: one worker thread per session, operations queued, cancellable
   |- IrisCapture: frame-locked IR pulses + engine
   |     |- IrisCamera: camera2 NDK (libcamera2ndk_vendor) -> cameraserver -> camera HAL
   |     |      iris camera = SYSTEM_CAMERA (hidden from apps), YUV 1920x1080, manual exposure
   |     |- IrLed: led:torch_0 + led:torch_1 (same IR emitter, currents add up)
   |     `- libcitymaniris_engine: segmentation, log-Gabor code, matching
   `- TemplateStore: /data/vendor_de/<user>/facedata, AES-256-GCM sealed
```

### Frame-locked pulses

Measured on the phone (6 Nov 2026, test kernel 6): `SENSOR_TIMESTAMP` is the start of exposure
of the first row (CLOCK_BOOTTIME), the rolling shutter skew is 29.81 ms, a frame lasts 33.44 ms,
the sysfs write takes 0.37 ms (max 1.3 ms) and the light follows within ~1 ms. A frame is fully
lit only when one pulse covers `[ts - 2 ms, ts + skew + exposure + 2 ms]`, about 32 ms plus the
exposure; free-running pulses lit 0 of 112 frames.

So after each frame the HAL picks a future frame `ts + m * frame` (at least K = 4 frames after the
previous target, and late enough for the duty rule) and schedules one pulse around its exposure.
The lit frame is recognised by its timestamp (frames arrive 55-68 ms after `ts`), checked against
the recorded switch times, and must be brighter than the last dark frame. The two frames next to
it are partly lit and ignored; the frame two away is the dark (ambient) reference, optionally
subtracted. `tests/timing_test.cpp` simulates this with the measured values (7.5 lit frames/s,
duty <= 33 %).

### LED limits (Config.h, not overridable)

| Limit | Value | Reason |
| --- | --- | --- |
| total current | 200 mA (two channels, 100 mA each) | Windows used 140/200/290/300 mA levels |
| pulse | <= 70 ms | Windows assist pulse length |
| duty | <= 35 % per pulse | own choice (Windows cooldown model unknown) |
| per operation | <= 4 s lit | own choice |
| per minute | <= 8 s lit | own choice |

The emitter is switched off on every error, cancel, close and service restart (rc file).

### Recognition

Each lit frame is split in two overlapping halves (`persist.vendor.iris.split`, default `rows`:
with the sensor mounted at 90 degrees the two eyes sit in the top and bottom halves of the sensor
image) and the engine looks for one iris in each. Enrollment keeps up to 16 good samples per
eye and builds a template per eye once one eye has 6; authentication accepts the first probe
that matches any stored eye, and counts one failure after 3 good non-matching frames or a
timeout with good frames. Lockout: timed (30 s) every 5 failures, permanent after 20.

### Enrollment preview

On Android 16 Settings passes a Surface in `FaceEnrollOptions.surfacePreview`; the HAL draws an
IR preview into it (`Preview.cpp`, `PreviewSink.cpp`). Only lit frames are shown (about 7.5 per
second): rotated by `ACAMERA_SENSOR_ORIENTATION` and mirrored like a front camera, a square crop
that follows the eyes with exponential smoothing of centre and zoom, bilinear sampling (from a
2x2 box-filtered half image when downscaling), percentile contrast stretch, gamma 0.85, a cold
monochrome ramp (#0B1A2A shadows to #CDEFFF / near-white highlights) and a soft vignette. For each
eye found: an antialiased ring around the iris (teal #4DD0E1 when the sample is good, amber
#FFB74D otherwise), a small ring on the pupil, and a progress arc that fills with the samples
collected for that eye. Rendering runs on its own thread with a one-frame slot (frames are
dropped while it is busy) and allocates nothing per frame; if the Surface dies (Settings paused)
drawing stops and enrollment goes on. `tests/preview_test.cpp` renders a synthetic two-eye frame
and a real frame to PPM (host: about 5 ms per 720x720 frame). The deprecated NativeHandle preview
of `enroll`/`enrollWithContext` is not supported.

## Tuning properties (root, `setprop`)

| Property | Default | Range |
| --- | --- | --- |
| `persist.vendor.iris.exposure_us` | 8000 (or the last good value) | 500-20000 |
| `persist.vendor.iris.iso` | 800 | 100-800 |
| `persist.vendor.iris.led_ma` | 140 | 25-200 (total) |
| `persist.vendor.iris.led_margin_us` | 2000 | 0-8000 |
| `persist.vendor.iris.period` | 4 | 3-8 frames |
| `persist.vendor.iris.split` | `rows` | `rows`, `cols`, `none` |
| `persist.vendor.iris.subtract` | false | ambient subtraction |
| `persist.vendor.iris.min_radius` / `max_radius` | 40 / 130 px | engine geometry |
| `persist.vendor.iris.threshold` | 0.36 | 0.20-0.40 |
| `persist.vendor.iris.camera_id` | auto | force a camera id |
| `persist.vendor.iris.preview_rotation` | sensor orientation | 0, 90, 180, 270 |
| `persist.vendor.iris.preview_mirror` | true | mirror the preview like a front camera |
| `persist.vendor.iris.dump` | false | save up to 64 lit frames to `/data/vendor/iris/dump` (biometric data!) |

## Integration (device tree)

- `PRODUCT_PACKAGES += android.hardware.biometrics.face-service.cityman` (pulls
  `libcitymaniris_engine`, `libcamera2ndk_vendor` and the AIDL libraries).
- `BOARD_VENDOR_SEPOLICY_DIRS += vendor/cityman-iris/hal/sepolicy` (or `BOARD_SEPOLICY_DIRS`).
- `frameworks/native/data/etc/android.hardware.biometrics.face.xml` in
  `vendor/etc/permissions`, or FaceService does not start.
- The camera HAL must list the iris camera with `SYSTEM_CAMERA` (or `ro.vendor.camera.mono_ids`
  with the iris camera public) and force mono on it.
- Settings must not open the front camera during enrollment: front (cost 77) + iris (cost 32)
  exceed 100 and cameraserver rejects the iris camera.
- Optional framework overlay `face_acquired_vendor`: 0 "Open your eyes", 1 "IR light cooling
  down, try again in a minute", 2 "Look at the top of the phone".

## Limits

- Weak sensor: no TEE on cityman, so the templates are protected by DAC/SELinux and sealing only;
  the HardwareAuthToken MAC is required to be present but cannot be verified (no shared HMAC key),
  and the success token is not MACed (keystore does not accept it).
- Matching threshold and eye geometry are engine defaults, not yet calibrated on real eyes.
- No liveness check beyond the IR image (the engine rejects texture-less images).

## Credits

- Structure of the service (worker thread per session, cancellation signal, registration) after the
  AOSP example face HAL, `hardware/interfaces/biometrics/face/aidl/default` (Apache-2.0).
- LED levels and pulse lengths from the reverse engineering of the Windows Phone driver
  `qccamflashtertiary8994.sys` (`IRIS-CITYMAN-2026-11-06.md` of the cityman bring-up).
