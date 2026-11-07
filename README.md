# cityman-iris

Iris unlock for the Microsoft Lumia 950 XL ("cityman") running LineageOS 23.2 (Android 16).

Android has no iris biometric anymore, so the iris scanner is exposed as a **weak (class 2) IR face
sensor**: it unlocks the phone, but apps that need strong biometrics (banking, payments, keystore
keys) still ask for the PIN. Settings and the lock screen say "Iris" through overlays in the device
tree.

| Directory | What |
| --- | --- |
| [`engine/`](engine) | Iris recognition library (`libcitymaniris_engine`): pupil/limbus segmentation, Daugman rubber sheet, log-Gabor phase code, masked Hamming matching, enrollment, versioned templates. Host CMake build and tests, `iris_cli`. |
| [`hal/`](hal) | `android.hardware.biometrics.face-service.cityman`: face AIDL V4 service. Opens the iris camera (a system camera) through the vendor camera2 NDK, pulses the IR LED in sync with the frames, runs the engine, stores sealed templates per user, draws the IR preview during enrollment. rc, VINTF and sepolicy included. |
| [`tools/iris_grab/`](tools/iris_grab) | Capture tool (RAW16/Y frames, manual exposure, steady, free or frame-locked LED pulses, timing). |

## Build

In the cityman LineageOS 23 tree this repository is `vendor/cityman-iris`
([manifest](https://github.com/giannoccarol/talkman-manifests/tree/lineage-23-cityman/lineageos-23)).
`CITYMAN_CAMERA_IRIS=true` (with the private iris camera calibration, see the device tree
`camera-sensors/ck71ir`) adds the service, the face feature and the sepolicy.

## Hardware notes

- Iris camera: Toshiba 0x0861, 1920x1088 RAW10 mono, CSIPHY2 lanes 3/4 in combo mode to CSID3.
- IR illuminator: both PMI8994 torch channels (`led:torch_0` and `led:torch_1`), currents add.
  The service lights one whole frame every 4 (pulse = rolling shutter skew + exposure + 4 ms,
  at most 70 ms, duty at most 35 %, total at most 200 mA, time budgets per operation and per
  minute); the kernel switches each channel off 100 ms after its last write.

## Status

Streams and captures on the phone; the service builds. Enrollment and unlock with a real eye,
the match threshold and the preview orientation are still to be validated on the phone.

## Credits

- J. Daugman's iris recognition methods and the other papers listed in [`engine/README.md`](engine/README.md).
- Service structure after the AOSP example face HAL, `hardware/interfaces/biometrics/face/aidl/default`.
- Camera and LED values from the reverse engineering of the Windows Phone drivers of the Lumia 950 XL,
  and the cityman kernel/device bring-up by Android4Lumia950 and its community (djokcho, EpicLPer,
  j0sh1x), hraj9258, archienz and LineageOS.

## License

Apache License 2.0, see [LICENSE](LICENSE).
