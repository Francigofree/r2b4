# Camera driver rationale — Camera Module 3 / R2B4

## Selected production stack

```text
Sony IMX708
  -> Linux media sensor driver / CSI-2 pipeline
  -> Raspberry Pi libcamera pipeline + ISP
  -> Picamera2
  -> R2B4 NativePicamera2Camera
```

This is the supported manufacturer path for Camera Module 3. The R2B4 adapter does not replace the kernel/libcamera driver; it owns and constrains Picamera2 as the robot-facing physical capability.

## Manufacturer facts used

Raspberry Pi Camera Module 3 documentation identifies:
- Sony IMX708, 4608x2592 / 11.9 MP;
- PDAF autofocus;
- RAW10 sensor output;
- common modes including 1080p50 and 720p120;
- full support through libcamera, including autofocus.

The Raspberry Pi Picamera2 manual (Product Information Portal update 2026-09-15) documents:
- Picamera2 as the Python interface built on libcamera;
- buffer allocation guidance: video configurations use larger buffer pools to tolerate processing/encoding jitter;
- `queue=False` to prevent Picamera2 retaining a completed request that can make a later capture one frame older;
- `SensorTimestamp` in nanoseconds since system boot, sampled at first-pixel readout;
- Camera Module 3 autofocus controls through libcamera;
- multiple main/lores streams;
- request sharing to worker processes with zero-copy mapped buffers for future multiprocessing perception.

## R2B4 decisions derived from those facts

- **Picamera2/libcamera is the only physical camera owner path.**
- **No OpenCV VideoCapture / direct V4L2 camera ownership.** OpenCV may later process published frames.
- `queue=False`: person following values freshness above historical frame completeness.
- `buffer_count=6`: absorbs normal camera/encoder scheduling jitter without application-side frame backlog.
- `main=1280x720 YUV420`: efficient recording / future GUI source.
- `lores=640x360 RGB888`: bounded analysis payload for future person detection.
- Continuous PDAF autofocus, normal speed: person distance changes should not require application focus logic.
- Camera model identity must contain `imx708`.
- SensorTimestamp is retained as the physical frame time; no invented exposure-midpoint correction.
- Runtime checks reject future timestamps, excessive completion lag, and non-increasing SensorTimestamp.
- Latest-only frame publication is the initial public frame contract. The implementation can later move to shared memory / Picamera2 remote requests without changing robot consumers.
