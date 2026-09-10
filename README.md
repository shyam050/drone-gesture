# Vision-Based Gesture Control for Autonomous Drones (Edge Offloading)

Control your drone by hand gestures. The heavy AI (hand detection + gesture
classification) runs on a laptop; the drone's Raspberry Pi only streams video
and proxies MAVLink. This pushes detection to ~25-30 FPS with ~50ms latency vs
5-8 FPS when running the model onboard the Pi.

```
  DRONE (Pi 4 + Pi Camera v2 + Pixhawk)
     capture -> JPEG -> UDP (port 5000) -----------------------\
                                                               v
  LAPTOP: UDP receiver -> MediaPipe Hands -> gesture classifier -> pymavlink
     ^                                                          |
     \------ MAVLink over UDP (port 14550) <-------- forward commands -----/
```

## Layout

| File | Runs on | Purpose |
|------|---------|---------|
| `src/config.py` | both | all settings in one file |
| `src/video_streamer.py` | Pi (`video_streamer.py` role) | capture + UDP JPEG stream |
| `src/video_receiver.py` | laptop | receive + decode UDP JPEG |
| `src/gesture_recognizer.py` | laptop | MediaPipe + 9-gesture classifier |
| `src/mavlink_controller.py` | both | MAVLink controller (laptop) + MAVLink proxy (Pi) |
| `src/ground_station.py` | laptop | main app |
| `src/onboard_pi.py` | Pi | main app (stream + proxy) |

## Requirements

Install on **both** machines:

```bash
pip install -r requirements.txt
```

- Pi only: `picamera2` (Bookworm) and `pyserial`.
- Laptop: any OpenCV + MediaPipe-capable machine (CPU runs ~30 FPS at 640x480).

## Setup

1. Edit `src/config.py`:
   - `GROUND_STATION_IP` -> your laptop's Wi-Fi IP.
   - `SERIAL_DEVICE` / `SERIAL_BAUD` on the Pi for your flight controller.
   - `TAKEOFF_ALTITUDE`, movement speeds, gesture action dict.
2. On the Pi: connect camera and run `python3 src/onboard_pi.py`.
3. On the laptop: run `python src/ground_station.py`.

Both connect to UDP `:14550`. The Pi proxies MAVLink between the FC's UART and
the laptop over Wi-Fi.

To test the GUI on the laptop with **no drone** (webcam plays the Pi camera):

```bash
python src/ground_station.py --webcam --no-mavlink
```

## Gestures -> Commands

| Gesture | Command | MAVLink |
|---------|---------|---------|
| Open palm | Hover | `SET_POSITION_TARGET_LOCAL_NED` v=0 |
| Fist | Land | `SET_MODE LAND` |
| Thumb up | Takeoff (GUIDED, 1.5 m) | `COMMAND_LONG` arm + `NAV_TAKEOFF` |
| Point up / down / left / right | Move 1 m/s | body-NED velocity |
| Peace | Ascend 0.3 m/s | body-NED `vz=-0.3` |
| Rock on | Descend 0.3 m/s | body-NED `vz=+0.3` |

All movement uses `MAV_FRAME_BODY_NED` + velocity-only mask
`0b0000111111000111`.

## Safety

- Hold-to-confirm (1 s) + 3/5 frame smoothing + 0.3 s cooldown.
- No hand detected -> send zero velocity (hover).
- Always keep an RC transmitter bound as the manual override.

## Testing on your laptop (no drone)

- `python test_video_path.py` — local UDP JPEG round-trip.
- `python test_gesture_recognizer.py` — classifier unit tests.
- `python test_mavlink_pipeline.py` — pipeline + MAVLink command tests.
- `python test_groundstation_pipeline.py` — full receive->detect->classify loop
  uses your webcam as the "Pi".

## Notes / Tuning

- Lower `JPEG_QUALITY` (70) shrinks packets; 640x480 stays under the 65 KB UDP
  limit. For higher resolution raise quality or split packets.
- Pointing up/down/left/right uses a direction-agnostic straight-finger test
  (tip-to-PIP distance vs PIP-to-MCP bone) so the same gesture works no matter
  which way you point.
- Add a new gesture by adding a `[thumb, index, middle, ring, pinky]` pattern
  to `config.GESTURES` and giving it a MAVLink command in
  `MavlinkController.execute`.