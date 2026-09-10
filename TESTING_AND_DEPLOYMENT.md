# HOW TO TEST & MAKE THE REAL DRONE WORK

Step-by-step guide from zero hardware to a flying, gesture-controlled drone.
Work through the phases **in order** — never skip ahead to flight.

---

## Quick Reference

| Thing | Value |
|-------|-------|
| Video stream | UDP, Pi → laptop, port **5000** |
| MAVLink | UDP, laptop ↔ Pi, port **14550** |
| Laptop runs | `ground_station.py`, `gesture_recognizer.py`, `video_receiver.py` |
| Pi runs | `onboard_pi.py`, `video_streamer.py`, MAVLink proxy |
| Confirm rule | 3 of 5 frames agree, held **1.0 s**, cooldown **0.3 s** |
| Movement | body-NED velocity, mask `0b0000111111000111` |

---

## Phase 0 — Test on the laptop (no drone, no Pi)

All of this runs on your Windows/macOS/Linux laptop. The webcam plays
the role of the Pi camera.

### 0.1 Install dependencies

```bash
cd drone-gesture
pip install -r requirements.txt
```

### 0.2 Run the automated tests

```bash
python test_gesture_recognizer.py    # 9-gesture classifier unit tests
python test_mavlink_pipeline.py      # smoothing / hold / cooldown / MAVLink commands
python test_mavlink_proxy.py         # Pi proxy forwards bytes both ways (UDP loopback)
python test_video_path.py            # JPEG-over-UDP round trip via your webcam
```

Expected: `ALL PASS`, `OK`, `Decoded 20/20 frames over UDP round-trip`.

### 0.3 Live gesture demo on the laptop (no drone)

Your webcam streams to the receiver in-process, and the gesture GUI runs.
Make sure no MAVLink heartbeat exists yet — the app prints
`NO MAVLINK` and continues.

```bash
python test_groundstation_pipeline.py                 # headless: receive -> detect -> classify
python src/ground_station.py --webcam --no-mavlink    # GUI: live video + gesture overlay
```

> `--webcam` uses your local camera as the video source (no Pi needed).
> `--no-mavlink` skips waiting for a flight controller so the window opens
> instantly. Without `--no-mavlink` it connects to the drone UART path in the
> background and shows `CONNECTING...` until a heartbeat arrives.

In the GUI window:
- Show your **open palm** → overlay says `Gesture: Hover`, then `SENDING: OPEN_PALM`
  after you hold it 1 s.
- Make a **fist** → `SENDING: FIST`.
- `q` quits, `H` sends hover, `ESC` sends land (land only works when connected).
- Nothing is sent to hardware yet because there is no flight controller.

> Gesture tip: use a well-lit background, keep your hand ~1 m from the camera,
> and rotate your hand slowly if the four "point" gestures misbehave.

---

## Phase 1 — Pi bring-up (no props, no motors)

### 1.1 Flash and configure the Pi

- Raspberry Pi OS **Bookworm (64-bit)**, latest.
- Enable in `raspi-config` → Interface:
  - `Camera` → yes
  - `Serial Port` → yes, and confirm `Serial console` is **off**
    (the console must not claim the UART).
- Add to `/boot/firmware/config.txt` (Bookworm) or `/boot/config.txt`:

  ```
  enable_uart=1
  ```
  (On kernel versions that need it: `dtoverlay=uart0`.)

- Reboot, then check the UART exists:

  ```bash
  ls -l /dev/serial0 /dev/ttyAMA0   # expect /dev/serial0 -> ttyAMA0 (or ttyS0)
  ```

- Check the camera:

  ```bash
  libcamera-hello                       # non-legacy; should show preview
  # Legacy V4L2 path (matches our cv2.VideoCapture(0)):
  sudo modprobe bcm2835-v4l2
  ls /dev/video0
  ```

If you only have `/dev/video0` via libcamera, the simplest path is to set
`SIMULATED_SOURCE = True` in `src/config.py` on the Pi so OpenCV uses
`cv2.VideoCapture(0)`. For best results use a USB webcam or the legacy
`bcm2835-v4l2` driver.

### 1.2 Install the code and dependencies on the Pi

```bash
cd ~
git clone <your-repo> drone-gesture -b dev   # or scp the drone-gesture folder
cd drone-gesture
sudo apt update
sudo apt install -y python3-pip python3-opencv python3-libcamera
pip install --break-system-packages -r requirements.txt pyserial
```

### 1.3 Point the Pi at your laptop

Edit `src/config.py` on the Pi — set:

```python
GROUND_STATION_IP = "192.168.1.50"   # your LAPTOP's Wi-Fi IP, not the Pi's
SERIAL_DEVICE = "/dev/serial0"        # adjust if your FC uses another port
SERIAL_BAUD = 921600                  # must match the FC's SERIALx_BAUD
```

Find the laptop IP (`ipconfig` on Windows / `ip a` on Linux). Prefer a
**dedicated access point / router** for the drone link, not the office Wi-Fi.

---

## Phase 2 — Laptop ↔ Pi integration (still no props)

Now verify the two UDP links work end to end, **before** any flight controller.

### 2.1 Video path — Pi streams, laptop shows video

On the Pi (console): `python3 src/onboard_pi.py` → start with only video:
it will also try the MAVLink proxy; if no FC is attached, disable the proxy
temporarily by commenting the proxy section in `onboard_pi.py`, or just
ignore the proxy error for this step.

On the laptop: `python src/ground_station.py`

Expect:
- Laptop window shows the Pi's camera feed, gesture overlay, `NO MAVLINK`.
- Pi console shows `[streamer] Streaming 640x480 @ 30fps ...`.

> If no video: Windows Firewall must allow Python (UDP out to Pi:5000 and in
> from it). On the Pi, `sudo iptables -P FORWARD ACCEPT` is not needed; check
> `ping` laptop IP from Pi and vice versa first.

### 2.2 MAVLink proxy test — heartbeat without motors

Attach the autopilot (Pixhawk / Cube / etc.) to the Pi over UART **now** but
leave it powered by its own bench supply, **props off**.

On the Pi: `python3 src/onboard_pi.py` (proxy now active).
On the laptop: start the ground station; the status line should flip to
`CONNECTED` within ~10 s (that means `wait_heartbeat` got a HEARTBEAT frame
relayed Pi→laptop).

If it stays `NO MAVLINK`:
- On the Pi, confirm the FC is actually emitting MAVLink on that UART
  (`SERIALx_PROTOCOL = 2` = MAVLink2, `SERIALx_BAUD` matches).
- Run a quick serial check on the Pi:

  ```bash
  stty -F /dev/serial0 921600 raw
  cat /dev/serial0 | xxd | head          # should show 0xFD (MAVLink2) frames
  ```

- Confirm the proxy log shows `[proxy] proxying ...` and no exceptions.

---

## Phase 3 — Bench test the commands safely (props OFF)

Keep props removed and the drone **tethered/strapped** so nothing can move.

1. Laptop: `python src/ground_station.py` → wait for `CONNECTED`.
2. Make an **open palm** for >1 s → command `HOVER` (harmless).
3. Make a **fist** → `LAND`. Watch in Mission Planner/QGroundControl
   (connect them to the same 14550 as a second GCS if you like) that the FC
   actually switches to LAND mode. Props off, so nothing spins.
4. **Do not** try THUMB_UP (takeoff) here — that arms motors. If you want to
   verify the command chain without arming, run Mission Planner and confirm
   the `COMMAND_LONG NAV_TAKEOFF` request appears and is rejected/acked by the
   FC (a real arm will be refused because GPS or failsafes aren't ready — that
   is exactly the behaviour you want to see).

### 3.1 Verify the telemetry the laptop sees

In the ground station GUI the status ALWAYS shows `CONNECTED`. Add this one-liner
on the laptop terminal (same project dir) to confirm two-way traffic:

```bash
python -c "from pymavlink import mavutil; c=mavutil.mavlink_connection('udpin:0.0.0.0:14550',source_system=255); c.wait_heartbeat(); print('heartbeat from sys', c.target_system)"
```

---

## Phase 4 — First controlled hover (big open space, experienced spotter)

### 4.1 Pre-flight checklist

- [ ] Props installed and torqued, all motors spin correct direction
      (Arm in a GCS once, props off, verify CCW/CW rotation).
- [ ] GPS lock (6+ sats), compass calibrated, EKF healthy.
- [ ] Battery charged; voltage failsafe configured.
- [ ] **RC transmitter bound and tested as manual override.**
      Fly in STABILIZE with the radio first. Gesture control is an additive
      mode, the radio is the safety layer.
- [ ] Failsafes that make sense for your build (LOW_BATT, RC_LOSS).
- [ ] `GROUND_STATION_IP` is correct (laptop), camera field of view covers
      where you stand.
- [ ] Laptop on Wi-Fi to the same drone AP; Pi on the same AP.
- [ ] Open, empty field / flying area, no people or obstacles overhead.

### 4.2 Flight procedure (keep it boring)

1. With the **radio**, fly to ~1.5–2 m and switch to **GUIDED**.
2. Laptop: `python src/ground_station.py` → status `CONNECTED`.
3. **Open palm** held 1 s → `HOVER` (position hold — first check it stays put).
4. **Point left/right/up/down** briefly (1 s hold) → drone translates ~1 m/s.
   Between moves keep open palm to re-hover.
5. **Peace / Rock on** → ascend/descend 0.3 m/s.
6. **Fist** → `LAND`. It auto-descends and disarms near the ground.
7. If anything looks wrong, **open palm first**, then grab the radio and go
   STABILIZE. The radio always wins.

### 4.3 The exact 9 gestures again

| Gesture | Command | Notes |
|---------|---------|-------|
| Open palm | Hover | v=0 |
| Fist | Land | modes → LAND |
| Thumb up | Takeoff (GUIDED, alt in config) | arms motors first |
| Point up      | Move forward  +1 m/s | |
| Point down    | Move backward −1 m/s | |
| Point left    | Move left    −1 m/s | body frame (y) |
| Point right   | Move right   +1 m/s | |
| Peace (✌️)    | Ascend  −0.3 m/s | vz negative = up (NED) |
| Rock on (🤘)  | Descend +0.3 m/s | |

---

## Troubleshooting

| Symptom | Likely fix |
|---------|-----------|
| No video window on laptop | Firewall (allow Python UDP in/out); check `ping` both ways; camera on Pi (`libcamera-hello`); `SIMULATED_SOURCE` on Pi correct |
| `NO MAVLINK` on laptop | FC not on UART / wrong `SERIALx_PROTOCOL` (must be 2) / baud mismatch / serial console on; proxy thread crashed (watch Pi console) |
| Gestures flicker | More light; hold farther from background; slow movements; keep hand inside frame |
| Thumb up/down confused | The classifier uses distance from the wrist — keep your palm facing the camera |
| Video lag or 5 FPS | Raise `JPEG_QUALITY` or lower to 640x480; use the drone AP; check laptop CPU |
| Commands ignore | 1 s hold not reached — you must hold the shape; cooldown 0.3 s |
| Takeoff refuses to arm | That's the FC failsafes doing their job — GPS/compass/gyro not ready. Check in the GCS |

---

## Final safety rules

- Props off until Phase 4. Always have an **experienced spotter**.
- The **RC transmitter is the override**, even when gesture control is armed.
- **Open palm = hover** is your panic gesture — keep it reliable.
- Never arm the motors via THUMB_UP unless you are in GUIDED with a solid
  position estimate and a clear sky.