"""Global configuration for the drone gesture control system.

All tunable settings live here. Both the ground station (laptop) and the
onboard (Raspberry Pi) code import from this single module.
"""

# ---------------------------------------------------------------------------
# Network topology
# ---------------------------------------------------------------------------

# IP of the ground station (laptop). The Pi sends video and proxies MAVLink
# to this address. Set to the laptop's Wi-Fi IP (e.g. 192.168.1.50).
GROUND_STATION_IP = "192.168.1.50"

# Port on which the laptop listens for the Pi's UDP video stream.
VIDEO_PORT = 5000

# Port over which MAVLink traffic flows between the laptop and the Pi.
MAVLINK_PORT = 14550

# The UDP broadcast address used by mavlink-router style setups. pymavlink
# generally talks unicast; keep this aligned with your GCS endpoint.
# 'udpin:0.0.0.0:14550' means "receive on all interfaces, port 14550".
UDP_MAVLINK_ENDPOINT = f"udpin:0.0.0.0:{MAVLINK_PORT}"

# Pi's serial connection to the flight controller.
# UART baud rate for a typical Pixhawk/companion setup.
SERIAL_DEVICE = "/dev/serial0"
SERIAL_BAUD = 921600

# ---------------------------------------------------------------------------
# Video parameters
# ---------------------------------------------------------------------------

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

# JPEG quality 0-100. Lower = smaller packets, more artifacts.
JPEG_QUALITY = 70

# The Pi reads frames from a CSI camera via V4L2 (or USB webcam).
# On the Pi with a v2 CSI camera this is typically:
CAMERA_INDEX = 0
# Set to a V4L2 path if using libcamera->v4l2loopback, otherwise leave None.
CAMERA_V4L2_PATH = None  # e.g. "/dev/video0"

# If you only want to test the gesture/GUI on your laptop (no Pi), set
# SIMULATED_SOURCE to True and point it at your built-in webcam.
SIMULATED_SOURCE = False

# ---------------------------------------------------------------------------
# Gesture recognition
# ---------------------------------------------------------------------------

# Smoothing: a recognized gesture must appear in at least this many of the
# last frame window to be "confirmed".
SMOOTHING_WINDOW = 5
SMOOTHING_MIN_VOTES = 2

# Hold-to-confirm: the same gesture must be held for this many seconds
# before a command is actually sent to the drone.
HOLD_TO_CONFIRM_SEC = 0.5

# Cooldown between commands (seconds).
COMMAND_COOLDOWN_SEC = 0.3

# Minimum confidence for MediaPipe to consider a detected hand valid.
HAND_DETECTION_CONFIDENCE = 0.5
HAND_TRACKING_CONFIDENCE = 0.5

# ---------------------------------------------------------------------------
# The 9 gestures and what they do
# ---------------------------------------------------------------------------

# Finger lookup order: [thumb, index, middle, ring, pinky]
GESTURES = {
    "OPEN_PALM":   [1, 1, 1, 1, 1],
    "FIST":        [0, 0, 0, 0, 0],
    "THUMB_UP":    [1, 0, 0, 0, 0],
    "THUMB_DOWN":  [1, 0, 0, 0, 0],
    "POINT_UP":    [0, 1, 0, 0, 0],
    "POINT_DOWN":  [0, 1, 0, 0, 0],
    "POINT_LEFT":  [0, 1, 0, 0, 0],
    "POINT_RIGHT": [0, 1, 0, 0, 0],
    "PEACE":       [0, 1, 1, 0, 0],
    "ROCK_ON":     [0, 1, 0, 0, 1],
}

# Human friendly names for logging / overlay drawing.
GESTURE_NAMES = {
    "OPEN_PALM":   "Hover",
    "FIST":        "Land",
    "THUMB_UP":    "Takeoff",
    "THUMB_DOWN":  "Move Backward",
    "POINT_UP":    "Move Forward",
    "POINT_DOWN":  "Move Backward",
    "POINT_LEFT":  "Move Left",
    "POINT_RIGHT": "Move Right",
    "PEACE":       "Ascend",
    "ROCK_ON":     "Descend",
}

# ---------------------------------------------------------------------------
# MAVLink / flight parameters
# ---------------------------------------------------------------------------

# System ID of the autopilot (Pi/Pixhawk). Typically 1.
AUTOPILOT_SYSTEM_ID = 1

# Takeoff altitude (meters) used by NAV_TAKEOFF.
TAKEOFF_ALTITUDE = 1.5

# Horizontal velocity (m/s) for the point-left/right/up/down move commands.
HORIZONTAL_SPEED = 1.0

# Vertical velocity for ascend/descend commands (m/s).
VERTICAL_SPEED = 0.3

# Type mask for SET_POSITION_TARGET_LOCAL_NED.
# Bits set to 1 mean "ignore this component", so we only use velocity.
# 0b0000_1111_1100_0111 ignores x,y,z positions as well as yaw/yaw-rate.
VELOCITY_ONLY_MASK = 0b0000111111000111

# MAVLink 2.0 over UDP. Set to False to fall back to MAVLink 1.0.
USE_MAVLINK2 = True

# True for the laptop side; legacy pymavlink 1.x used the 'udp' string.
# Keep False if your pymavlink is very old.
MAVLINK_ENDPOINT = "udpin"
