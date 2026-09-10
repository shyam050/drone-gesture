"""MAVLink controller, used on both the laptop and the Pi.

Ground station role: send the 9 gesture commands over UDP.
Pi role: bidirectional proxy between the UDP link and the FC's serial UART.

All movement commands use SET_POSITION_TARGET_LOCAL_NED in the BODY_NED frame
with a velocity-only type mask.
"""

import socket
import threading
import time

from pymavlink import mavutil

from config import (
    AUTOPILOT_SYSTEM_ID,
    GROUND_STATION_IP,
    HORIZONTAL_SPEED,
    MAVLINK_PORT,
    SERIAL_BAUD,
    SERIAL_DEVICE,
    TAKEOFF_ALTITUDE,
    VELOCITY_ONLY_MASK,
    VERTICAL_SPEED,
)


class MavlinkController:
    """High-level wrapper around a pymavlink connection."""

    # Body-frame velocity setpoints per gesture.
    GESTURE_VELOCITIES = {
        "OPEN_PALM":   (0.0, 0.0, 0.0),
        "THUMB_UP":    (0.0, 0.0, 0.0),   # takeoff handled separately
        "THUMB_DOWN":  (-HORIZONTAL_SPEED, 0.0, 0.0),
        "POINT_UP":    (HORIZONTAL_SPEED, 0.0, 0.0),
        "POINT_DOWN":  (-HORIZONTAL_SPEED, 0.0, 0.0),
        "POINT_LEFT":  (0.0, -HORIZONTAL_SPEED, 0.0),
        "POINT_RIGHT": (0.0, HORIZONTAL_SPEED, 0.0),
        "PEACE":       (0.0, 0.0, -VERTICAL_SPEED),
        "ROCK_ON":     (0.0, 0.0, VERTICAL_SPEED),
    }

    def __init__(self, endpoint=None, system_components=1):
        self.conn = None
        self.target_system = AUTOPILOT_SYSTEM_ID
        self.target_component = 1
        self.endpoint = endpoint
        self.system_components = system_components

    # ------------------------------------------------------------------ #
    # Connections
    # ------------------------------------------------------------------ #
    def connect(self, endpoint=None):
        """Open a MAVLink connection. `endpoint` is a pymavlink string like
        'udpin:0.0.0.0:14550' or '/dev/serial0'."""
        endpoint = endpoint or self.endpoint
        if endpoint is None:
            raise ValueError("connect() needs an endpoint")
        print(f"[mavlink] connecting to {endpoint}")
        self.conn = mavutil.mavlink_connection(
            endpoint, baud=SERIAL_BAUD, dialect="ardupilotmega"
        )
        self.conn.wait_heartbeat(timeout=10)
        print("[mavlink] heartbeat received")
        return self.conn

    def connect_udp(self, port=MAVLINK_PORT):
        return self.connect(f"udpin:0.0.0.0:{port}")

    def connect_serial(self, device=SERIAL_DEVICE, baud=SERIAL_BAUD):
        self.endpoint = device
        return self.connect(device)

    def set_system_id(self, sysid):
        self.target_system = sysid

    # ------------------------------------------------------------------ #
    # Command helpers
    # ------------------------------------------------------------------ #
    def send_velocity(self, vx, vy, vz, mask=None):
        """Send a body-frame velocity setpoint."""
        mask = mask if mask is not None else VELOCITY_ONLY_MASK
        self.conn.mav.set_position_target_local_ned_send(
            int(0),            # time_boot_ms (0 = asap)
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            mask,
            0, 0, 0,           # x, y, z positions (ignored)
            vx, vy, vz,        # velocities
            0, 0, 0,           # accelerations
            0, 0,              # yaw, yaw_rate
        )

    def set_mode(self, mode_name):
        """Switch to an ArduPilot mode by name, e.g. 'LAND', 'GUIDED'."""
        mode_id = self.conn.mode_mapping().get(mode_name)
        if mode_id is None:
            raise ValueError(f"unknown mode: {mode_name}")
        self.conn.set_mode_apm(mode_id)

    def arm(self):
        self.conn.arducopter_arm()
        self.conn.motors_armed_wait()

    def takeoff(self, altitude=TAKEOFF_ALTITUDE):
        """Arm and take off to `altitude` meters (GUIDED mode)."""
        self.set_mode("GUIDED")
        self.arm()
        self.conn.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0,
            0,
            altitude,
        )

    def land(self):
        self.set_mode("LAND")

    def hover(self):
        self.send_velocity(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------ #
    # Gesture dispatcher
    # ------------------------------------------------------------------ #
    def execute(self, gesture):
        """Map a confirmed gesture to a MAVLink command."""
        if gesture == "OPEN_PALM":
            print("[mavlink] HOVER")
            self.hover()
        elif gesture == "FIST":
            print("[mavlink] LAND")
            self.land()
        elif gesture == "THUMB_UP":
            print(f"[mavlink] TAKEOFF @ {TAKEOFF_ALTITUDE}m")
            self.takeoff()
        elif gesture in self.GESTURE_VELOCITIES:
            vx, vy, vz = self.GESTURE_VELOCITIES[gesture]
            print(f"[mavlink] MOVE vx={vx:.1f} vy={vy:.1f} vz={vz:.1f}")
            self.send_velocity(vx, vy, vz)
        else:
            print(f"[mavlink] no command for '{gesture}'")


class MavlinkProxy:
    """Bidirectional raw-byte proxy: UDP <-> serial UART, running on the Pi.

    The Pi is a pure "camera + radio": it must NOT re-parse MAVLink (partial
    frames would be dropped and the protocol corrupted). Instead it shuttles
    raw bytes:

      UART -> UDP : every byte read from the FC is sent in one datagram to
                    the ground station (laptop, port 14550).
      UDP -> UART : every datagram received on the Pi's bound socket is
                    written verbatim to the FC's serial port.

    Flow in the field:
      - Pi binds UDP port 14550 and sends from that same port, so the
        laptop's `udpin` replies back to (pi_ip, 14550).
      - Laptop `MavlinkController.connect_udp()` uses `udpin:0.0.0.0:14550`.
    """

    def __init__(self, serial_device=SERIAL_DEVICE, serial_baud=SERIAL_BAUD,
                 gcs_ip=GROUND_STATION_IP, gcs_port=MAVLINK_PORT,
                 bind_ip="", bind_port=MAVLINK_PORT, chunk=4096):
        self.serial_device = serial_device
        self.serial_baud = serial_baud
        self.gcs_addr = (gcs_ip, gcs_port)
        self.chunk = chunk
        self.serial = None
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind((bind_ip, bind_port))
        self.udp.settimeout(0.1)  # periodic so the loop notices shutdown
        self.running = True

    def open_serial(self):
        import serial
        self.serial = serial.Serial(self.serial_device, self.serial_baud,
                                    timeout=0.1)
        print(f"[proxy] serial {self.serial_device} @ {self.serial_baud}")

    def _uart_to_udp(self):
        """Read whatever the FC sent and forward it to the laptop."""
        while self.running:
            if self.serial and self.serial.in_waiting:
                data = self.serial.read(self.serial.in_waiting)
                if data:
                    self.udp.sendto(data, self.gcs_addr)

    def _udp_to_uart(self):
        """Receive datagrams from the laptop (gesture commands) and write
        them to the FC's serial port."""
        while self.running:
            try:
                data, _addr = self.udp.recvfrom(65536)
                if self.serial and data:
                    self.serial.write(data)
            except socket.timeout:
                continue
            except OSError:
                break

    def run(self):
        self.open_serial()
        threading.Thread(target=self._uart_to_udp, daemon=True).start()
        threading.Thread(target=self._udp_to_uart, daemon=True).start()
        print(f"[proxy] proxying serial <-> UDP "
              f"(GCS {self.gcs_addr[0]}:{self.gcs_addr[1]}). Ctrl-C to stop.")
        try:
            while self.running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if self.serial:
                self.serial.close()
            self.udp.close()
