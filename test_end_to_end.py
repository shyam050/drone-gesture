"""Closed-loop end-to-end simulation:  gesture -> pipeline -> MAVLink UDP ->
proxy -> FC serial bytes.  Everything runs on one host over loopback.

Chain under test:
  1. GestureRecognizer.classify() on synthetic landmarks  ->  gesture name
  2. GesturePipeline (smoothing + hold + cooldown)        ->  confirmed gesture
  3. MavlinkController.execute(confirmed)                 ->  MAVLink packet
     (the laptop connection is a real pymavlink udpin socket, exactly as
     MavlinkController.connect() creates it in production)
  4. MavlinkProxy (Pi side, raw UDP<->serial shuttler)    ->  FC serial buffer
  5. Decode the FC serial buffer and assert the message.

Usage: python test_end_to_end.py
"""

import os
import sys
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))
sys.path.insert(0, os.path.dirname(__file__))

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mav20

from gesture_recognizer import GestureRecognizer
from ground_station import GesturePipeline
from mavlink_controller import MavlinkController, MavlinkProxy
from test_helpers import FakeClock, make_hand

PI_PORT = 15100   # Pi-side proxy bind port
GCS_PORT = 15101  # laptop-side udpin port


class _Sink:
    """File-like object that captures whatever pymavlink writes to it."""

    def __init__(self):
        self.bytes = bytearray()

    def write(self, b):
        self.bytes += bytes(b)


def _encode_mavlink2_heartbeat():
    """Build real MAVLink2 HEARTBEAT frame bytes (v20 dialect) the way a
    flight controller would emit them. (get_msgbuf() is unreliable in this
    pymavlink build, so capture via a write sink instead.)"""
    sink = _Sink()
    m = mav20.MAVLink(sink)
    m.heartbeat_send(type=mav20.MAV_TYPE_QUADROTOR,
                     autopilot=mav20.MAV_AUTOPILOT_ARDUPILOTMEGA,
                     base_mode=0, custom_mode=0, system_status=0,
                     mavlink_version=3)
    return bytes(sink.bytes)


class FakeSerial:
    def __init__(self):
        self.in_buffer = b""
        self.out_bytes = b""

    @property
    def in_waiting(self):
        return len(self.in_buffer)

    def read(self, n):
        n = min(n, len(self.in_buffer))
        data, self.in_buffer = self.in_buffer[:n], self.in_buffer[n:]
        return data

    def write(self, data):
        self.out_bytes += data

    def close(self):
        pass


class TestEndToEnd(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock(0.0)
        self.proxy = MavlinkProxy(gcs_ip="127.0.0.1", gcs_port=GCS_PORT,
                                  bind_ip="127.0.0.1", bind_port=PI_PORT)
        self.fc = FakeSerial()
        self.proxy.serial = self.fc
        with mock.patch.object(self.proxy, "open_serial", lambda: None):
            threading.Thread(target=self.proxy.run, daemon=True).start()

        # Laptop side: same kind of connection MavlinkController.connect_udp()
        # builds in production (a pymavlink udpin socket).
        self.gcs = mavutil.mavlink_connection(
            f"udpin:127.0.0.1:{GCS_PORT}", source_system=255
        )
        self.mvc = MavlinkController()
        self.mvc.conn = self.gcs
        self.mvc.target_system = 1
        self.mvc.target_component = 1

    def tearDown(self):
        self.proxy.running = False
        try:
            self.gcs.close()
        except Exception:
            pass

    def _relay_fc_heartbeat(self):
        """Put a real MAVLink2 HEARTBEAT frame into the fake FC's serial buffer
        so the proxy relays it to the laptop and the laptop latches its peer."""
        self.fc.in_buffer += _encode_mavlink2_heartbeat()

    def _wait(self, cond, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if cond():
                return True
            time.sleep(0.02)
        return False

    def test_full_chain_rockon_to_fc_serial(self):
        # 1. Classify a real ROCK_ON hand shape.
        recognizer = GestureRecognizer()
        gesture = recognizer.classify(make_hand([0, 1, 0, 0, 1]))
        recognizer.hands.close()
        self.assertEqual(gesture, "ROCK_ON")

        # 2. Drive the pipeline to confirmation (hold + cooldown satisfied).
        pipeline = GesturePipeline(now=self.clock, window=5, min_votes=3,
                                   hold=1.0, cooldown=0.0)
        confirmed = None
        for _ in range(40):
            self.clock.advance(1.0 / 30.0)
            c, _ = pipeline.push(gesture)
            if c:
                confirmed = c
                break
        self.assertEqual(confirmed, "ROCK_ON")

        # 3. Relay the FC heartbeat through the proxy; the laptop must
        #    receive it (a real recv_match also latches the proxy as peer).
        self._relay_fc_heartbeat()
        hb = self.gcs.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        self.assertIsNotNone(hb, "laptop must receive the FC heartbeat "
                                 "relayed by the proxy")

        # 4. Execute the confirmed gesture -> MAVLink packet out to the proxy.
        self.mvc.execute(confirmed)

        # 5. The proxy must have written the packet to the FC serial buffer.
        self.assertTrue(
            self._wait(lambda: len(self.fc.out_bytes) > 0),
            "proxy must forward the command to the FC serial"
        )

        # 6. Decode it and verify velocity-only body-NED, vz = +0.3.
        data = self.fc.out_bytes
        parser = mav20.MAVLink(None)
        msg = parser.parse_char(data)
        self.assertIsNotNone(msg, "serial bytes must decode as a MAVLink msg")
        self.assertEqual(msg.get_type(), "SET_POSITION_TARGET_LOCAL_NED")
        self.assertEqual(msg.coordinate_frame,
                         mavutil.mavlink.MAV_FRAME_BODY_NED)
        self.assertEqual(msg.type_mask, 0b0000111111000111)  # velocity only
        self.assertAlmostEqual(msg.vz, 0.3, places=5)        # descend


if __name__ == "__main__":
    unittest.main()