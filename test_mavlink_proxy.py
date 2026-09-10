"""Tests the MavlinkProxy byte shuttling on loopback with a fake serial".

The proxy is a raw-byte UDP<->serial bridge. Here we:
  1. bind a "GCS" socket on 127.0.0.1:PORT
  2. run the proxy pointed at gcs_ip=127.0.0.1, same PORT
  3. use a fake serial object to emulate the FC
  4. assert bytes sent by the "FC" reach the GCS socket, and bytes sent by
     the GCS socket reach the fake serial's write buffer.
"""

import os
import sys
import socket
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from mavlink_controller import MavlinkProxy

PI_PORT = 15000   # Pi-side proxy bind port
GCS_PORT = 15001  # laptop-side socket port


class FakeSerial:
    """Minimal pyserial stand-in."""
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


class TestMavlinkProxy(unittest.TestCase):
    def test_bidirectional_byte_shuttle(self):
        proxy = MavlinkProxy(gcs_ip="127.0.0.1", gcs_port=GCS_PORT,
                             bind_ip="127.0.0.1", bind_port=PI_PORT)
        proxy.serial = FakeSerial()

        # GCS side socket (acts like the laptop's udpin:0.0.0.0:14550)
        gcs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        gcs.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        gcs.bind(("127.0.0.1", GCS_PORT))
        gcs.settimeout(2)

        # Avoid touching the real serial device on this machine, and keep the
        # patch alive for the whole test so the proxy's run() finds it.
        with mock.patch.object(proxy, "open_serial", lambda: None):
            t = threading.Thread(target=proxy.run, daemon=True)
            t.start()
            time.sleep(0.2)

            try:
                # Direction 1: FC -> GCS. Fake the FC sending a HEARTBEAT frame.
                fake_heartbeat = (b"\xFD" + bytes(range(1, 24)))  # mav2 chunk
                proxy.serial.in_buffer = fake_heartbeat
                deadline = time.time() + 2
                got = None
                while time.time() < deadline and got is None:
                    try:
                        got, addr = gcs.recvfrom(65536)
                    except socket.timeout:
                        pass
                self.assertEqual(got, fake_heartbeat,
                                 "bytes from 'FC' must reach the GCS socket")
                self.assertEqual(addr, ("127.0.0.1", PI_PORT),
                                 "GCS reply must be addressable at proxy port")

                # Direction 2: GCS -> FC. Send a command from the laptop side.
                fake_command = (b"\xFD" + bytes(range(24, 60)))
                gcs.sendto(fake_command, ("127.0.0.1", PI_PORT))
                deadline = time.time() + 2
                while time.time() < deadline and \
                        proxy.serial.out_bytes != fake_command:
                    time.sleep(0.02)
                self.assertEqual(proxy.serial.out_bytes, fake_command,
                                 "bytes from GCS must reach the FC serial")
            finally:
                proxy.running = False
                gcs.close()


if __name__ == "__main__":
    unittest.main()