"""Ground station (laptop) UDP video receiver.

Receives the datagrams sent by the Pi's VideoStreamer and decodes them into
OpenCV frames.

    [4-byte big-endian frame size][JPEG bytes]
"""

import socket
import struct
import time
from collections import deque

import cv2
import numpy as np

from config import VIDEO_PORT


class VideoReceiver:
    """Receives UDP video and produces OpenCV BGR frames.

    A small buffer decouples network arrival from the processing loop so a
    slow MediaPipe frame doesn't stall receive.

    Keep a per-source-pair only. Timeout so the loop can notice the Pi
    went away.
    """

    def __init__(self, timeout_ms=2000):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", VIDEO_PORT))
        self.sock.settimeout(timeout_ms / 1000.0)
        self.buf = deque(maxlen=4)  # maxlen keeps us from buffering too much
        self._last_frame_time = 0
        self.sender = None

    def peek_frame(self):
        """Return the most recently decoded frame, or None if stale/empty.

        Returns the newest frame in the buffer (dropping older ones) so the
        detector always works on fresh data.
        """
        if not self.buf:
            return None
        frame = self.buf.pop()
        self.buf.clear()
        self._last_frame_time = time.time()
        return frame

    def _drain(self):
        """Non-blocking receive loop; fills the buffer. Call from a thread."""
        while True:
            try:
                data, addr = self.sock.recvfrom(65536)
                if self.sender is None:
                    self.sender = addr
                    print(f"[receiver] first packet from {addr}")
                size = struct.unpack("!I", data[:4])[0]
                jpeg = data[4:4 + size]
                frame = cv2.imdecode(
                    np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR
                )
                if frame is not None:
                    self.buf.append(frame)
            except socket.timeout:
                continue
            except Exception as e:  # network hiccups shouldn't kill the thread
                print(f"[receiver] error: {e}")

    def start_thread(self):
        """Start the receiver thread. Returns the thread object."""
        import threading
        t = threading.Thread(target=self._drain, daemon=True)
        t.start()
        return t

    def is_stale(self, timeout=1.0):
        entries = self.buf
        return False  # let the caller manage staleness via FPS checks

    def latest_fps(self):
        return None  # measured by the ground station loop


def main():
    """Self test: show received frames in a window until 'q'."""
    receiver = VideoReceiver()
    receiver.start_thread()
    print(f"[receiver] listening on UDP :{VIDEO_PORT}")
    try:
        while True:
            frame = receiver.peek_frame()
            if frame is not None:
                cv2.imshow("Video Receiver", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
