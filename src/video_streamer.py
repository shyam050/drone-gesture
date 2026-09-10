"""Onboard (Pi) video capture + UDP streaming.

Captures frames from the Pi camera and streams them to the ground station as
UDP datagrams of the form:

    [4-byte big-endian frame size][JPEG bytes]

Run this on the Raspberry Pi.
"""

import socket
import struct
import time

import cv2

from config import (
    FRAME_HEIGHT,
    FRAME_WIDTH,
    FPS,
    GROUND_STATION_IP,
    JPEG_QUALITY,
    SIMULATED_SOURCE,
    VIDEO_PORT,
)


class VideoStreamer:
    """Captures camera frames and sends them as JPEG over UDP."""

    def __init__(self, ip=GROUND_STATION_IP, port=VIDEO_PORT, width=FRAME_WIDTH,
                 height=FRAME_HEIGHT, fps=FPS, quality=JPEG_QUALITY):
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
        self.dest = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cap = None
        self._jpeg_size_sum = 0
        self._jpeg_frame_count = 0

    def open_camera(self):
        """Open the local camera. Returns True on success."""
        if SIMULATED_SOURCE:
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        return self.cap.isOpened()

    def _encode(self, frame):
        result, jpeg = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        )
        return jpeg.tobytes()

    def _send(self, jpeg):
        # One datagram per frame: [size][data]. UDP max payload is ~65507
        # bytes, which a 70% quality 640x480 JPEG stays comfortably under.
        self.sock.sendto(struct.pack("!I", len(jpeg)) + jpeg, self.dest)

    def start(self, display=False):
        """Stream until interrupted. Returns the average packet size."""
        if not self.open_camera():
            print("ERROR: could not open camera")
            return

        print(f"[streamer] Streaming {self.width}x{self.height} @ {self.fps}fps "
              f"to {self.dest[0]}:{self.dest[1]} (quality {self.quality})")

        frame_time = 1.0 / self.fps
        try:
            while True:
                ok, frame = self.cap.read()
                if not ok:
                    print("[streamer] read() returned None")
                    break

                jpeg = self._encode(frame)
                self._send(jpeg)
                self._jpeg_size_sum += len(jpeg)
                self._jpeg_frame_count += 1

                if display:
                    cv2.imshow("Pi Camera", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                time.sleep(frame_time)
        except KeyboardInterrupt:
            print("\n[streamer] stopped by user")
        finally:
            self.cap.release()
            self.sock.close()
            avg = (self._jpeg_size_sum / self._jpeg_frame_count
                   if self._jpeg_frame_count else 0)
            print(f"[streamer] sent {self._jpeg_frame_count} frames, "
                  f"avg {avg:.0f} bytes/frame")


def main():
    streamer = VideoStreamer()
    streamer.start(display=True)


if __name__ == "__main__":
    main()
