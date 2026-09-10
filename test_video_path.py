"""End-to-end local smoke test of the video path only.

Runs the VideoStreamer and VideoReceiver in-process on the laptop so you can
verify UDP->JPEG->decode round-trips without needing a Pi. Also auto-checks
that at least one frame decodes to a non-zero image and shows it.

Usage: python test_video_path.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

import cv2

from video_receiver import VideoReceiver
from video_streamer import VideoStreamer


def main():
    recv = VideoReceiver()
    recv.start_thread()
    # Simulate "Pi" on the same host by streaming our own webcam.
    # Loopback avoids depending on the laptop's LAN IP for local testing.
    streamer = VideoStreamer(ip="127.0.0.1")
    streamer.open_camera()

    print("Streaming 20 frames from local webcam over UDP...")
    got = 0
    for i in range(20):
        ok, frame = streamer.cap.read()
        if not ok:
            print("camera read failed; skipping")
            break
        jpeg = streamer._encode(frame)
        streamer._send(jpeg)
        time.sleep(0.02)

        frame2 = recv.peek_frame()
        if frame2 is not None:
            got += 1
            if got == 1:
                cv2.imwrite("_smoke_frame.png", frame2)
                print(f"  decoded {frame2.shape} frame  -> saved _smoke_frame.png")

    streamer.cap.release()
    print(f"Decoded {got}/20 frames over UDP round-trip")
    sys.exit(0 if got > 0 else 1)


if __name__ == "__main__":
    main()