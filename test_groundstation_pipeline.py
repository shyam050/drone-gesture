"""Integration-ish smoke test: receive video, run detection+classify on real
received frames, feed the gesture pipeline. No GUI, no MAVLink (uses FakeConnection).

Usage: python test_groundstation_pipeline.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

import numpy as np

from gesture_recognizer import GestureRecognizer
from ground_station import GesturePipeline
from test_helpers import FakeConnection
from video_receiver import VideoReceiver
from video_streamer import VideoStreamer


def main():
    recv = VideoReceiver()
    recv.start_thread()
    streamer = VideoStreamer(ip="127.0.0.1")
    streamer.open_camera()

    recognizer = GestureRecognizer()
    pipeline = GesturePipeline()
    fake = FakeConnection()

    # Send ~60 frames; on each received frame run detect+classify+pipeline and
    # assert pipeline accepts the result without error.
    imported = 0
    for i in range(60):
        ok, frame = streamer.cap.read()
        if not ok:
            break
        jpeg = streamer._encode(frame)
        streamer._send(jpeg)
        time.sleep(0.02)

        frame2 = recv.peek_frame()
        if frame2 is None:
            continue
        imported += 1
        lm, _ = recognizer.detect(frame2)
        gesture = recognizer.classify(lm) if lm is not None else "UNKNOWN"
        confirmed, smoothed = pipeline.push(gesture)
        if confirmed:
            print(f"  confirmed: {confirmed}")

    streamer.cap.release()
    recognizer.hands.close()
    print(f"Imported {imported} frames; pipeline ran without error")
    sys.exit(0)


if __name__ == "__main__":
    main()