"""Ground station (laptop) main application.

Integrates video -> hand landmark detection -> gesture classification
-> smoothing -> hold-to-confirm -> MAVLink command, and shows a live overlay
with detected gesture and telemetry hints.

Video sources:
  * default   : UDP stream from the Pi (production).
  * --webcam  : local webcam, so you can train/test on the laptop with
                no drone at all.

Run:
  laptop (with drone):  python src/ground_station.py
  laptop (webcam sim):  python src/ground_station.py --webcam --no-mavlink
"""

import argparse
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np

from config import (
    COMMAND_COOLDOWN_SEC,
    FRAME_HEIGHT,
    FRAME_WIDTH,
    GESTURE_NAMES,
    HOLD_TO_CONFIRM_SEC,
    MAVLINK_PORT,
    SMOOTHING_MIN_VOTES,
    SMOOTHING_WINDOW,
)
from gesture_recognizer import _finger_extended, GestureRecognizer
from mavlink_controller import MavlinkController
from video_receiver import VideoReceiver

WINDOW_NAME = "Drone Gesture Control"


class WebcamSource:
    """Local webcam video source, used only for laptop-side testing."""

    def __init__(self, index=0, width=FRAME_WIDTH, height=FRAME_HEIGHT):
        self.cap = None
        if sys.platform == "win32":
            self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(index)
        if self.cap is None or not self.cap.isOpened():
            print(f"[ground] WARNING: webcam {index} is not available")
            self.cap = None
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        print(f"[ground] using local webcam (index {index}) as video source")

    def is_open(self):
        return self.cap is not None

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self):
        if self.cap:
            self.cap.release()
            self.cap = None


class GesturePipeline:
    """Smoothing + hold-to-confirm + cooldown state machine."""

    def __init__(self, window=SMOOTHING_WINDOW, min_votes=SMOOTHING_MIN_VOTES,
                 hold=HOLD_TO_CONFIRM_SEC, cooldown=COMMAND_COOLDOWN_SEC,
                 now=None):
        self.window = window
        self.min_votes = min_votes
        self.hold = hold
        self.cooldown = cooldown
        self._now = now or time.time
        self.history = deque(maxlen=window)
        self.held_gesture = None
        self.hold_started = None
        self.last_command = None
        self.last_command_time = 0

    def push(self, gesture):
        """Feed one classified gesture; return (confirmed_gesture_or_None,
        smoothed_gesture)."""
        t = self._now()
        self.history.append(gesture)
        smoothed = self._smoothed()
        confirmed = None

        # The confirm condition: the *majority* of the window must currently
        # hold `smoothed` AND it must have been held for `hold` seconds AND we
        # are past the `cooldown` since the last confirmed command.
        if smoothed is not None:
            if smoothed != self.held_gesture:
                self.held_gesture = smoothed
                self.hold_started = t
            elif (self._majority_is(smoothed) and
                  (t - self.hold_started) >= self.hold and
                  (t - self.last_command_time) >= self.cooldown):
                confirmed = smoothed
                # Keep held_gesture so a longer hold doesn't re-fire; the
                # cooldown governs repeats. Reset the hold timer anyway.
                self.hold_started = t
                self.last_command = smoothed
                self.last_command_time = t
        else:
            # No reliable gesture in view: forget the hold timer.
            self.held_gesture = None
            self.hold_started = None

        return confirmed, smoothed

    def _majority_is(self, gesture):
        counted = sum(1 for g in self.history if g == gesture)
        return counted >= self.min_votes

    def _smoothed(self):
        counts = {}
        for g in self.history:
            counts[g] = counts.get(g, 0) + 1
        best, n = max(counts.items(), key=lambda kv: kv[1])
        if n >= self.min_votes and best != "UNKNOWN":
            return best
        return None


class GroundStationApp:
    def __init__(self, webcam=False, no_mavlink=False, camera_index=0,
                 fast=False):
        self.receiver = VideoReceiver()
        self.webcam = WebcamSource(camera_index) if webcam else None
        self.recognizer = GestureRecognizer()
        if fast:
            self.pipeline = GesturePipeline(window=3, min_votes=1,
                                            hold=0.1, cooldown=0.1)
        else:
            self.pipeline = GesturePipeline()
        self.mav = MavlinkController()
        self.connected = False
        self.connect_failed = False
        self.no_mavlink = no_mavlink
        self.webcam_mode = webcam
        self.fps = 0.0
        self._fps_window = deque(maxlen=30)

    def connect(self):
        """Connect to the drone over UDP in a background thread so the GUI
        starts immediately even before a heartbeat arrives."""
        if self.no_mavlink:
            print("[ground] MAVLink disabled (--no-mavlink)")
            self.connect_failed = True
            return
        threading.Thread(target=self._connect_worker, daemon=True).start()

    def _connect_worker(self):
        try:
            self.mav.connect_udp(MAVLINK_PORT)
            self.connected = True
            self.connect_failed = False
            print("[ground] MAVLink connected")
        except Exception as e:
            self.connect_failed = True
            print(f"[ground] MAVLink connect failed (continuing without): {e}")

    # ------------------------------------------------------------------ #
    # Video source abstraction
    # ------------------------------------------------------------------ #
    def _next_frame(self):
        """Pull one frame from the active source or None."""
        if self.webcam_mode:
            return self.webcam.read()
        return self.receiver.peek_frame()

    @staticmethod
    def _placeholder():
        return np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), np.uint8)

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    def run(self):
        self.receiver.start_thread()
        self.connect()
        cv2.namedWindow(WINDOW_NAME)
        print("[ground] gesture control live. 'q' quits, 'H' hovers, ESC lands.")

        try:
            while True:
                frame = self._next_frame()
                confirmed = None
                pattern_str = ""

                if frame is None:
                    frame = self._placeholder()
                    smoothed = "WAITING_VIDEO"
                else:
                    t0 = time.time()
                    landmarks, frame = self.recognizer.detect(frame)
                    if landmarks is not None:
                        gesture = self.recognizer.classify(landmarks)
                        confirmed, smoothed = self.pipeline.push(gesture)
                        # Show the raw finger pattern for debugging
                        thumb = self.recognizer._thumb_extended(landmarks)
                        idx = _finger_extended(
                            landmarks, landmarks[5], landmarks[6], landmarks[8])
                        mid = _finger_extended(
                            landmarks, landmarks[9], landmarks[10], landmarks[12])
                        rng = _finger_extended(
                            landmarks, landmarks[13], landmarks[14], landmarks[16])
                        pnk = _finger_extended(
                            landmarks, landmarks[17], landmarks[18], landmarks[20])
                        pattern_str = (f"[T{int(thumb)} I{int(idx)} "
                                       f"M{int(mid)} R{int(rng)} P{int(pnk)}]")
                        if confirmed and self.connected:
                            self.mav.execute(confirmed)
                    else:
                        smoothed = "NO_HAND"
                        self.pipeline.push("UNKNOWN")
                    self._tick_fps(t0)

                self._overlay(frame, smoothed, confirmed, pattern_str)
                cv2.imshow(WINDOW_NAME, frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("h") and self.connected:
                    self.mav.hover()
                elif key == 27 and self.connected:
                    self.mav.land()
        finally:
            if self.webcam:
                self.webcam.release()
            cv2.destroyAllWindows()

    def _tick_fps(self, t0):
        dt = time.time() - t0
        if dt > 0:
            self._fps_window.append(1.0 / dt)
            self.fps = sum(self._fps_window) / len(self._fps_window)

    def _overlay(self, frame, smoothed, confirmed, pattern_str=""):
        h = frame.shape[0]

        if smoothed == "WAITING_VIDEO":
            cv2.putText(frame, "WAITING FOR VIDEO...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
            if self.webcam_mode:
                tell = "webcam source" if self.webcam.is_open() \
                    else "webcam not available (check index)"
                cv2.putText(frame, tell, (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 1)
        else:
            if smoothed:
                name = GESTURE_NAMES.get(smoothed, smoothed)
                color = (0, 255, 0) if smoothed in GESTURE_NAMES \
                    else (0, 165, 255)
                cv2.putText(frame, f"Gesture: {name}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            if confirmed:
                cv2.putText(frame, f">> SENDING: {confirmed}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            if pattern_str:
                # Debug line: raw finger pattern T I M R P (1 = extended)
                cv2.putText(frame, "Pattern " + pattern_str, (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        if self.no_mavlink:
            status = "MAVLINK DISABLED"
        elif self.connected:
            status = "CONNECTED"
        elif self.connect_failed:
            status = "NO MAVLINK"
        else:
            status = "CONNECTING..."
        cv2.putText(frame, f"{status}  {self.fps:.1f} FPS", (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main():
    parser = argparse.ArgumentParser(
        description="Drone gesture ground station",
        epilog="examples:\n"
               "  production (Pi streams video):  python src/ground_station.py\n"
               "  laptop webcam test:              python src/ground_station.py "
               "--webcam --no-mavlink",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--webcam", "-w", action="store_true",
                        help="use the local webcam as video source "
                             "(laptop testing, no Pi needed)")
    parser.add_argument("--camera", type=int, default=0,
                        help="webcam index to use with --webcam (default 0)")
    parser.add_argument("--no-mavlink", action="store_true",
                        help="skip the MAVLink connection (pure gesture demo)")
    parser.add_argument("--fast", action="store_true",
                        help="fast response mode for testing (0.1s hold, "
                             "1-vote smoothing)")
    args = parser.parse_args()

    GroundStationApp(webcam=args.webcam, no_mavlink=args.no_mavlink,
                     camera_index=args.camera, fast=args.fast).run()


if __name__ == "__main__":
    main()
