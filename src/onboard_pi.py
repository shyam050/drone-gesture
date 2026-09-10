"""Onboard (Raspberry Pi) main application.

Two jobs on the Pi:
  1. Stream video to the ground station (VideoStreamer).
  2. Proxy MAVLink between the FC's serial UART and the ground station
     (MavlinkProxy).

Run on the Pi:  python3 src/onboard_pi.py
"""

import signal
import threading

from mavlink_controller import MavlinkProxy
from video_streamer import VideoStreamer


class OnboardApp:
    def __init__(self):
        self.streamer = VideoStreamer()
        self.proxy = MavlinkProxy()
        self.running = True

    def run(self):
        signal.signal(signal.SIGINT, self._shutdown)
        signal.signal(signal.SIGTERM, self._shutdown)

        # Start MAVLink proxy in a background thread (serial + UDP).
        proxy_thread = threading.Thread(target=self.proxy.run, daemon=True)
        proxy_thread.start()

        # Stream video in the main thread.
        self.streamer.start(display=True)

    def _shutdown(self, *_):
        self.running = False
        self.proxy.running = False
        print("\n[onboard] shutting down")


def main():
    OnboardApp().run()


if __name__ == "__main__":
    main()
