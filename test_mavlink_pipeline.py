"""Tests for the gesture pipeline (smoothing / hold / cooldown) and the
MAVLink command dispatching."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))
sys.path.insert(0, os.path.dirname(__file__))

from ground_station import GesturePipeline
from mavlink_controller import MavlinkController
from test_helpers import FakeClock, FakeConnection


class TestPipeline(unittest.TestCase):
    def make_pipeline(self, hold=1.0, cooldown=0.3):
        self.clock = FakeClock(100.0)
        return GesturePipeline(window=5, min_votes=3, hold=hold,
                               cooldown=cooldown, now=self.clock)

    def feed(self, p, gesture, n=1, dt=1.0 / 30.0, hold_advance=0.0):
        """Feed `gesture` n times, advancing the clock dt per frame.
        Returns the list of confirmed commands."""
        confirmed = []
        for _ in range(n):
            if hold_advance > 0:
                self.clock.advance(hold_advance)
            else:
                self.clock.advance(dt)
            c, _ = p.push(gesture)
            if c:
                confirmed.append(c)
        return confirmed

    def test_three_frames_confirm_gesture(self):
        p = self.make_pipeline(hold=0.1, cooldown=0.0)
        # Over 30 frames at 33ms, hold (0.1) is exceeded almost immediately.
        confirmed = self.feed(p, "FIST", n=30)
        self.assertTrue(confirmed, "FIST should confirm after >hold seconds")
        self.assertEqual(confirmed[0], "FIST")

    def test_gesture_must_be_held_long_enough(self):
        p = self.make_pipeline(hold=2.0, cooldown=0.0)
        # 10 frames in 0.33s total; hold of 2s is NOT reached.
        confirmed = self.feed(p, "FIST", n=10)
        self.assertEqual(confirmed, [], "hold not long enough -> no confirm")

    def test_switch_gesture_resets_hold(self):
        p = self.make_pipeline(hold=0.1, cooldown=0.0)
        # Feed OPEN_PALM for a while.
        c1 = self.feed(p, "OPEN_PALM", n=15)
        self.assertTrue(c1 and c1[0] == "OPEN_PALM")
        # Immediately switch: FIST should need a fresh hold.
        self.clock.advance(0.0)  # no extra time
        confirmed = self.feed(p, "FIST", n=1)
        self.assertEqual(confirmed, [], "fresh gesture needs hold time")

    def test_cooldown_limits_repeats(self):
        p = self.make_pipeline(hold=0.05, cooldown=0.5)
        confirmed = self.feed(p, "ROCK_ON", n=10)
        # First confirm happens; repeats within cooldown are suppressed.
        self.assertEqual(len(confirmed), 1, "cooldown should limit repeats")

    def test_unknown_is_filtered(self):
        p = self.make_pipeline(hold=0.05, cooldown=0.0)
        confirmed = self.feed(p, "UNKNOWN", n=30)
        self.assertEqual(confirmed, [])


class TestMavlink(unittest.TestCase):
    def setUp(self):
        self.conn = FakeConnection()
        self.mvc = MavlinkController(endpoint="udpin:0.0.0.0:14550")
        self.mvc.conn = self.conn

    def test_hover_sends_zero_velocity(self):
        self.mvc.hover()
        self.assertEqual(len(self.conn.mav.velocity_calls), 1)
        args = self.conn.mav.velocity_calls[0][0]
        vx, vy, vz = args[8], args[9], args[10]
        self.assertEqual((vx, vy, vz), (0.0, 0.0, 0.0))

    def test_velocity_mask_is_body_ned(self):
        self.mvc.send_velocity(1.0, 0.0, 0.0)
        args = self.conn.mav.velocity_calls[0][0]
        frame = args[3]
        mask = args[4]
        self.assertEqual(frame, 8)  # MAV_FRAME_BODY_NED
        self.assertEqual(mask, 0b0000111111000111)

    def test_land_sets_land_mode(self):
        self.mvc.land()
        self.assertIn(9, self.conn.modes)  # LAND enum id

    def test_takeoff_arms_and_sets_guided(self):
        self.mvc.takeoff(altitude=1.5)
        self.assertTrue(self.conn.armed)
        self.assertIn(4, self.conn.modes)  # GUIDED
        # command_long_send was invoked with NAV_TAKEOFF
        cmds = self.conn.mav.command_calls
        self.assertTrue(cmds, "expected a COMMAND_LONG for takeoff")
        args = cmds[0][0]
        self.assertEqual(args[2], 22)  # MAV_CMD_NAV_TAKEOFF
        self.assertEqual(args[10], 1.5)  # altitude (param7)

    def test_execute_maps_rock_on_to_move(self):
        self.mvc.execute("ROCK_ON")
        args = self.conn.mav.velocity_calls[0][0]
        vx, vy, vz = args[8], args[9], args[10]
        self.assertEqual((vx, vy, vz), (0.0, 0.0, 0.3))


if __name__ == "__main__":
    unittest.main()