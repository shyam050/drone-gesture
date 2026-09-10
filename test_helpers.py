"""Shared test utilities: minimal mock for pymavlink and a fake landmark."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))


class _LM:
    """Stand-in for a MediaPipe landmark object."""
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class FakeClock:
    """Deterministic clock for testing the gesture pipeline."""
    def __init__(self, start=0.0):
        self.t = start

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def _recorder():
    """Return (wrapper_with_.calls, fn_list)."""
    calls = []

    def wrapper(*args, **_kwargs):
        calls.append((args, _kwargs))
        return None

    wrapper.calls = calls
    return wrapper, calls


class FakeMavlink:
    """Stub for the pymavlink `mav` object used by MavlinkController."""

    MAV_FRAME_BODY_NED = 8  # body frame enum value

    def __init__(self):
        self.set_position_target_local_ned_send, self.velocity_calls = _recorder()
        self.command_long_send, self.command_calls = _recorder()


class FakeConnection:
    """Mimics the minimal mavutil interface used by MavlinkController."""

    def __init__(self):
        self.mav = FakeMavlink()
        self.modes = []
        self.armed = False

    def mode_mapping(self):
        return {"GUIDED": 4, "LAND": 9, "LOITER": 5}

    def set_mode_apm(self, mode_id):
        self.modes.append(mode_id)

    def arducopter_arm(self):
        self.armed = True

    def motors_armed_wait(self):
        pass

    def wait_heartbeat(self, timeout=10):
        return None


def make_hand(finger_extended):
    """Same synthetic hand builder as the recognizer test."""
    lm = [_LM(0.5, 0.9)]
    for _ in range(20):
        lm.append(_LM(0.5, 0.5))
    lm[3] = _LM(0.5, 0.86)
    lm[4] = _LM(0.75, 0.45) if finger_extended[0] else _LM(0.5, 0.88)
    fingers = {"index": (5, 6, 8), "middle": (9, 10, 12),
               "ring": (13, 14, 16), "pinky": (17, 18, 20)}
    for i, (_, (mcp, pip, tip)) in enumerate(fingers.items(), start=1):
        x = 0.30 + 0.08 * i
        ext = finger_extended[i]
        lm[mcp] = _LM(x, 0.7)
        lm[pip] = _LM(x, 0.5)
        lm[tip] = _LM(x, 0.2 if ext else 0.55)
    return lm