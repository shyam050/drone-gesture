"""Synthetic test for the GestureRecognizer classifier.

Builds fake 21-point hand landmark objects with controlled finger postures and
asserts the classifier returns the expected gesture. No camera / GPU needed.

Usage: python test_gesture_recognizer.py
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))


class _LM:
    """Minimal stand-in for a MediaPipe landmark (has .x/.y/.z)."""
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


def make_hand(finger_extended):
    """Build a generic hand where each of 5 fingers is straight/curled.

    finger_extended = [thumb, index, middle, ring, pinky] in {0,1}.
    Thumb uses the wrist-distance rule; other fingers use the bone-direction
    rule (TIP continues the MCP->PIP direction).
    Geometry:
      wrist = (0.5, 0.9), thumb IP = (0.5, 0.86)
      extended thumb tip = (0.75, 0.45)  -> far from wrist
      curled  thumb tip = (0.5, 0.72)    -> close to wrist
      non-thumb straight: MCP->PIP->TIP all point up (negative y)
      non-thumb curled : TIP folds back (positive y), opposite the bone
    """
    lm = [_LM(0.5, 0.9)]                     # wrist (index 0)
    # placeholders for the remaining 20 slots, then override the joints
    for _ in range(20):
        lm.append(_LM(0.5, 0.5))

    # Thumb.
    lm[3] = _LM(0.5, 0.86)                   # thumb IP
    lm[4] = _LM(0.75, 0.45) if finger_extended[0] else _LM(0.5, 0.88)

    # Non-thumb fingers: tip.y < pip.y => extended (upward).
    # helper map: name -> (mcp, pip, tip_index)
    # wrist at (0.5, 0.9); MCPs sit just above the palm, PIP above that,
    # straight tips high above, curled tips below the PIP.
    fingers = {"index": (5, 6, 8), "middle": (9, 10, 12),
               "ring": (13, 14, 16), "pinky": (17, 18, 20)}
    for i, (_, (mcp, pip, tip)) in enumerate(fingers.items(), start=1):
        x = 0.30 + 0.08 * i
        ext = finger_extended[i]
        lm[mcp] = _LM(x, 0.7)
        lm[pip] = _LM(x, 0.5)
        # straight: tip far above PIP; curled: tip near/below PIP
        lm[tip] = _LM(x, 0.2 if ext else 0.55)

    return lm


def set_thumb_direction(lm, up):
    """After building a THUMB hand, set thumb tip above/below the IP to pick
    up vs down. Keep the thumb extended (far from wrist) in both cases."""
    lm[3] = _LM(0.5, 0.86)
    # next line keeps dist(tip,wrist) > dist(ip,wrist) in both cases
    lm[4] = _LM(0.75, 0.1) if up else _LM(0.75, 0.9)
    return lm


def test():
    from gesture_recognizer import GestureRecognizer
    r = GestureRecognizer()

    cases = {
        "OPEN_PALM": [1, 1, 1, 1, 1],
        "FIST":      [0, 0, 0, 0, 0],
        "PEACE":     [0, 1, 1, 0, 0],
        "ROCK_ON":   [0, 1, 0, 0, 1],
    }
    ok = True
    for expected, pat in cases.items():
        got = r.classify(make_hand(pat))
        status = "PASS" if got == expected else "FAIL"
        if got != expected:
            ok = False
        print(f"{status}: {expected:10s} -> {got}")

    # Thumb up/down
    for up, expected in ((True, "THUMB_UP"), (False, "THUMB_DOWN")):
        hand = set_thumb_direction(make_hand([1, 0, 0, 0, 0]), up)
        got = r.classify(hand)
        status = "PASS" if got == expected else "FAIL"
        if got != expected:
            ok = False
        print(f"{status}: {expected:10s} -> {got}")

    # Point directions (index extended, others curled).
    # The index chain (MCP->PIP->TIP) is placed as a straight finger along
    # the pointing direction, which is how a real hand looks.
    idx_pat = [0, 1, 0, 0, 0]
    # (mcp, pip, tip) for index landmarks along each direction
    chains = {
        "POINT_UP":    ((0.45, 0.75), (0.45, 0.55), (0.45, 0.25)),
        "POINT_DOWN":  ((0.45, 0.25), (0.45, 0.45), (0.45, 0.75)),
        "POINT_LEFT":  ((0.65, 0.50), (0.50, 0.50), (0.25, 0.50)),
        "POINT_RIGHT": ((0.35, 0.50), (0.50, 0.50), (0.75, 0.50)),
    }
    for expected, (mcp, pip, tip) in chains.items():
        hand = make_hand(idx_pat)
        hand[5].x, hand[5].y = mcp
        hand[6].x, hand[6].y = pip
        hand[8].x, hand[8].y = tip
        got = r.classify(hand)
        status = "PASS" if got == expected else "FAIL"
        if got != expected:
            ok = False
        print(f"{status}: {expected:10s} -> {got}")

    # Unknown (e.g. only two fingers spread wrong) should be safe
    hand = make_hand([1, 1, 1, 0, 1])
    got = r.classify(hand)
    print(f"INFO:  {'MIXED':10s} -> {got} (should not be a control gesture)")

    print("\n" + ("ALL PASS" if ok else "SOME FAILED"))
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    test()
