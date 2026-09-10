"""MediaPipe-based hand landmark detection and 9-gesture classifier.

The classifier maps 21 hand landmarks to one of the 9 control gestures:

    OPEN_PALM, FIST, THUMB_UP, THUMB_DOWN, POINT_UP/DOWN/LEFT/RIGHT,
    PEACE, ROCK_ON

Finger logic follows the spec:
  - Thumb is "extended" when the tip is farther from the wrist than the
    thumb IP joint is.
  - Index/Middle/Ring/Pinky are "extended" when their second bone
    (PIP->TIP) continues in the direction of their first bone (MCP->PIP).
    This is direction-agnostic, so fingers pointing up/down/left/right are
    all handled, and it is stable under landmark jitter.
"""

import math

import cv2
import mediapipe as mp

from config import (
    GESTURES,
    HAND_DETECTION_CONFIDENCE,
    HAND_TRACKING_CONFIDENCE,
)


def _distance(a, b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5


def _finger_extended(hand, mcp, pip, tip):
    """Return True if a finger is (mostly) straight, regardless of the
    direction it points.

    A straight finger continues along its first bone: the vector PIP->TIP
    points the same way as MCP->PIP, so their normalized dot product (cos
    of the angle between them) is large. A curled finger folds the tip back
    toward the palm and the vectors diverge (cos small or negative).

    Threshold 0.3 -> accept angles up to ~72 degrees, which absorbs
    MediaPipe landmark noise without losing real curled fingers.
    """
    ax = pip.x - mcp.x
    ay = pip.y - mcp.y
    bx = tip.x - pip.x
    by = tip.y - pip.y
    la = math.hypot(ax, ay)
    lb = math.hypot(bx, by)
    if la < 1e-6 or lb < 1e-6:
        return False
    cos = (ax * bx + ay * by) / (la * lb)
    return cos > 0.3


class GestureRecognizer:
    """Wraps MediaPipe Hands and turns landmarks into gesture names."""

    # MediaPipe landmark indices.
    WRIST = 0
    THUMB_IP = 3
    THUMB_TIP = 4
    INDEX_MCP = 5
    INDEX_PIP = 6
    INDEX_TIP = 8
    MIDDLE_MCP = 9
    MIDDLE_PIP = 10
    MIDDLE_TIP = 12
    RING_MCP = 13
    RING_PIP = 14
    RING_TIP = 16
    PINKY_MCP = 17
    PINKY_PIP = 18
    PINKY_TIP = 20

    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=HAND_DETECTION_CONFIDENCE,
            min_tracking_confidence=HAND_TRACKING_CONFIDENCE,
        )
        self.mp_draw = mp.solutions.drawing_utils

    def detect(self, frame):
        """Detect one hand and return (landmarks, image_with_overlay).

        `landmarks` is a list of 21 normalized landmark objects or None.
        """
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.hands.process(rgb)
        rgb.flags.writeable = True

        landmarks = None
        if results.multi_hand_landmarks and results.multi_hand_landmarks:
            landmarks = results.multi_hand_landmarks[0].landmark

        if landmarks:
            self.mp_draw.draw_landmarks(
                frame,
                results.multi_hand_landmarks[0],
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_draw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
            )

        return landmarks, frame

    def classify(self, hand):
        """Classify a hand's 21 landmarks into a gesture name (str)."""
        thumb = self._thumb_extended(hand)
        index = _finger_extended(hand, hand[self.INDEX_MCP],
                                 hand[self.INDEX_PIP], hand[self.INDEX_TIP])
        middle = _finger_extended(hand, hand[self.MIDDLE_MCP],
                                  hand[self.MIDDLE_PIP], hand[self.MIDDLE_TIP])
        ring = _finger_extended(hand, hand[self.RING_MCP],
                                hand[self.RING_PIP], hand[self.RING_TIP])
        pinky = _finger_extended(hand, hand[self.PINKY_MCP],
                                 hand[self.PINKY_PIP], hand[self.PINKY_TIP])

        pattern = [int(thumb), int(index), int(middle), int(ring), int(pinky)]

        family = self._match_family(pattern)
        if family is None:
            return "UNKNOWN"

        if family in ("POINT", "THUMB"):
            return self._disambiguate_direction(family, hand)
        return family

    def _thumb_extended(self, hand):
        """Thumb is extended if its tip is farther from the wrist than its
        thumb IP joint is."""
        return _distance(hand[self.THUMB_TIP], hand[self.WRIST]) > _distance(
            hand[self.THUMB_IP], hand[self.WRIST]
        )

    def _match_family(self, pattern):
        """Return the gesture *family* for a finger pattern, or None.

        THUMB_UP/THUMB_DOWN share one pattern and the four POINT gestures share
        another, so ambiguous families are resolved by direction afterwards.
        """
        for gesture, pat in GESTURES.items():
            if pat != pattern:
                continue
            if gesture.startswith("POINT"):
                return "POINT"
            if gesture.startswith("THUMB"):
                return "THUMB"
            return gesture
        return None

    def _disambiguate_direction(self, family, hand):
        """Resolve a THUMB or POINT family into a specific named gesture."""
        if family == "THUMB":
            tip, ip = hand[self.THUMB_TIP], hand[self.THUMB_IP]
            return "THUMB_UP" if tip.y < ip.y else "THUMB_DOWN"

        tip, pip = hand[self.INDEX_TIP], hand[self.INDEX_PIP]
        dx = tip.x - pip.x
        dy = tip.y - pip.y
        if abs(dy) > abs(dx):
            return "POINT_UP" if dy < 0 else "POINT_DOWN"
        return "POINT_LEFT" if dx < 0 else "POINT_RIGHT"
