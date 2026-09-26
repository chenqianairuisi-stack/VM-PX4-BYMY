#!/usr/bin/env python3
"""Timestamped HSV observations of the simulated red balloon."""

import cv2
from .detector_base import LatestFrameDetector, run_detector


class RedColorDetector(LatestFrameDetector):
    def __init__(self):
        super().__init__("red_color_detector")
        self.declare_parameter("min_area", 40.0)

    def detect(self, frame):
        if frame.shape[1] > 480:
            scale = 480 / frame.shape[1]
            frame = cv2.resize(
                frame,
                (480, max(1, int(frame.shape[0] * scale))),
                interpolation=cv2.INTER_AREA,
            )
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 100, 80), (12, 255, 255))
        mask |= cv2.inRange(hsv, (168, 100, 80), (179, 255, 255))
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        )
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < float(self.get_parameter("min_area").value):
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if min(w, h) / max(w, h) < 0.45:
                continue
            candidates.append((area, x, y, w, h))
        if not candidates:
            return None
        area, x, y, w, h = max(candidates)
        height, width = frame.shape[:2]
        return (
            (x + w / 2) / width,
            (y + h / 2) / height,
            w / width,
            h / height,
            min(1.0, area / (w * h)),
        )


def main(args=None):
    run_detector(RedColorDetector, args)


if __name__ == "__main__":
    main()
