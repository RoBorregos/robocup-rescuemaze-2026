#!/usr/bin/env python3
from __future__ import annotations

import cv2

from vision_detector import VisionDetector
from vision_protocol import CAM_LEFT, CAM_RIGHT, victim_name


def run_preview() -> None:
    detector = VisionDetector()
    try:
        print("Press q to quit preview")
        while True:
            victim_right = detector.detect_victim(CAM_RIGHT)
            victim_left = detector.detect_victim(CAM_LEFT)

            ok_r, frame_r = detector.cap_right.read() if detector.cap_right is not None else (False, None)
            ok_l, frame_l = detector.cap_left.read() if detector.cap_left is not None else (False, None)

            if ok_r and frame_r is not None:
                text = f"RIGHT: {victim_name(victim_right)}"
                cv2.putText(frame_r, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.imshow("RIGHT", frame_r)

            if ok_l and frame_l is not None:
                text = f"LEFT: {victim_name(victim_left)}"
                cv2.putText(frame_l, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.imshow("LEFT", frame_l)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        detector.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run_preview()
