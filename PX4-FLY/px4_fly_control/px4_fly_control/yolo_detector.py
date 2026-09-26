#!/usr/bin/env python3
"""Optional timestamped YOLO adapter; supply balloon-specific weights."""

from .detector_base import LatestFrameDetector, run_detector


class YoloDetector(LatestFrameDetector):
    def __init__(self):
        super().__init__("yolo_detector")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("confidence", 0.55)
        self.declare_parameter("image_size", 416)
        self.model = None
        try:
            from ultralytics import YOLO

            self.model = YOLO(str(self.get_parameter("model").value))
            self.model.fuse()
        except Exception as exc:
            self.get_logger().error(
                f"Unable to load YOLO; install ultralytics and set model:=...: {exc}"
            )

    def detect(self, frame):
        if self.model is None:
            return
        result = self.model.predict(
            frame,
            conf=float(self.get_parameter("confidence").value),
            imgsz=int(self.get_parameter("image_size").value),
            max_det=3,
            verbose=False,
        )[0]
        if result.boxes is None or len(result.boxes) == 0:
            return
        best = max(result.boxes, key=lambda box: float(box.conf[0]))
        x1, y1, x2, y2 = [float(v) for v in best.xyxy[0]]
        height, width = frame.shape[:2]
        return (
            (x1 + x2) / 2 / width,
            (y1 + y2) / 2 / height,
            (x2 - x1) / width,
            (y2 - y1) / height,
            float(best.conf[0]),
        )


def main(args=None):
    run_detector(YoloDetector, args)


if __name__ == "__main__":
    main()
