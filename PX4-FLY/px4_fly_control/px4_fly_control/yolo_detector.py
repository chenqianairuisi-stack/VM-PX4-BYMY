"""Drone-only YOLO adapter; missing or generic COCO weights fail at startup."""
from pathlib import Path
from .detector_base import LatestFrameDetector, run_detector


class YoloDetector(LatestFrameDetector):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('model', '')
        # The controller keeps a higher acquisition threshold and only uses
        # lower-score boxes when associated with an existing predicted track.
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('image_size', 416)
        self.declare_parameter('device', 'cpu')
        path = Path(str(self.get_parameter('model').value)).expanduser()
        if not path.is_file():
            raise FileNotFoundError('Set model:=/absolute/path/to/trained/drone.onnx or drone.pt')
        self.onnx = path.suffix.lower() == '.onnx'
        if self.onnx:
            from .drone_onnx import DroneOnnx
            self.model = DroneOnnx(path)
        else:
            from ultralytics import YOLO
            self.model = YOLO(str(path))
            if self.model.names != {0: 'drone'}:
                raise ValueError('Expected trained single-class drone weights; generic COCO weights are not supported')
            self.model.fuse()
        self.get_logger().info(f'Drone weights loaded: {path}')
        from .drone_onnx import TemporalCropDetector
        self.crop_detector = TemporalCropDetector(self.detect)

    def detect_stamped(self, frame, stamp):
        return self.crop_detector.detect(frame, stamp)

    def detect(self, frame):
        confidence = float(self.get_parameter('confidence').value)
        if self.onnx:
            return self.model.predict(frame, confidence)
        result = self.model.predict(
            frame, conf=confidence, classes=[0],
            imgsz=int(self.get_parameter('image_size').value),
            device=str(self.get_parameter('device').value), max_det=3, verbose=False,
        )[0]
        if result.boxes is None or len(result.boxes) == 0:
            return None
        best = max(result.boxes, key=lambda box: float(box.conf[0]))
        x1, y1, x2, y2 = [float(v) for v in best.xyxy[0]]
        height, width = frame.shape[:2]
        return ((x1+x2)/2/width, (y1+y2)/2/height,
                (x2-x1)/width, (y2-y1)/height, float(best.conf[0]))


def main(args=None):
    run_detector(YoloDetector, args)


if __name__ == '__main__':
    main()
