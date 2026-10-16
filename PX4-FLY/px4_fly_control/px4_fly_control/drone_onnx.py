"""Single-class Ultralytics YOLOv8 ONNX inference without PyTorch in the VM."""
import ast
import cv2
import numpy as np


def letterbox(frame, size):
    height, width = frame.shape[:2]
    scale = min(size / width, size / height)
    scaled_w, scaled_h = round(width * scale), round(height * scale)
    left, top = (size - scaled_w) // 2, (size - scaled_h) // 2
    image = np.full((size, size, 3), 114, np.uint8)
    image[top:top+scaled_h, left:left+scaled_w] = cv2.resize(frame, (scaled_w, scaled_h))
    tensor = image[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return tensor, scale, left, top


def best_box(prediction, frame_shape, scale, left, top, confidence):
    prediction = np.asarray(prediction)
    if prediction.ndim != 3 or prediction.shape[0] != 1 or prediction.shape[1] != 5:
        raise ValueError('Expected single-class YOLOv8 output [1,5,N]')
    rows = prediction[0].T
    rows = rows[np.isfinite(rows).all(axis=1) & (rows[:, 4] >= confidence)]
    if not len(rows):
        return None
    # Only one target is consumed; NMS cannot remove the highest-score box.
    height, width = frame_shape[:2]
    for cx, cy, w, h, score in sorted(rows, key=lambda row: -row[4]):
        x1, x2 = np.clip([(cx-w/2-left)/scale, (cx+w/2-left)/scale], 0, width)
        y1, y2 = np.clip([(cy-h/2-top)/scale, (cy+h/2-top)/scale], 0, height)
        if x2 > x1 and y2 > y1:
            return ((x1+x2)/2/width, (y1+y2)/2/height,
                    (x2-x1)/width, (y2-y1)/height, float(score))
    return None


class DroneOnnx:
    def __init__(self, path):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        self.session = ort.InferenceSession(str(path), sess_options=options,
                                           providers=['CPUExecutionProvider'])
        metadata = self.session.get_modelmeta().custom_metadata_map
        names = ast.literal_eval(metadata.get('names', '{}'))
        if names not in ({0: 'drone'}, {'0': 'drone'}, ['drone']):
            raise ValueError('ONNX must contain exactly one class named drone')
        self.input = self.session.get_inputs()[0]
        shape = self.input.shape
        if len(shape) != 4 or shape[:2] != [1, 3] or not isinstance(shape[2], int) or shape[2] != shape[3]:
            raise ValueError('Export a static square YOLOv8 ONNX model')
        self.size = shape[2]

    def predict(self, frame, confidence):
        tensor, scale, left, top = letterbox(frame, self.size)
        output = self.session.run(None, {self.input.name: tensor})[0]
        return best_box(output, frame.shape, scale, left, top, confidence)


class TemporalCropDetector:
    """Magnify a recent small target, then map every box to the full image.

    Exposure timestamps drive the bounded crop prediction. Periodic full-frame
    inference and immediate fallback prevent the crop becoming a permanent
    search region. Artificial crop-edge boxes are never used to infer range.
    """
    def __init__(self, infer, min_pixels=160, max_age=.6, full_period=5):
        self.infer = infer
        self.min_pixels, self.max_age, self.full_period = min_pixels, max_age, full_period
        self.box = self.stamp = self.shape = None
        self.velocity = np.zeros(2)
        self.frames = 0

    def associated(self, box, stamp):
        if self.box is None:
            return False
        dt = stamp - self.stamp
        center = np.asarray(self.box[:2]) + self.velocity * min(dt, .3)
        tolerance = min(.18, max(.08, 2 * self.box[2]) + .25 * dt)
        return float(np.linalg.norm(np.asarray(box[:2]) - center)) <= tolerance

    def detect(self, frame, stamp):
        height, width = frame.shape[:2]
        if not np.isfinite(stamp) or stamp <= 0:
            return None
        if (self.shape != (height, width) or self.stamp is None
                or stamp <= self.stamp or stamp - self.stamp > self.max_age):
            self.box = self.stamp = None
            self.velocity[:] = 0
        self.shape = (height, width)
        self.frames += 1
        result = None
        if (self.box is not None and self.box[2] < .16
                and self.frames % self.full_period != 0):
            center = np.asarray(self.box[:2]) + self.velocity * min(stamp-self.stamp, .3)
            side = int(min(min(height, width), max(
                self.min_pixels, self.box[2]*width*6, self.box[3]*height*6
            )))
            x0 = int(np.clip(round(center[0]*width-side/2), 0, width-side))
            y0 = int(np.clip(round(center[1]*height-side/2), 0, height-side))
            candidate = self.infer(frame[y0:y0+side, x0:x0+side])
            if candidate is not None:
                cx, cy, bw, bh, confidence = candidate
                crop_edges = (
                    (cx-bw/2)*side <= 2 and x0 > 0,
                    (cy-bh/2)*side <= 2 and y0 > 0,
                    (cx+bw/2)*side >= side-2 and x0+side < width,
                    (cy+bh/2)*side >= side-2 and y0+side < height,
                )
                if not any(crop_edges):
                    mapped = ((x0+cx*side)/width, (y0+cy*side)/height,
                              bw*side/width, bh*side/height, confidence)
                    if self.associated(mapped, stamp):
                        result = mapped
        if result is None:
            result = self.infer(frame)
        if result is not None:
            associated = self.associated(result, stamp)
            if associated:
                dt = stamp-self.stamp
                if .02 <= dt <= self.max_age:
                    observed = (np.asarray(result[:2])-self.box[:2])/dt
                    observed *= min(1., 1.5/max(float(np.linalg.norm(observed)), 1e-9))
                    self.velocity = .6*self.velocity + .4*observed
            if associated or result[4] >= .55:
                if not associated:
                    # A full-frame reacquisition starts a new image track;
                    # do not carry the old target's image velocity into it.
                    self.velocity[:] = 0
                self.box, self.stamp = result, stamp
        return result
