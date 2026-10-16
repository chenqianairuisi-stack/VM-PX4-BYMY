import numpy as np
import pytest
from px4_fly_control.drone_onnx import letterbox, best_box, TemporalCropDetector


def test_letterbox_and_inverse_preserve_camera_coordinates():
    frame = np.zeros((360,480,3), np.uint8)
    tensor, scale, left, top = letterbox(frame,416)
    assert tensor.shape == (1,3,416,416)
    prediction = np.array([[[208], [208], [83.2], [31.2], [.8]]])
    box = best_box(prediction, frame.shape, scale, left, top, .55)
    np.testing.assert_allclose(box, [.5,.5,.2,.1,.8])


def test_misses_invalid_boxes_and_wrong_classes_are_rejected():
    shape = (360,480,3)
    assert best_box(np.array([[[208],[208],[10],[10],[.1]]]),shape,1,0,0,.55) is None
    assert best_box(np.array([[[np.nan],[208],[10],[10],[.9]]]),shape,1,0,0,.55) is None
    with pytest.raises(ValueError):
        best_box(np.zeros((1,84,100)),shape,1,0,0,.55)


def test_highest_confidence_and_clipping():
    boxes = np.array([[[0,240],[180,180],[40,80],[40,80],[.95,.8]]])
    result = best_box(boxes, (360,480,3),1,0,0,.55)
    assert result[0] - result[2]/2 == pytest.approx(0)
    assert result[4] == pytest.approx(.95)


class InferenceSequence:
    def __init__(self, *outputs):
        self.outputs = iter(outputs)
        self.shapes = []

    def __call__(self, frame):
        self.shapes.append(frame.shape[:2])
        return next(self.outputs)


SMALL = (.5, .5, .04, .02, .8)


def test_temporal_crop_maps_center_and_span_to_full_camera():
    infer = InferenceSequence(SMALL, (.6, .4, .12, .045, .7))
    detector = TemporalCropDetector(infer)
    frame = np.zeros((360, 480, 3), np.uint8)
    detector.detect(frame, 1.)
    box = detector.detect(frame, 1.1)
    assert infer.shapes == [(360, 480), (160, 160)]
    np.testing.assert_allclose(box, [256/480, 164/360, .04, .02, .7])


@pytest.mark.parametrize('candidate', [None, (.025, .5, .05, .02, .9),
                                      (.5, .025, .05, .05, .9),
                                      (.975, .5, .05, .02, .9),
                                      (.5, .975, .05, .05, .9),
                                      (.9, .9, .1, .1, .9)])
def test_crop_miss_artificial_edge_or_unassociated_box_falls_back(candidate):
    infer = InferenceSequence(SMALL, candidate, SMALL)
    detector = TemporalCropDetector(infer)
    frame = np.zeros((360, 480, 3), np.uint8)
    detector.detect(frame, 1.)
    assert detector.detect(frame, 1.1) == SMALL
    assert infer.shapes == [(360, 480), (160, 160), (360, 480)]


def test_real_image_edge_survives_crop_mapping_for_bearing_only_update():
    initial = (.02, .5, .04, .02, .8)
    infer = InferenceSequence(initial, (.03, .5, .06, .045, .8))
    detector = TemporalCropDetector(infer)
    frame = np.zeros((360, 480, 3), np.uint8)
    detector.detect(frame, 1.)
    box = detector.detect(frame, 1.1)
    assert len(infer.shapes) == 2
    assert box[0] - box[2]/2 == pytest.approx(0)
    assert box[2] == pytest.approx(.02)


def test_periodic_full_frame_and_expired_track_do_not_keep_cropping():
    infer = InferenceSequence(SMALL, (.5, .5, .12, .045, .8), SMALL, SMALL)
    detector = TemporalCropDetector(infer, full_period=3)
    frame = np.zeros((360, 480, 3), np.uint8)
    for stamp in (1., 1.1, 1.2, 2.):
        detector.detect(frame, stamp)
    assert infer.shapes == [(360, 480), (160, 160), (360, 480), (360, 480)]
    np.testing.assert_allclose(detector.velocity, 0)


def test_low_confidence_can_continue_but_cannot_seed_or_move_crop_to_distractor():
    low = (.5, .5, .04, .02, .4)
    unrelated = (.9, .5, .04, .02, .4)
    infer = InferenceSequence(low, SMALL, (.5, .5, .12, .045, .4), unrelated)
    detector = TemporalCropDetector(infer, full_period=2)
    frame = np.zeros((360, 480, 3), np.uint8)
    detector.detect(frame, 1.)
    assert detector.box is None
    detector.detect(frame, 1.1)
    box = detector.detect(frame, 1.2)
    assert box[4] == .4 and detector.stamp == 1.2
    detector.detect(frame, 1.3)
    assert detector.stamp == 1.2
    assert detector.box == box


@pytest.mark.parametrize('stamp,shape', [(1., (360, 480)), (.5, (360, 480)),
                                        (1.1, (480, 640))])
def test_duplicate_rewind_or_resolution_change_restarts_full_frame(stamp, shape):
    infer = InferenceSequence(SMALL, SMALL)
    detector = TemporalCropDetector(infer)
    detector.detect(np.zeros((360, 480, 3), np.uint8), 1.)
    detector.velocity[:] = 1.
    detector.detect(np.zeros((*shape, 3), np.uint8), stamp)
    assert infer.shapes == [(360, 480), shape]
    np.testing.assert_allclose(detector.velocity, 0)


def test_new_high_confidence_full_frame_track_does_not_inherit_velocity():
    other = (.9, .5, .04, .02, .9)
    infer = InferenceSequence(SMALL, other)
    detector = TemporalCropDetector(infer, full_period=2)
    frame = np.zeros((360, 480, 3), np.uint8)
    detector.detect(frame, 1.)
    detector.velocity[:] = .1
    detector.detect(frame, 1.1)
    assert detector.box == other
    np.testing.assert_allclose(detector.velocity, 0)
