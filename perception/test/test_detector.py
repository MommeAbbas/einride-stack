import numpy as np
import pytest
from perception.detector import letterbox, boxes_from_output, OUTPUT_SHAPE


def test_letterbox_rgb_and_nms_return_original_pixels():
    image = np.zeros((720, 1280, 3), np.uint8)
    image[:, :, 0] = 255
    tensor, geometry = letterbox(image)
    assert tensor.shape == (1, 3, 640, 640)
    assert geometry == (0.5, 0, 140, 1280, 720)
    assert tensor[0, 0, 200, 300] == 1 and tensor[0, 2, 200, 300] == 0
    output = np.zeros(OUTPUT_SHAPE, np.float32)
    output[0, :, 0] = [320, 320, 100, 100, .9]
    output[0, :, 1] = [322, 320, 100, 100, .8]
    output[0, :, 2] = [20, 20, 5, 5, .95]  # Padding-only box must be rejected.
    boxes = boxes_from_output(output, geometry, .25, .45)
    assert len(boxes) == 1
    assert np.allclose(boxes[0], [540, 260, 200, 200, .9])


def test_bad_output_is_error_not_empty_evidence():
    with pytest.raises(ValueError):
        boxes_from_output(np.zeros((1, 6, 8400)), (1, 0, 0, 640, 640), .25, .45)
    assert boxes_from_output(np.zeros(OUTPUT_SHAPE), (1, 0, 0, 640, 640), .25, .45) == []
