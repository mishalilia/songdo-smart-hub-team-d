# Author: Andrey Li (12250249)
"""S1 camera path (8.2): foot points, suppression, homography and an empty frame."""
import numpy as np
from smart_hub.camera_detector import (CameraCounter, foot_points, PETS_IMAGE_POINTS,
                                       PETS_MAP_POINTS, suppress)


def test_foot_point_is_bottom_centre():
    assert foot_points([(10, 20, 30, 60)]) == [(25.0, 80)]


def test_suppress_keeps_best_of_overlapping_boxes():
    boxes = [(0, 0, 10, 20), (1, 1, 10, 20), (50, 0, 10, 20)]
    assert suppress(boxes, [0.5, 0.9, 0.4]) == [(1, 1, 10, 20), (50, 0, 10, 20)]


def test_homography_maps_calibration_points_and_back():
    c = CameraCounter()
    for got, want in zip(c.to_map(PETS_IMAGE_POINTS), PETS_MAP_POINTS):
        assert np.allclose(got, want, atol=1e-3)
    for got, want in zip(c.to_image(PETS_MAP_POINTS), PETS_IMAGE_POINTS):
        assert np.allclose(got, want, atol=1e-2)


def test_empty_frame_has_no_people():
    assert CameraCounter().map_points(np.zeros((576, 768, 3), np.uint8)) == []
