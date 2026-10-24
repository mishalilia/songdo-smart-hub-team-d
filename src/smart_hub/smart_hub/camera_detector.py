# Author: Andrey Li (12250249)
"""
S1 camera path: people detection, foot points and the image-to-map homography (8.2).

OpenCV's built-in HOG people detector finds pedestrians in a frame; the bottom centre of
each box is the foot point, which a homography from four image points to four floor points
in the map frame turns into map coordinates. The map points then go to the same
ZoneTracker as the synthetic detections, so counting rules do not change.
"""
import cv2
import numpy as np

# Calibration for the PETS 2009 view used in the S1 video test (OpenCV sample vtest.avi,
# 768 x 576): a quadrilateral of the paved path, mapped onto a 5 m x 3 m patch around Q.
PETS_IMAGE_POINTS = [(160, 176), (672, 160), (736, 304), (96, 240)]
PETS_MAP_POINTS = [(1.5, 7.5), (6.5, 7.5), (6.5, 4.5), (1.5, 4.5)]
# People there are about 55 px tall, below the detector's 128 px window: detect in a crop
# around the calibrated area (x0, y0, x1, y1), enlarged by PETS_UPSCALE.
PETS_ROI = (60, 40, 760, 330)
PETS_UPSCALE = 2.5


def foot_points(boxes):
    """Return the bottom centre (x + w / 2, y + h) of each (x, y, w, h) box."""
    return [(x + w / 2.0, y + h) for x, y, w, h in boxes]


def suppress(boxes, scores, overlap=0.4):
    """Greedy non-maximum suppression; keeps the higher-scoring of overlapping boxes."""
    order = sorted(range(len(boxes)), key=lambda i: -scores[i])
    kept = []
    for i in order:
        x, y, w, h = boxes[i]
        if all(_iou((x, y, w, h), boxes[k]) <= overlap for k in kept):
            kept.append(i)
    return [boxes[i] for i in kept]


def _iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix = max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0.0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


class CameraCounter:
    """Frame -> map-frame foot points of the detected pedestrians."""

    def __init__(self, image_points=PETS_IMAGE_POINTS, map_points=PETS_MAP_POINTS,
                 min_score=0.3, roi=PETS_ROI, upscale=PETS_UPSCALE):
        self.homography = cv2.getPerspectiveTransform(
            np.float32(image_points), np.float32(map_points))
        self.min_score = min_score
        self.roi, self.upscale = roi, upscale
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detect(self, frame):
        """Return pedestrian boxes (x, y, w, h) in pixels."""
        x0, y0 = 0, 0
        if self.roi:
            x0, y0, x1, y1 = self.roi
            frame = frame[y0:y1, x0:x1]
        k = self.upscale or 1.0
        if k != 1.0:
            frame = cv2.resize(frame, None, fx=k, fy=k, interpolation=cv2.INTER_LINEAR)
        rects, weights = self.hog.detectMultiScale(
            frame, winStride=(8, 8), padding=(8, 8), scale=1.05)
        boxes, scores = [], []
        for r, w in zip(rects, np.ravel(weights) if len(rects) else []):
            if w >= self.min_score:
                x, y, bw, bh = (float(v) / k for v in r)
                boxes.append((x + x0, y + y0, bw, bh))
                scores.append(float(w))
        return suppress(boxes, scores)

    def to_map(self, points):
        """Map image points (u, v) to map-frame points (x, y)."""
        if not points:
            return []
        pts = np.float32(points).reshape(-1, 1, 2)
        return [tuple(float(c) for c in p[0])
                for p in cv2.perspectiveTransform(pts, self.homography)]

    def to_image(self, points):
        """Inverse mapping, map frame -> image (for drawing zones on frames)."""
        pts = np.float32(points).reshape(-1, 1, 2)
        inv = np.linalg.inv(self.homography)
        return [tuple(float(c) for c in p[0]) for p in cv2.perspectiveTransform(pts, inv)]

    def map_points(self, frame):
        """Detect pedestrians in a frame and return their foot points in the map frame."""
        return self.to_map(foot_points(self.detect(frame)))
