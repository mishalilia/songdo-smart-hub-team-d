# Author: Andrey Li (12250249)
"""
S1 camera path: people detection, foot points and the image-to-map homography (8.2).

A detector finds pedestrians in a frame; a point near the bottom centre of each box is the
foot point, which a homography from four image points to four floor points in the map frame
turns into map coordinates. The map points then go to the same ZoneTracker as the synthetic
detections, so counting rules do not change.

Detectors (DETECTORS): `yolox` (default) is the YOLOX-s CNN of the OpenCV Model Zoo
(models/yolox_s_2022nov.onnx, Apache-2.0); `hog` is OpenCV's HOG people detector, kept as
the reproducible baseline, and `hog_midterm` the HOG settings of the midterm. Thresholds and
foot ratios were chosen on the development part of the labelled video only
(evidence/s1_video/split.json, ablation_plan.json).
"""
import os

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

# Frozen settings per detector: score threshold, NMS overlap, smallest box height (px) and
# the height fraction of the foot point (HOG boxes carry a margin below the feet).
DETECTORS = {
    'yolox': {'min_score': 0.1, 'nms_iou': 0.6, 'min_height': 30.0, 'foot_ratio': 0.95},
    'hog': {'min_score': 0.2, 'nms_iou': 0.3, 'min_height': 0.0, 'foot_ratio': 0.85},
    'hog_midterm': {'min_score': 0.3, 'nms_iou': 0.4, 'min_height': 0.0, 'foot_ratio': 1.0},
}
YOLOX_MODEL = 'yolox_s_2022nov.onnx'
YOLOX_INPUT = 640
HOG_HIT_THRESHOLD = -0.5     # candidate windows; the per-detector min_score selects among them


def default_model_path(name=YOLOX_MODEL):
    """Return the installed model file, or the one in the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('smart_hub'), 'models', name)
        if os.path.exists(path):
            return path
    except (ImportError, LookupError):
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'models', name)


def foot_points(boxes, ratio=1.0):
    """Return (x + w / 2, y + ratio * h) of each (x, y, w, h) box; ratio 1 = bottom centre."""
    return [(x + w / 2.0, y + ratio * h) for x, y, w, h in boxes]


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


class Yolox:
    """YOLOX-s person detector on OpenCV DNN (letterboxed RGB input, raw 0..255 pixels)."""

    PERSON = 0

    def __init__(self, model_path, size=YOLOX_INPUT):
        self.net = cv2.dnn.readNet(model_path)
        self.size = size
        grids, strides = [], []
        for stride in (8, 16, 32):
            n = size // stride
            xv, yv = np.meshgrid(np.arange(n), np.arange(n))
            grid = np.stack((xv, yv), 2).reshape(-1, 2)
            grids.append(grid)
            strides.append(np.full((len(grid), 1), stride))
        self.grids, self.strides = np.concatenate(grids), np.concatenate(strides)

    def detect(self, frame, min_score):
        """Return person boxes (x, y, w, h) and scores in frame pixels."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        ratio = min(self.size / rgb.shape[0], self.size / rgb.shape[1])
        resized = cv2.resize(rgb, (int(rgb.shape[1] * ratio), int(rgb.shape[0] * ratio)))
        blob = np.full((self.size, self.size, 3), 114.0, np.float32)
        blob[:resized.shape[0], :resized.shape[1]] = resized
        self.net.setInput(np.transpose(blob, (2, 0, 1))[np.newaxis])
        out = self.net.forward(self.net.getUnconnectedOutLayersNames())[0][0]
        score = out[:, 4] * out[:, 5 + self.PERSON]
        keep = score >= min_score
        centre = (out[keep, :2] + self.grids[keep]) * self.strides[keep] / ratio
        size = np.exp(out[keep, 2:4]) * self.strides[keep] / ratio
        boxes = [(float(cx - w / 2.0), float(cy - h / 2.0), float(w), float(h))
                 for (cx, cy), (w, h) in zip(centre, size)]
        return boxes, [float(s) for s in score[keep]]


class CameraCounter:
    """Frame -> map-frame foot points of the detected pedestrians."""

    def __init__(self, image_points=PETS_IMAGE_POINTS, map_points=PETS_MAP_POINTS,
                 detector='yolox', model_path='', roi=PETS_ROI, upscale=PETS_UPSCALE):
        if detector not in DETECTORS:
            raise ValueError('unknown detector %r, expected one of %s' % (
                detector, ', '.join(sorted(DETECTORS))))
        self.homography = cv2.getPerspectiveTransform(
            np.float32(image_points), np.float32(map_points))
        self.detector = detector
        self.settings = DETECTORS[detector]
        self.min_score = self.settings['min_score']
        self.roi, self.upscale = roi, upscale
        self.model_path = ''
        if detector == 'yolox':
            self.model_path = model_path or default_model_path()
            self.yolox = Yolox(self.model_path)
        else:
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detect(self, frame):
        """Return pedestrian boxes (x, y, w, h) in pixels."""
        if self.detector == 'yolox':
            boxes, scores = self.yolox.detect(frame, self.min_score)
        else:
            boxes, scores = self._detect_hog(frame)
        pairs = [(b, s) for b, s in zip(boxes, scores) if b[3] >= self.settings['min_height']]
        return suppress([b for b, _ in pairs], [s for _, s in pairs], self.settings['nms_iou'])

    def _detect_hog(self, frame):
        x0, y0 = 0, 0
        if self.roi:
            x0, y0, x1, y1 = self.roi
            frame = frame[y0:y1, x0:x1]
        k = self.upscale or 1.0
        if k != 1.0:
            frame = cv2.resize(frame, None, fx=k, fy=k, interpolation=cv2.INTER_LINEAR)
        rects, weights = self.hog.detectMultiScale(
            frame, hitThreshold=HOG_HIT_THRESHOLD, winStride=(8, 8), padding=(8, 8),
            scale=1.05)
        boxes, scores = [], []
        for r, w in zip(rects, np.ravel(weights) if len(rects) else []):
            if w >= self.min_score:
                x, y, bw, bh = (float(v) / k for v in r)
                boxes.append((x + x0, y + y0, bw, bh))
                scores.append(float(w))
        return boxes, scores

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
        return self.to_map(foot_points(self.detect(frame), self.settings['foot_ratio']))
