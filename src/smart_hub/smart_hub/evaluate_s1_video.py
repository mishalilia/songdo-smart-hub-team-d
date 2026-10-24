# Author: Andrey Li (12250249)
"""
S1 video evaluation: run the camera path over a recorded video and compare with manual counts.

    ros2 run smart_hub evaluate_s1_video --video vtest.avi --truth truth.csv
    ros2 run smart_hub evaluate_s1_video --video vtest.avi --sheet 5,10,15 --sheet-out sheet.png

Each frame goes through CameraCounter (HOG detection, foot points, homography) and the same
ZoneTracker that passenger_detector uses; the queue count at each truth time is compared with
the manual count (CSV `time_sec,count[,occluded]`). Writes counts.csv and result.json (MAE,
exact-match rate, MAE on the frames marked as partially occluded, detector time per frame) to
--out. Precision/recall needs per-person box labels, which count annotations do not have.
`--sheet` instead draws zone Q on frames at the given seconds, for counting people by hand.
"""
import argparse
import csv
import json
import os
import time

import cv2
import numpy as np
from smart_hub import hub_map
from smart_hub.camera_detector import CameraCounter
from smart_hub.tracker import ZoneTracker


def zone_polygon(counter):
    xmin, xmax, ymin, ymax = hub_map.QUEUE_Q
    corners = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
    return np.int32(counter.to_image(corners))


def make_sheet(video, seconds, out, counter):
    cap = cv2.VideoCapture(video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
    tiles = []
    for s in seconds:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(round(s * fps)))
        ok, frame = cap.read()
        if not ok:
            continue
        cv2.polylines(frame, [zone_polygon(counter)], True, (0, 255, 255), 2)
        cv2.putText(frame, 't=%gs' % s, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (0, 0, 255), 3)
        tiles.append(cv2.resize(frame, (480, 360)))
    while len(tiles) % 3:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)]
    cv2.imwrite(out, np.vstack(rows))


def run(video, counter):
    """Return [(t, count)] once per processed frame and the mean detector time (ms)."""
    cap = cv2.VideoCapture(video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
    tracker = ZoneTracker()
    out, spent, n = [], 0.0, 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = n / fps
        start = time.monotonic()
        points = counter.map_points(frame)
        spent += time.monotonic() - start
        tracker.update(points, t)
        out.append((t, tracker.queue_count()))
        n += 1
    return out, 1000.0 * spent / max(n, 1)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--video', required=True)
    parser.add_argument('--truth', help='CSV time_sec,count of manual counts')
    parser.add_argument('--out', default=os.path.join('evidence', 's1_video'))
    parser.add_argument('--sheet', help='comma-separated seconds: draw zone Q on those frames')
    parser.add_argument('--sheet-out', default='s1_sheet.png')
    opts, _ = parser.parse_known_args(args)
    counter = CameraCounter()
    if opts.sheet:
        make_sheet(opts.video, [float(s) for s in opts.sheet.split(',')], opts.sheet_out,
                   counter)
        return
    counts, ms = run(opts.video, counter)
    os.makedirs(opts.out, exist_ok=True)
    with open(os.path.join(opts.out, 'counts.csv'), 'w', newline='') as f:
        csv.writer(f).writerows([('time_sec', 'count')] + [('%.1f' % t, c) for t, c in counts])
    result = {'video': os.path.basename(opts.video), 'frames': len(counts),
              'detector_ms_per_frame': round(ms, 1)}
    if opts.truth:
        with open(opts.truth) as f:
            truth = [(float(r['time_sec']), int(r['count']), r.get('occluded') == '1')
                     for r in csv.DictReader(f)]
        pairs = []
        for t, expected, occluded in truth:
            got = min(counts, key=lambda c: abs(c[0] - t))[1]
            pairs.append({'time_sec': t, 'manual': expected, 'system': got,
                          'occluded': occluded})
        errors = [abs(p['manual'] - p['system']) for p in pairs]
        occ = [abs(p['manual'] - p['system']) for p in pairs if p['occluded']]
        clear = [abs(p['manual'] - p['system']) for p in pairs if not p['occluded']]
        result.update(occluded_frames=len(occ),
                      mae_occluded=round(sum(occ) / len(occ), 2) if occ else None,
                      mae_not_occluded=round(sum(clear) / len(clear), 2) if clear else None,
                      bias=round(sum(p['system'] - p['manual'] for p in pairs) / len(pairs), 2),
                      precision_recall='not available: count labels, no per-person boxes')
        result.update(samples=len(pairs), mae=round(sum(errors) / len(errors), 2),
                      exact_match=round(sum(e == 0 for e in errors) / len(errors), 2),
                      within_one=round(sum(e <= 1 for e in errors) / len(errors), 2),
                      comparison=pairs)
    with open(os.path.join(opts.out, 'result.json'), 'w') as f:
        json.dump(result, f, indent=2)
    print(json.dumps({k: v for k, v in result.items() if k != 'comparison'}))


if __name__ == '__main__':
    main()
