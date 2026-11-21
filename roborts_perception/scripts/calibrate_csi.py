#!/usr/bin/env python3
"""Chessboard calibration for the gimbal CSI camera (design doc appendix A.3).

Live mode grabs frames from the same GStreamer pipeline the perception node
uses, detects the chessboard and collects ``--samples`` distinct poses:

    ros2 run roborts_perception calibrate_csi --cols 9 --rows 6 \
        --square-size 0.025 --output camera_csi.yaml

Then copy the generated file over ``roborts_perception/config/camera_csi.yaml``
(or pass it to ``person_gesture_node`` as ``camera_info_path``).

Offline mode calibrates from already-saved images (useful when the board was
captured with ``gst-launch-1.0`` or in a previous session):

    ros2 run roborts_perception calibrate_csi --images /tmp/calib --output ...

The output is a ROS ``camera_info``-style YAML with ``plumb_bob`` distortion,
which is exactly what ``camera_model.CameraModel.from_yaml`` expects.
"""

import argparse
import glob
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import yaml

from csi_camera import CsiCamera

IMAGE_EXTENSIONS = ('*.png', '*.jpg', '*.jpeg', '*.bmp')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Chessboard calibration for the gimbal CSI camera')
    parser.add_argument('--cols', type=int, default=9,
                        help='inner corners along the board width (default: 9)')
    parser.add_argument('--rows', type=int, default=6,
                        help='inner corners along the board height (default: 6)')
    parser.add_argument('--square-size', type=float, default=0.025,
                        help='square edge length in metres (default: 0.025)')
    parser.add_argument('--samples', type=int, default=25,
                        help='number of accepted views (default: 25)')
    parser.add_argument('--interval', type=float, default=0.5,
                        help='minimum seconds between accepted views '
                             '(default: 0.5)')
    parser.add_argument('--output', default='camera_csi.yaml',
                        help='output YAML path (default: ./camera_csi.yaml)')
    parser.add_argument('--images', default=None,
                        help='calibrate from images in this directory '
                             'instead of the live camera')
    parser.add_argument('--save-images', default=None,
                        help='save each accepted live frame to this directory')
    parser.add_argument('--show', action='store_true',
                        help='show a preview window (requires a display)')
    parser.add_argument('--sensor-id', type=int, default=0)
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--height', type=int, default=1080)
    parser.add_argument('--framerate', type=int, default=30)
    return parser.parse_args(argv)


def find_chessboard(gray, pattern_size):
    """Return ``(found, corners)`` with sub-pixel corners when available."""
    finder_sb = getattr(cv2, 'findChessboardCornersSB', None)
    if finder_sb is not None:
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        found, corners = finder_sb(gray, pattern_size, flags=flags)
        if found:
            return True, corners.astype(np.float32)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(
        gray, pattern_size, flags=flags)
    if not found:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30, 0.001)
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def make_object_points(cols, rows, square_size):
    points = np.zeros((rows * cols, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size
    return points


def _is_new_pose(corners, previous, image_size):
    """Reject near-identical views so the calibration stays well conditioned."""
    if previous is None:
        return True
    current = corners.reshape(-1, 2)
    previous = previous.reshape(-1, 2)
    diagonal = float(np.hypot(image_size[0], image_size[1]))

    center_shift = np.linalg.norm(
        current.mean(axis=0) - previous.mean(axis=0))
    if center_shift > 0.04 * diagonal:
        return True

    span_current = current.max(axis=0) - current.min(axis=0)
    span_previous = previous.max(axis=0) - previous.min(axis=0)
    return bool(np.any(np.abs(span_current - span_previous) > 0.02 * diagonal))


def collect_live(args, pattern_size, object_template):
    camera = CsiCamera(
        sensor_id=args.sensor_id,
        width=args.width,
        height=args.height,
        framerate=args.framerate)
    try:
        camera.start()
    except RuntimeError as exc:
        print(f'error: cannot start CSI camera: {exc}', file=sys.stderr)
        return None

    if args.save_images:
        os.makedirs(args.save_images, exist_ok=True)

    show = args.show
    object_points = []
    image_points = []
    image_size = None
    previous_corners = None
    last_accept = 0.0
    last_status = 0.0

    print(f'collecting {args.samples} views of a '
          f'{args.cols}x{args.rows} inner-corner board '
          f'({args.square_size * 1000:.1f} mm squares); move the board '
          'between samples, Ctrl+C to finish early')

    try:
        while len(image_points) < args.samples:
            frame = camera.read_latest(timeout=1.0)
            if frame is None:
                error = camera.error_message
                if error:
                    print(f'\nerror: {error}', file=sys.stderr)
                    return None
                continue

            gray = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY)
            if image_size is None:
                image_size = (gray.shape[1], gray.shape[0])
            found, corners = find_chessboard(gray, pattern_size)
            now = time.monotonic()

            accepted = False
            if (found and now - last_accept >= args.interval and
                    _is_new_pose(corners, previous_corners, image_size)):
                image_points.append(corners.reshape(-1, 2).astype(np.float32))
                object_points.append(object_template.copy())
                previous_corners = corners
                last_accept = now
                accepted = True
                index = len(image_points) - 1
                print(f'  accepted view {index + 1}/{args.samples} '
                      f'(mean reprojection error is reported at the end)')
                if args.save_images:
                    path = os.path.join(
                        args.save_images, f'calib_{index:02d}.png')
                    cv2.imwrite(path, frame.image)

            if now - last_status >= 1.0:
                last_status = now
                state = 'board detected' if found else 'searching for board'
                print(f'  {len(image_points)}/{args.samples} views, {state}   ',
                      end='\r', flush=True)

            if show:
                preview = frame.image if not found else frame.image.copy()
                if found and not accepted:
                    cv2.drawChessboardCorners(
                        preview, pattern_size, corners, found)
                try:
                    cv2.imshow('calibrate_csi', preview)
                    key = cv2.waitKey(1) & 0xFF
                except cv2.error as exc:
                    print(f'\nwarning: cannot open preview window ({exc}); '
                          'continuing without display', file=sys.stderr)
                    show = False
                    continue
                if key in (27, ord('q')):
                    print('\ninterrupted by user')
                    break
    except KeyboardInterrupt:
        print('\ninterrupted by user')
    finally:
        camera.stop()
        if show:
            cv2.destroyAllWindows()

    if len(image_points) < 5:
        print(f'error: only {len(image_points)} views collected, need at '
              'least 5', file=sys.stderr)
        return None
    return object_points, image_points, image_size


def collect_images(path, pattern_size, object_template):
    paths = []
    for extension in IMAGE_EXTENSIONS:
        paths.extend(glob.glob(os.path.join(path, extension)))
    paths = sorted(set(paths))
    if not paths:
        print(f'error: no images found in {path}', file=sys.stderr)
        return None

    object_points = []
    image_points = []
    image_size = None
    for image_path in paths:
        image = cv2.imread(image_path)
        if image is None:
            print(f'  skip unreadable image: {image_path}')
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        elif image_size != (gray.shape[1], gray.shape[0]):
            print(f'  skip {image_path}: size differs from the first image')
            continue
        found, corners = find_chessboard(gray, pattern_size)
        if not found:
            print(f'  skip {image_path}: no chessboard')
            continue
        object_points.append(object_template.copy())
        image_points.append(corners.reshape(-1, 2).astype(np.float32))
        print(f'  {len(image_points)} views ({os.path.basename(image_path)})')

    if len(image_points) < 5:
        print(f'error: only {len(image_points)} usable images, need at least 5',
              file=sys.stderr)
        return None
    return object_points, image_points, image_size


def calibrate(object_points, image_points, image_size):
    print(f'calibrating with {len(image_points)} views at '
          f'{image_size[0]}x{image_size[1]} ...')
    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None)

    print('per-view reprojection error:')
    for index, (points, corners, rvec, tvec) in enumerate(
            zip(object_points, image_points, rvecs, tvecs)):
        projected, _ = cv2.projectPoints(
            points, rvec, tvec, camera_matrix, distortion)
        projected = projected.reshape(-1, 2).astype(np.float64)
        observed = corners.reshape(-1, 2).astype(np.float64)
        error = float(np.linalg.norm(projected - observed, axis=1).mean())
        print(f'  view {index + 1:2d}: {error:.3f} px')
    return rms, camera_matrix, distortion


def write_calibration(path, image_size, camera_matrix, distortion, rms):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    data = {
        'image_width': int(image_size[0]),
        'image_height': int(image_size[1]),
        'camera_name': 'csi_camera',
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [float(value) for value in camera_matrix.reshape(-1)],
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1,
            'cols': 5,
            'data': [float(value) for value in
                     distortion.reshape(-1)[:5]],
        },
        'reprojection_error': float(rms),
        'calibrated': True,
        'calibrated_date': datetime.now().isoformat(timespec='seconds'),
    }
    with open(path, 'w', encoding='utf-8') as stream:
        stream.write(
            '# CSI camera intrinsics generated by '
            '`ros2 run roborts_perception calibrate_csi`.\n'
            '# Copy this file to '
            'roborts_perception/config/camera_csi.yaml.\n')
        yaml.safe_dump(data, stream, sort_keys=False)
    print(f'wrote {path} (RMS reprojection error {rms:.3f} px)')


def main(argv=None):
    args = parse_args(argv)
    pattern_size = (args.cols, args.rows)
    object_template = make_object_points(
        args.cols, args.rows, args.square_size)

    if args.images:
        collected = collect_images(
            os.path.expanduser(args.images), pattern_size, object_template)
    else:
        collected = collect_live(args, pattern_size, object_template)
    if collected is None:
        return 1

    object_points, image_points, image_size = collected
    rms, camera_matrix, distortion = calibrate(
        object_points, image_points, image_size)

    if rms > 0.5:
        print(f'warning: RMS error {rms:.3f} px exceeds the 0.5 px target; '
              'recollect with more varied views and better focus',
              file=sys.stderr)

    write_calibration(
        os.path.expanduser(args.output),
        image_size, camera_matrix, distortion, rms)
    return 0


if __name__ == '__main__':
    sys.exit(main())
