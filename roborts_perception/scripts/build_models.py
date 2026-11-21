#!/usr/bin/env python3
"""Build TensorRT engines for the gimbal perception node (offline only).

Run this on the target Jetson with the same TensorRT version used at runtime.
It exports Ultralytics pose weights to ONNX and calls trtexec.  Neither torch
nor ultralytics is used by the running node.

Examples:
  # P4: person pose engine (downloads yolo11n-pose.pt if it is missing)
  ros2 run roborts_perception build_models --pose

  # P5: hand pose engine from a community YOLO11-hand-pose checkpoint
  ros2 run roborts_perception build_models --hand-weights /path/to/best.pt

Outputs (inside <package share>/models):
  yolo11n-pose.onnx, yolo11n-pose_fp16.engine
  hand_pose.onnx, hand_pose_fp16.engine
"""

import argparse
import os
import shutil
import subprocess
import sys
import urllib.request


def models_dir():
    """Prefer the source tree so generated files are installed by colcon."""
    here = os.path.dirname(os.path.abspath(__file__))
    source = os.path.join(os.path.dirname(here), 'models')
    if os.path.isdir(source):
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('roborts_perception'), 'models')
    except Exception:
        return source


POSE_WEIGHTS_URL = (
    'https://github.com/ultralytics/assets/releases/download/v8.3.0/'
    'yolo11n-pose.pt'
)


def find_trtexec():
    for candidate in ('trtexec', '/usr/src/tensorrt/bin/trtexec'):
        path = shutil.which(candidate) or (
            candidate if os.path.isfile(candidate) else None)
        if path:
            return path
    raise SystemExit(
        'error: trtexec not found (install libnvinfer-bin)')


def export_onnx(weights, imgsz, output_dir, batch=1, output_name=None):
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            'error: ultralytics/torch are required for this offline step: '
            f'{exc}')

    model = YOLO(weights)
    path = model.export(
        format='onnx', imgsz=imgsz, opset=17, simplify=False,
        dynamic=False, batch=batch)
    target = os.path.join(
        output_dir, output_name or os.path.basename(path))
    if os.path.abspath(path) != os.path.abspath(target):
        shutil.move(path, target)
    print(f'exported {target}')
    return target


def build_engine(onnx_path, engine_path):
    trtexec = find_trtexec()
    command = [
        trtexec, f'--onnx={onnx_path}', '--fp16',
        f'--saveEngine={engine_path}']
    print('running:', ' '.join(command))
    subprocess.run(command, check=True)
    if not os.path.isfile(engine_path):
        raise SystemExit(f'error: engine was not written: {engine_path}')
    print(f'built {engine_path}')


def ensure_pose_weights(output_dir, weights):
    if weights:
        if not os.path.isfile(weights):
            raise SystemExit(f'error: weights not found: {weights}')
        return weights
    target = os.path.join(output_dir, 'yolo11n-pose.pt')
    if os.path.isfile(target):
        return target
    print(f'downloading {POSE_WEIGHTS_URL}')
    urllib.request.urlretrieve(POSE_WEIGHTS_URL, target)
    return target


def main():
    parser = argparse.ArgumentParser(
        description='Build TensorRT engines for roborts_perception')
    parser.add_argument('--pose', action='store_true',
                        help='build the person pose engine (P4)')
    parser.add_argument('--pose-weights', default=None,
                        help='local yolo11n-pose .pt (default: download)')
    parser.add_argument('--hand-weights', default=None,
                        help='hand-pose .pt; builds the P5 engine')
    parser.add_argument('--pose-imgsz', type=int, default=640)
    parser.add_argument('--hand-imgsz', type=int, default=320)
    parser.add_argument('--hand-batch', type=int, default=1)
    args = parser.parse_args()

    if not args.pose and not args.hand_weights:
        parser.error('select --pose and/or --hand-weights')

    output_dir = models_dir()
    os.makedirs(output_dir, exist_ok=True)

    if args.pose:
        weights = ensure_pose_weights(output_dir, args.pose_weights)
        onnx_path = export_onnx(
            weights, args.pose_imgsz, output_dir, batch=1,
            output_name='yolo11n-pose.onnx')
        build_engine(
            onnx_path,
            os.path.join(output_dir, 'yolo11n-pose_fp16.engine'))

    if args.hand_weights:
        if not os.path.isfile(args.hand_weights):
            raise SystemExit(
                f'error: hand weights not found: {args.hand_weights}')
        onnx_path = export_onnx(
            args.hand_weights, args.hand_imgsz, output_dir,
            batch=args.hand_batch, output_name='hand_pose.onnx')
        build_engine(
            onnx_path,
            os.path.join(output_dir, 'hand_pose_fp16.engine'))

    print('done; rebuild the package (colcon build --packages-select '
          'roborts_perception --symlink-install) so the launch file sees '
          'new engine files')
    return 0


if __name__ == '__main__':
    sys.exit(main())
