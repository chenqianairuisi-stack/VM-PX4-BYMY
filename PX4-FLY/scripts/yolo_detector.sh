#!/usr/bin/env bash
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source "$REPO_DIR/install/local_setup.bash"
MODEL="${YOLO_MODEL:-$REPO_DIR/weights/drone.onnx}"
YOLO_PYTHON="${YOLO_PYTHON:-$REPO_DIR/.venv-yolo/bin/python}"
if [[ ! -f "$MODEL" ]]; then
  echo "Missing trained drone model: $MODEL" >&2
  echo 'Run train_windows_gpu.cmd in the shared training folder, then ./scripts/import_drone_model.sh.' >&2
  exit 1
fi
if [[ ! -x "$YOLO_PYTHON" ]]; then
  echo 'Prepare the inference environment: ./scripts/setup_yolo_inference.sh' >&2
  exit 1
fi
if [[ "${1:-}" == --check ]]; then
  "$YOLO_PYTHON" -c 'from px4_fly_control.drone_onnx import DroneOnnx; import sys; DroneOnnx(sys.argv[1])' "$MODEL"
  exit
fi
exec "$YOLO_PYTHON" -m px4_fly_control.yolo_detector --ros-args -p "model:=$MODEL" "$@"
