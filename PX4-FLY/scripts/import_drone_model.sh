#!/usr/bin/env bash
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE="${1:-$HOME/VM-shared/VM-PX4-BYMY/drone_training/trained_model}"
if [[ ! -f "$SOURCE/drone.onnx" || ! -f "$SOURCE/metrics.json" ]]; then
  echo "Training output not found in: $SOURCE" >&2
  exit 1
fi
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source "$REPO_DIR/install/local_setup.bash"
"$REPO_DIR/.venv-yolo/bin/python" -c 'from px4_fly_control.drone_onnx import DroneOnnx; import sys; DroneOnnx(sys.argv[1])' "$SOURCE/drone.onnx"
cp "$SOURCE/drone.onnx" "$REPO_DIR/weights/drone.onnx"
cp "$SOURCE/metrics.json" "$REPO_DIR/weights/metrics.json"
if [[ -f "$SOURCE/drone.pt" ]]; then cp "$SOURCE/drone.pt" "$REPO_DIR/weights/drone.pt"; fi
printf '%s\n' 'Imported drone model. Check the held-out test metrics before a SITL follow run.'
