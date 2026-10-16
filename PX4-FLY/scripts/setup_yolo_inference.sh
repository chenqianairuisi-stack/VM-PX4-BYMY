#!/usr/bin/env bash
set -euo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
/usr/bin/python3 -m venv --without-pip --system-site-packages "$REPO_DIR/.venv-yolo"
"$REPO_DIR/.venv-yolo/bin/python" -m pip install 'numpy<2' 'onnxruntime==1.20.1'
