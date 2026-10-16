#!/usr/bin/env bash
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export FOLLOW_MODE=virtual_contact
exec "$REPO_DIR/scripts/start_single_terminal.sh"
