#!/usr/bin/env bash
# Run create_map unit tests that do not need a full ROS install.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src/create_map:${PYTHONPATH:-}"
python3 -m pytest -q "${REPO_ROOT}/src/create_map/test"
