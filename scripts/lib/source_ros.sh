# shellcheck shell=bash
# ROS setup.bash references optional AMENT_* vars; with `set -u` that aborts.
# Use petcam_source <setup.bash> instead of bare source.

petcam_source() {
  local setup_file="$1"
  if [[ ! -f "${setup_file}" ]]; then
    echo "ERROR: setup file not found: ${setup_file}" >&2
    return 1
  fi
  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
}
