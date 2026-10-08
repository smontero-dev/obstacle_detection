#!/usr/bin/env bash
# Opens a shell in the ROS 2 Humble development container (Linux hosts only).
#
# First call: builds the image (cached, so it only rebuilds when
# requirements.txt or the Dockerfile change) and starts the container with GUI
# support, mounting the repository at ~/ros2_ws.
# Later calls while it is running: open another shell in the same container.
#
# Requires Docker and rocker (pip install rocker).
set -euo pipefail

IMAGE=obstacle_detection:humble
CONTAINER=obstacle_detection
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    exec docker exec -it "${CONTAINER}" \
        bash -c 'source /opt/ros/humble/setup.bash && cd ~/ros2_ws && exec bash'
fi

if ! command -v rocker >/dev/null; then
    echo 'rocker not found: install it with "pip install rocker" (or activate its venv)' >&2
    exit 1
fi

echo "Building ${IMAGE} (the first time takes a few minutes)..."
docker build -q -t "${IMAGE}" -f "${REPO_DIR}/docker/Dockerfile" "${REPO_DIR}" >/dev/null

# rocker joins the command into one string and splits it again respecting
# quotes, so the inner command must be passed as a single quoted argument.
rocker --x11 --user --network=host --name "${CONTAINER}" \
    --volume "${REPO_DIR}:/home/${USER}/ros2_ws" \
    -- "${IMAGE}" "bash -c 'cd ~/ros2_ws && exec bash'"
