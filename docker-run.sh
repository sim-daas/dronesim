#!/bin/bash

# Set image name and container name from arguments or use defaults
IMAGE="${1:-px4-dronesim:v1}"
CONTAINER_NAME="${2:-dronebase_dev}"

# Use third argument as host repo path, or default to current directory
HOST_REPO_PATH="${3:-$(pwd)}"
CONTAINER_REPO_PATH="/root/dronews/src/dronesim"

# Determine if custom volume mapping is provided
if [[ "$HOST_REPO_PATH" == *:* ]]; then
    VOLUME_ARG=( -v "$HOST_REPO_PATH" )
else
    VOLUME_ARG=( -v "$HOST_REPO_PATH:$CONTAINER_REPO_PATH" )
fi

# Shift to allow passing extra -v options as further arguments
shift 3 || true

# X11 and user environment (host must provide XAUTH/XAUTHORITY)
XSOCK="/tmp/.X11-unix"
XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"

# Check for NVIDIA GPU
if command -v nvidia-smi &> /dev/null; then
    GPU_OPTS="--gpus all \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all"
else
    GPU_OPTS=""
fi

# Check if container exists
if docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
    # If running, exec into it
    if docker ps --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
        echo "Container '${CONTAINER_NAME}' is already running. Exec into it."
        docker exec -it "${CONTAINER_NAME}" bash
        exit 0
    else
        echo "Container '${CONTAINER_NAME}' exists but is stopped. Starting and attaching..."
        docker start "${CONTAINER_NAME}"
        docker exec -it "${CONTAINER_NAME}" bash
        exit 0
    fi
fi

# --privileged gives access to all host devices (USB, etc.)
DOCKER_RUN_CMD=(docker run "${VOLUME_ARG[@]}" \
    -it \
    --privileged \
    --net=host \
    --ipc=host \
    -v /dev/shm:/dev/shm \
    -v "${XSOCK}:${XSOCK}" \
    -v /dev/dri:/dev/dri \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY="${XAUTH}" \
    -v "${XAUTH}:${XAUTH}" \
    --name "${CONTAINER_NAME}" \
    ${GPU_OPTS} \
    "$@" \
    "${IMAGE}")

# Print the docker run command
printf '\nDocker run command:\n'
printf '%s ' "${DOCKER_RUN_CMD[@]}"
printf '\n\n'

# Run the command
"${DOCKER_RUN_CMD[@]}"
