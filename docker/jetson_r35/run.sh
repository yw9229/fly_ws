#!/bin/bash

# Default image name (must match the name in build.sh)
IMAGE_NAME="my_jetson_r35"

# 定義顏色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # 無顏色

# Get the directory to mount (parent of the script's location)
# MOUNT_DIR=$(dirname "$(cd "$(dirname "$0")" && pwd)")               # Get the parent directory of the script's location
MOUNT_DIR=$(dirname "$(dirname "$(cd "$(dirname "$0")" && pwd)")")  # Get the grandparent directory of the script's location


# Run the Docker container
echo -e "${YELLOW}Running Docker container from image '$IMAGE_NAME'...${NC}"
docker run -it --rm \
           -v /dev:/dev \
           -v "$MOUNT_DIR":/workspace \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           -v ${HOME}/.Xauthority:/home/${user}/.Xauthority \
           -e DISPLAY=$DISPLAY \
           -e QT_X11_NO_MITSHM=1 \
           -e XAUTHORITY=${HOME}/.Xauthority \
           --net=host \
           --ipc=host \
           --privileged \
           --runtime nvidia \
           "$IMAGE_NAME"

# Check if the container ran successfully
if [ $? -eq 0 ]; then
  echo -e "${GREEN}Success:${NC} Docker container exited successfully."
else
  echo -e "${RED}Error:${NC} Docker container failed to run."
  exit 1
fi
