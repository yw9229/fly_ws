#!/bin/bash

# Default image name (can be modified by the user)
IMAGE_NAME="my_jetson_r35"


# 定義顏色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # 無顏色


# Check if the image name is valid
if [[ "$IMAGE_NAME" =~ [^a-z0-9_.-] ]]; then
  echo -e "${RED}Error:${NC} Image name '$IMAGE_NAME' contains invalid characters. Only lowercase letters, numbers, dots, underscores, and dashes are allowed."
  exit 1
fi

# Check if the image name is already in use
if docker images --format "{{.Repository}}" | grep -q "^${IMAGE_NAME}$"; then
  echo -e "${YELLOW}Warning:${NC} Image name '$IMAGE_NAME' already exists."

  # Prompt user for overwrite confirmation with default option
  read -p "Do you want to overwrite the existing image? (Y/n): " RESPONSE
  RESPONSE=${RESPONSE,,} # Convert to lowercase

  # If response is not 'n', proceed with overwrite
  if [[ "$RESPONSE" == "n" ]]; then
    echo -e "${RED}Aborted:${NC} Build cancelled by the user."
    exit 1
  else
    echo -e "${YELLOW}Info:${NC} Overwriting the existing image '$IMAGE_NAME'."
  fi
fi

# Build the Docker image
echo -e "${YELLOW}Building Docker image '$IMAGE_NAME'...${NC}"
docker build -t "$IMAGE_NAME" .

# Check if the build was successful
if [ $? -eq 0 ]; then
  echo -e "${GREEN}Success:${NC} Docker image '$IMAGE_NAME' built successfully!"
else
  echo -e "${RED}Error:${NC} Failed to build Docker image."
  exit 1
fi
