#!/bin/bash

# Define the Docker image name (using the one you just built)
IMAGE_NAME="isaac-sim:custom-build"

# Run the Docker container
docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all \
    -e "ACCEPT_EULA=Y" --rm --network=host -p 47995:47995 \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    $IMAGE_NAME
