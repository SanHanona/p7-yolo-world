#!/bin/bash

# Define the Docker image name (using the one you just built)
IMAGE_NAME="llama:latest"

docker run --rm -it --privileged --net=host --runtime nvidia --gpus all --name llama_container $IMAGE_NAME