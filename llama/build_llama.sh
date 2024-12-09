#!/bin/bash

# Build the Docker image using Dockerfile_isaacsim
echo "Building Docker image from Dockerfile_llama.."

docker build -t llama:latest -f Dockerfile_llama .

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "Docker image built successfully."
else
    echo "Docker image build failed."
    exit 1
fi
