#!/bin/bash

# Prompt for username and password
echo "Logging into nvcr.io"
read -p "Username: " username
read -sp "Password: " password
echo ""

# Login to nvcr.io
echo "$password" | docker login nvcr.io -u "$username" --password-stdin

# Check if login was successful
if [ $? -ne 0 ]; then
    echo "Docker login failed. Exiting."
    exit 1
fi

# Build the Docker image using Dockerfile_isaacsim
echo "Building Docker image from Dockerfile_isaacsim..."
docker build -t isaac-sim:custom-build -f Dockerfile_isaacsim .

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "Docker image built successfully."
else
    echo "Docker image build failed."
    exit 1
fi
