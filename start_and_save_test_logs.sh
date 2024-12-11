#!/bin/bash

# Define variables
SERVICE_NAME="yolo-demo"
LOG_DURATION=15  # Duration in seconds to capture logs
GESTURE_TEST_DIR="gesture_test"  # Directory to store log files

# Check if the gesture_test directory exists, if not, create it
if [ ! -d "$GESTURE_TEST_DIR" ]; then
  mkdir "$GESTURE_TEST_DIR"
fi

# Find the next available log file number
i=1
while [ -f "${GESTURE_TEST_DIR}/gesture_test${i}.txt" ]; do
  ((i++))
done

# Set the log file name with the incremented number
LOG_FILE="${GESTURE_TEST_DIR}/gesture_test${i}.txt"

# Check if the container is running
CONTAINER_ID=$(docker ps -qf "name=${SERVICE_NAME}")

# Start the container if it's not running
if [ -z "$CONTAINER_ID" ]; then
  echo "Starting container '$SERVICE_NAME'..."
  docker compose -f docker-compose-dev.yaml up "$SERVICE_NAME" -d
  if [ $? -eq 0 ]; then
    echo "Container '$SERVICE_NAME' started successfully."
    # Get the new container ID
    CONTAINER_ID=$(docker ps -qf "name=${SERVICE_NAME}")
  else
    echo "Failed to start the container '$SERVICE_NAME'."
    exit 1
  fi
fi


# Save the logs for a limited duration
echo "Capturing logs from container '$CONTAINER_ID' for $LOG_DURATION seconds..."
timeout "$LOG_DURATION" docker logs -f "$CONTAINER_ID" > "$LOG_FILE"

# Check if logs were successfully captured
if [ $? -eq 0 ]; then
  echo "Logs captured successfully for $LOG_DURATION seconds and saved to $LOG_FILE."
else
  echo "Failed to capture logs. Check if the container is running properly."
fi

# Stop the container after capturing logs
echo "Stopping the container '$SERVICE_NAME'..."
docker stop "$CONTAINER_ID"

# docker container prune -f

# Confirm completion
if [ $? -eq 0 ]; then
  echo "Container '$SERVICE_NAME' stopped successfully."
else
  echo "Failed to stop the container '$SERVICE_NAME'."
  exit 1
fi


