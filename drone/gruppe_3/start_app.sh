#!/bin/bash

# Ensure the script stops on errors
set -e

DRONE_CONTAINER="drone_container"
DRONE_START_FILE="start_drone"

OPENVINS_CONTAINER="openvins"
OPENVINS_START_FILE="start"

# Start the first Docker container using its start.sh
echo "Starting the first Docker container..."
(cd $DRONE_CONTAINER && ./$DRONE_START_FILE.sh) &

# Start the second Docker container using its start.sh
echo "Starting the second Docker container..."
(cd $OPENVINS_CONTAINER && ./$OPENVINS_START_FILE.sh) &

# Wait for all background processes to complete
wait

echo "Both containers have been started."
