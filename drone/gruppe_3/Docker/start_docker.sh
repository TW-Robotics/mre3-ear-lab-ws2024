#!/bin/bash

# Ensure the script stops on errors
set -e

# Define the image name
IMAGE_NAME="ros_noetic_drone"

# Build the Docker image
echo "Building the Docker image..."
docker build -t $IMAGE_NAME .

# Run the Docker container with the required arguments
echo "Starting the Docker container..."
# wenn ich interaktive session starten will einfach bash nach image name (anschließend sh händisch in session ausführen), sonst die auszuführende sh einfügen um sie abzuspulen
docker run -it --net host --privileged -v /dev:/dev -v $PWD/app:/app $IMAGE_NAME bash /app/commands.sh #&
#docker run ... & 
#docker run ...