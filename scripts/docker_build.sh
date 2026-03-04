#!/bin/bash

# Docker Build Script for ROS2 Workspace with working:2 Base Image
# This script builds a Docker image using working:2 as base and performs colcon build

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
WORKSPACE_DIR="${1:-.}"
IMAGE_NAME="${2:-crawler:latest}"
DOCKERFILE_PATH="${3:-./scripts/Dockerfile.prod}"
BUILD_TAG="robot-build-$(date +%s)"

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Validate workspace
if [ ! -d "$WORKSPACE_DIR/src" ]; then
    print_error "ROS2 workspace 'src' directory not found at $WORKSPACE_DIR/src"
    exit 1
fi

if [ ! -f "$DOCKERFILE_PATH" ]; then
    print_error "Dockerfile not found at $DOCKERFILE_PATH"
    exit 1
fi

print_status "Starting Docker build process..."
print_status "Workspace: $WORKSPACE_DIR"
print_status "Image name: $IMAGE_NAME"
print_status "Dockerfile: $DOCKERFILE_PATH"

# Check if working:2 image exists
print_status "Verifying base image 'working:2' exists..."
if ! docker image inspect working:2 > /dev/null 2>&1; then
    print_error "Base image 'working:2' not found. Please build it first."
    exit 1
fi

print_status "Base image 'working:2' found ✓"

# Build the Docker image
print_status "Building Docker image..."
docker build \
    -f "$DOCKERFILE_PATH" \
    -t "$BUILD_TAG" \
    -t "$IMAGE_NAME" \
    "$WORKSPACE_DIR"

if [ $? -eq 0 ]; then
    print_status "Docker image built successfully! ✓"
    print_status "Image tags: $BUILD_TAG, $IMAGE_NAME"
    
    # Display image information
    echo ""
    print_status "Image details:"
    docker image inspect "$IMAGE_NAME" --format='Size: {{.Size}} bytes'
    
    # Option to verify the build
    print_status "Build verification:"
    docker run --rm "$IMAGE_NAME" bash -c "echo 'Checking ROS2 installation...' && source /opt/ros/jazzy/setup.bash && ros2 --version && echo 'ROS2 ready!' && if [ -f install/setup.bash ]; then source install/setup.bash && echo 'Workspace setup sourced successfully!'; else echo 'Warning: install/setup.bash not found'; fi"
    
    echo ""
    print_status "To run the container, use:"
    echo "  docker run -it --rm $IMAGE_NAME bash"
    echo ""
    print_status "To run with GPU support, use:"
    echo "  docker run -it --rm --gpus all $IMAGE_NAME bash"
    echo ""
    print_status "To run with ROS2 launch, use:"
    echo "  docker run -it --rm $IMAGE_NAME ros2 launch <package> <launch_file>"
else
    print_error "Docker build failed!"
    exit 1
fi
