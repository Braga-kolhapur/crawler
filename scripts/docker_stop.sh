#!/bin/bash

# Docker Stop Script
# Stops the crawler-container gracefully

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

CONTAINER_NAME="crawler-container"

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

# Check if container is running
if ! sudo docker ps | grep -q "$CONTAINER_NAME"; then
    print_warning "Container '$CONTAINER_NAME' is not running"
    exit 0
fi

print_status "Stopping container '$CONTAINER_NAME'..."

# Stop the container gracefully
if sudo docker stop "$CONTAINER_NAME"; then
    print_status "Container stopped successfully ✓"
else
    print_error "Failed to stop container"
    exit 1
fi

# Optional: Remove the container (commented out as docker run uses --rm)
# sudo docker rm "$CONTAINER_NAME"
