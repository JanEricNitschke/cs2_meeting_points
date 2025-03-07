#!/bin/bash

set -e  # Exit on error

if [ $# -ne 1 ]; then
    echo "Usage: $0 <map_name>"
    exit 1
fi

# Get the directory of the script to resolve paths correctly
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"  # Assume script is inside ./scripts/, so move one level up

MAP_NAME="$1"
IMAGE_PATH="$REPO_ROOT/spread_images/${MAP_NAME}/spread_${MAP_NAME}_"
OUTPUT_GIF="$REPO_ROOT/spread_gifs/${MAP_NAME}/spread.gif"
MAX_SIZE=100000000    # Strict upper limit: 100MB
MIN_ACCEPTABLE_SIZE=80000000  # 80MB
PREFERRED_MAX_SIZE=95000000   # 95MB

# Ensure output directory exists
mkdir -p "$REPO_ROOT/spread_gifs/${MAP_NAME}"

# Get the total number of images
TOTAL_IMAGES=$(ls ${IMAGE_PATH}*.png | wc -l)

# Binary search bounds
LOW=1
HIGH=$TOTAL_IMAGES
BEST_N=1

generate_gif() {
    ffmpeg -framerate 3 -i "${IMAGE_PATH}"%d.png -frames:v "$1" -loop -1 -y "$OUTPUT_GIF" -loglevel error
}

while [ $LOW -le "$HIGH" ]; do
    MID=$(((LOW + HIGH) / 2))

    echo "Testing with $MID frames..."
    generate_gif $MID

    GIF_SIZE=$(stat -c%s "$OUTPUT_GIF")

    if [ "$GIF_SIZE" -lt "$MAX_SIZE" ]; then
        # Valid GIF size, update best found
        BEST_N=$MID
        LOW=$((MID + 1))  # Try more frames if possible

        # Stop early if the GIF is within the preferred range
        if [ "$GIF_SIZE" -ge "$MIN_ACCEPTABLE_SIZE" ] && [ "$GIF_SIZE" -le "$PREFERRED_MAX_SIZE" ]; then
            echo "Optimal GIF found in range 85MB–95MB!"
            break
        fi
    else
        # Too large, reduce frames
        HIGH=$((MID - 1))
    fi
done

echo "Generating final GIF with $BEST_N frames..."
generate_gif $BEST_N

echo "Final GIF size: $(stat -c%s "$OUTPUT_GIF") bytes"
echo "Used $BEST_N frames to stay between 85MB and 95MB (strictly <100MB)"
