#!/bin/bash
# Build and Deploy Script for Rust Webots Controllers 

set -e

BUILD_TYPE="${1:-release}"

echo "  Building Rust-based Detection"
echo "  Build type: $BUILD_TYPE"

# check for Rust installation present 
if ! command -v cargo &> /dev/null; then
    echo "ERROR: Rust/Cargo not found. Install from https://rustup.rs"
    exit 1
fi

# check/set WEBOTS_HOME
if [ -z "$WEBOTS_HOME" ]; then
    echo "WEBOTS_HOME not set. Attempting to detect Webots"
    
    # check for Webots.app - mac version: export WEBOTS_HOME=\"/Applications/Webots.app/Contents\"
    if [ -d "/Applications/Webots.app/Contents/lib/controller" ]; then
        export WEBOTS_HOME="/Applications/Webots.app/Contents"
        echo "Found macOS Webots: $WEBOTS_HOME"
    else
        echo "Error:: Cannot find Webots installation."
        echo "Set WEBOTS_HOME manually:"
        exit 1
    fi
fi

# verify library exists
if [ ! -d "$WEBOTS_HOME/lib/controller" ]; then
    echo "Error: Webots controller library not found at $WEBOTS_HOME/lib/controller"
    exit 1
fi

echo "Using WEBOTS_HOME=$WEBOTS_HOME"
echo "Library: $WEBOTS_HOME/lib/controller"
echo ""

# build
echo "Building controllers"
if [ "$BUILD_TYPE" = "debug" ]; then
    cargo build
    TARGET_DIR="target/debug"
else
    cargo build --release
    TARGET_DIR="target/release"
fi

# create controller directories
echo ""
echo "Deploying controllers..."
mkdir -p controllers/drone_thermal
mkdir -p controllers/drone_bbox
mkdir -p controllers/ground_station

# copy executables
cp "$TARGET_DIR/drone_thermal" controllers/drone_thermal/
cp "$TARGET_DIR/drone_bbox" controllers/drone_bbox/
cp "$TARGET_DIR/ground_station" controllers/ground_station/

# make executable
chmod +x controllers/drone_thermal/drone_thermal
chmod +x controllers/drone_bbox/drone_bbox
chmod +x controllers/ground_station/ground_station

echo "Build complete"
echo "Controllers deployed to:"
echo " - controllers/drone_thermal/drone_thermal"
echo " - controllers/drone_bbox/drone_bbox"
echo " - controllers/ground_station/ground_station"
