#!/bin/bash
# Run Script for Webots with Rust Controllers to launch Webots 

# detect WEBOTS_HOME if not set
if [ -z "$WEBOTS_HOME" ]; then
    if [ -d "/Applications/Webots.app/Contents" ]; then
        export WEBOTS_HOME="/Applications/Webots.app/Contents"
    elif [ -d "/usr/local/webots" ]; then
        export WEBOTS_HOME="/usr/local/webots"
    else
        echo "Error: WEBOTS_HOME not set and Webots not found"
        exit 1
    fi
fi

echo "WEBOTS_HOME=$WEBOTS_HOME"

# set library path
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS verison: 
    export DYLD_LIBRARY_PATH="$WEBOTS_HOME/lib/controller:$DYLD_LIBRARY_PATH"
    echo "DYLD_LIBRARY_PATH set"
else
    # other standard verions: 
    export LD_LIBRARY_PATH="$WEBOTS_HOME/lib/controller:$LD_LIBRARY_PATH"
    echo "LD_LIBRARY_PATH set"
fi

# kill any processes on ports
echo "Clearing ports 5001 and 5002"
kill -9 $(lsof -t -i :5001) 2>/dev/null || true
kill -9 $(lsof -t -i :5002) 2>/dev/null || true

# launch Webots
echo "Launching Webots"
if [[ "$OSTYPE" == "darwin"* ]]; then
    open -a Webots "$(pwd)/worlds/fire_detection_rust.wbt"
else
    webots "$(pwd)/worlds/fire_detection_rust.wbt" &
fi

echo "Webots launched with world"
