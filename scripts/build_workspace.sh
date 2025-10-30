#!/bin/bash
# COGNITUS - Build ROS2 Workspace

set -e

echo "════════════════════════════════════════"
echo "  🔨 Building COGNITUS Workspace"
echo "════════════════════════════════════════"

# Source ROS2
source /opt/ros/foxy/setup.bash

# Build
colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash

echo ""
echo "✓ Build complete!"
echo ""
echo "Source workspace:"
echo "  source install/setup.bash"
echo ""
