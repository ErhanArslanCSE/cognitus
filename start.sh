#!/bin/bash
# COGNITUS Startup Script
# Works on both Docker and LIMO Pro

set -e

echo "═══════════════════════════════════════════════"
echo "  🤖 COGNITUS Starting..."
echo "═══════════════════════════════════════════════"

# Detect environment
if [ -f "/.dockerenv" ]; then
    ENV="docker"
else
    ENV="limo_pro"
fi

echo "Environment: $ENV"
echo ""

# Source ROS2 Foxy
source /opt/ros/foxy/setup.bash
echo "✓ ROS2 Foxy sourced"

# Build if needed
if [ ! -d "install" ]; then
    echo ""
    echo "Building workspace..."
    colcon build --symlink-install
fi

# Source workspace
source install/setup.bash
echo "✓ Workspace sourced"

# Export
export ROS_DOMAIN_ID=0

echo ""
echo "═══════════════════════════════════════════════"
echo "  ✓ COGNITUS Ready!"
echo "═══════════════════════════════════════════════"
echo ""

# Launch based on environment
if [ "$ENV" == "limo_pro" ]; then
    echo "🤖 Launching on LIMO Pro hardware..."
    echo ""
    ros2 launch cognitus_integration limo_pro.launch.py
else
    echo "🐳 Launching in Docker (simulation)..."
    echo ""
    ros2 launch cognitus_integration simulation.launch.py
fi
