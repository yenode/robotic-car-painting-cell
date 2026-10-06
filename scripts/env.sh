#!/usr/bin/env bash
# Source this file from the repository root in every simulation terminal.
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/install/setup.bash"
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=painting_cell_42
export ROS_LOG_DIR=/tmp/painting_cell_logs
