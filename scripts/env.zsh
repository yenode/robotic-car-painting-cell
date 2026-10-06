#!/usr/bin/env zsh
# Source this file from the repository root in every Zsh simulation terminal.
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.zsh
_painting_cell_root="${0:A:h:h}"
source "${_painting_cell_root}/install/setup.zsh"
unset _painting_cell_root
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=painting_cell_42
export ROS_LOG_DIR=/tmp/painting_cell_logs
