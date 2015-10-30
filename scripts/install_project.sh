#!/usr/bin/env bash
# Installation script that creates a catkin workspace and moves this repository into that workspace

CURRENT_DIR=${PWD}
cd ~/
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
mv ${CURRENT_DIR}/../../mechatronics ~/catkin_ws/src/.
cd ~/catkin_ws/
catkin_make
