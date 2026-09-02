#!/bin/bash

# 1. Set environment variables
export WEBOTS_HOME=/snap/webots/current/usr/share/webots
export LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller
export PYTHONPATH=$WEBOTS_HOME/lib/controller/python
export WEBOTS_TMPDIR=/home/muhammad-khuram-linux/snap/webots/common

echo "Connecting to Webots..."

# 2. Run the controller
/home/muhammad-khuram-linux/Desktop/Fyp-Project-Paper/Fyp-Project-Paper/cropcraft/venv/bin/python3 /home/muhammad-khuram-linux/Desktop/Fyp-Project-Paper/Fyp-Project-Paper/Delta-Robot-Subsystem/Delta_robot_webots/controllers/delta_controller/delta_controller.py
