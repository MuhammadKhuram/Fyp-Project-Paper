# Autonomous Delta Robot Simulation - Setup & Run Guide

This simulation uses Webots as the 3D environment and an external Python script (`<extern>`) to handle the complex kinematics, nearest-neighbor path optimization, and robot control.

## Prerequisites
1. **Webots R2025a** installed on your system.
2. **Python 3.10+** installed.
3. A Python virtual environment with required dependencies.

## Step 1: Prepare the Python Environment
Open your terminal, navigate to your main project folder, and set up the virtual environment:

```bash
# Create the virtual environment (if not already created)
python3 -m venv webots_env

# Install required mathematical packages
./webots_env/bin/pip install numpy opencv-python ultralytics
```
*(Note: `opencv-python` and `ultralytics` are included for the YOLOv8 vision pipeline).*

## Step 2: Configure the Launch Script
Because this uses an `<extern>` controller, Webots communicates with Python through local IPC (Inter-Process Communication) outside of the standard sandbox. 

Open the **`run_delta.sh`** file and ensure the paths match your specific machine. If you are using the Snap version of Webots on Ubuntu, the environment variables in the script must be set to:

```bash
export WEBOTS_HOME=/snap/webots/current/usr/share/webots
export WEBOTS_TMPDIR=~/snap/webots/common
```
**Important:** Ensure the final line in `run_delta.sh` points accurately to your absolute path for `webots_env/bin/python3` and `delta_controller.py`.

## Step 3: Launch the Webots World
1. Open Webots.
2. Go to **File -> Open World...** and select **`worlds/delta_robot.wbt`**.
3. You should see the crop field, the weeds, and the Delta Robot hovering above them.
4. **CRITICAL STEP:** Press the **Play (▶️)** button at the top of Webots. 
   *(The simulation time will stay at 00:00:00, and the console will say "Waiting for local or remote connection...". This is normal! Webots is pausing until the Python script talks to it).*

## Step 4: Run the Controller
With Webots waiting and paused, open a new terminal window, navigate to your project folder, and run the bash script:

```bash
# Make sure the script is executable (only needed the first time)
chmod +x run_delta.sh

# Run the script
./run_delta.sh
```

## What to Expect
As soon as you run the script:
1. The terminal will generate and print a list of 6 random weed coordinates.
2. It will apply a **Greedy Nearest-Neighbor** algorithm to sort them into the most efficient path.
3. The Webots simulation will unfreeze and connect to the Python controller.
4. You will see the Delta robot flawlessly execute the precise **Approach -> Plunge -> Dwell (Cut) -> Retract** sequence for all 6 weeds before returning to its dead-center hover position.
```