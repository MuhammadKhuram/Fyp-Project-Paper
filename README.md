# Autonomous Weed Removal Robot - Simulation & Modeling

This repository contains the software simulation, kinematic modeling, and vision AI for an autonomous weeding robot. 

## How the Code is Structured
We are using 4 core Python files.

* **`delta_robot_kinematics.py`**: Generates 3D workspace and joint angle plots.
* **`mobile_robot_navigation.py`**: Generates path planning and EKF error plots.
* **`yolov8_weed_detector.py`**: Evaluates YOLOv8 performance and saves bounding-box images.
* **`main_robot_controller.py`**: Simulates the full robotic mission and logs battery/latency metrics.

## Setup
1. Clone the repository.
2. Install dependencies: `pip install -r requirements.txt`
3. Run any of the 4 python files to generate outputs.






---
## 💻 Team Guide: Git & Terminal Crash Course

If you are new to using Git and the terminal, use this cheat sheet to help you navigate, run files, create folders, and upload your code safely without deleting each other's work!

### 1. How to Download the Code (Clone)

The very first time you want to get this code on your computer, open your terminal (in VS Code, Command Prompt, or Git Bash) and run:

```bash
git clone https://github.com/MuhammadKhuram/Fyp-Project-Paper.git C:\Users\MUHAMMAD_KHURAM\Desktop\Fyp-Project-Paper\Fyp-Project-Paper
cd "C:\Users\MUHAMMAD_KHURAM\Desktop\Fyp-Project-Paper\Fyp-Project-Paper"
```



### 2. The Daily Git Workflow (Pull, Commit, Push)

Every time you sit down to work, and every time you finish working, follow these exact 5 steps to avoid ruining the code.

**Step 1: ALWAYS pull before you start typing!**

This brings any changes your groupmate made down to your computer so you are both synchronized.

```bash
git pull origin main
```

**Step 2: Check what you changed**

After you write some code, check which files you modified:

```bash
git status
```

**Step 3: Stage your changes**

This tells Git you are ready to save all the files you just edited:

```bash
git add .
```

*(The `.` means "add everything". If you only want to add one specific file, use `git add filename.py`)*

**Step 4: Commit your changes**

This saves a "snapshot" of your work with a message explaining what you did:

```bash
git commit -m "Added matplotlib plotting to the kinematics file"
```

**Step 5: Push your code to GitHub**

This uploads your saved snapshot to the repository so your groupmate can see it:

```bash
git push origin main
```

### 3. Basic Terminal Commands

If you want to use the terminal instead of the mouse to navigate and create things:

* **See what is inside your current folder:**
  * Windows: `dir`
  * Mac/Linux/Git Bash: `ls`
* **Go inside a folder:**
  * `cd folder_name` (e.g., `cd outputs`)
* **Go back/up one folder:**
  * `cd ..`
* **Create a new folder:**
  * `mkdir new_folder_name`
* **Create a new empty file:**
  * Windows (Command Prompt): `code new_file.py`, `code readme.md`, `code file.txt` etc.

### 4. How to Run Our Python Files

To run any of the 4 code files and generate the simulation graphs, make sure you are in the main project folder in your terminal. Then type `python` followed by the file name:

```bash
python delta_robot_kinematics.py
```

*(If the terminal gives you an error like "ModuleNotFoundError", it means you forgot to install the libraries! Run `pip install -r requirements.txt` first).*

### ⚠️ Golden Rule for Group Work

**COMMUNICATE!** Before you start editing a file (like `yolov8_weed_detector.py`), message your groupmate and say, *"Hey, I am working on the YOLO file right now."* This prevents you both from editing the exact same lines of code at the exact same time, which causes annoying "Merge Conflicts."