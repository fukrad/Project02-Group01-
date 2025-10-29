# Project02-Group01-

# DoBot Pick-and-Place Project

**Course**: Space Robotics - Project 02 Group 01  
**Team**: Abhijay Sekhar Choudhary, Kai Ikeuchi, Ronan Raikar, Subesh Shanmugam

## Overview
Vision-based robotic pick-and-place system using YOLOv8 object detection and DoBot Magician arm. Detects and grasps pen, USB drive, and bottle top with 85-90% accuracy.

## System Architecture

### Hardware
- DoBot Magician M1 robotic arm
- Intel RealSense D435i RGB-D camera
- Windows 11 + WSL2 Ubuntu 20.04

### Software
- ROS Noetic
- YOLOv8 NANO (89.8% mAP@0.5)
- PCL/Open3D for 3D pose estimation
- Python 3.8

## Key Decisions

### Object Detection: SIFT → YOLOv8
- **Initial**: SIFT + RANSAC (20-40% accuracy) ❌
- **Final**: YOLOv8 NANO (85-90% accuracy) ✅
- **Reason**: SIFT failed due to low texture and viewpoint sensitivity

### Motion Control: MoveIt! → Direct Control
- **Initial**: MoveIt! motion planning ❌
- **Final**: Direct Cartesian waypoints ✅
- **Reason**: Simpler and sufficient for pick-and-place

## Repository Structure

src/ ├── yolo_detector.py # YOLO detection ROS node ├── dobot_pickup_proper.py # Pickup control node ├── calibration_xyz.py # Calibration script └── calibration_30points_with_rotation.json # Calibration data

## Installation

### Prerequisites
```bash
# Install Python packages
pip install ultralytics opencv-python pyrealsense2 numpy open3d

# Install ROS packages
sudo apt install ros-noetic-realsense2-camera ros-noetic-cv-bridge
DoBot Driver
cd ~/catkin_ws/src
git clone https://github.com/gapaul/dobot_magician_driver.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
How to Run
Setup (WSL2)
# Windows PowerShell (Admin): Attach USB
usbipd attach --wsl --busid 2-2

# WSL2: Fix permissions
sudo modprobe ch341 && sleep 2 && sudo chmod 666 /dev/ttyUSB0
source ~/catkin_ws/devel/setup.bash
Launch System (4 terminals)
Terminal 1: DoBot
roslaunch dobot_magician_driver dobot_magician.launch
Terminal 2: Camera
roslaunch realsense2_camera rs_camera.launch align_depth:=true
Terminal 3: YOLO
python3 yolo_detector.py
Terminal 4: Pickup
python3 dobot_pickup_proper.py
Then select object: 1 (pen), 2 (USB), 3 (top)
Calibration
Run once for camera-robot calibration:
python3 calibration_xyz.py
Move DoBot to 30 random positions
Press c to capture, q to skip
Saves to calibration_30points_with_rotation.json
Technical Details
YOLOv8 Training: 600 images (480 train / 120 val), labeled with LabelImg
Calibration: ArUco marker ID 1, SVD algorithm for transformation
3D Pose: Depth sampling + PCL OBB for orientation
References
Redmon et al. (2016). "You Only Look Once: Unified, Real-Time Object Detection"
Rusu & Cousins (2011). "3D is here: Point Cloud Library"
License
MIT

4. Commit message: **"Update README with project documentation"**
5. Click **"Commit changes"**

---

### **2. Add `requirements.txt`** (Python dependencies)

1. Click **"Add file"** → **"Create new file"**
2. Filename: `requirements.txt`
3. Content:
ultralytics>=8.0.0 opencv-python>=4.8.0 pyrealsense2>=2.50.0 numpy>=1.21.0 open3d>=0.17.0
4. Commit message: **"Add Python dependencies"**

---

### **3. Optional: Add your presentation slides**

1. Create folder: `docs/presentation.pdf` (upload your slides)
2. Commit message: **"Add project presentation"**

---

## **Priority order:**

1. ✅ **Update README.md** ← Do this first! Most important!
2. ⚠️ Add `requirements.txt` (nice to have)
3. ⚠️ Add presentation slides (optional)

**Start with updating the README.md - that's what assessors will read first!** 📖
