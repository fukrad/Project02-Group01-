# Project02-Group01-


# DoBot Pick-and-Place Project

**Course**: Space Robotics - Project 02 Group 01  
**Team**: Abhijay Sekhar Choudhary, Kai Ikeuchi, Ronan Raikar, Subesh Shanmugam

Vision-based robotic pick-and-place system using YOLOv8 and DoBot Magician.

---

## Installation

### 1. Install Python Dependencies
```bash
pip install ultralytics opencv-python pyrealsense2 numpy open3d
2. Install ROS Packages
sudo apt install ros-noetic-realsense2-camera ros-noetic-cv-bridge
3. Install DoBot Driver
cd ~/catkin_ws/src
git clone https://github.com/gapaul/dobot_magician_driver.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
How to Run
Setup (WSL2 Only)
Windows PowerShell (as Admin):
usbipd attach --wsl --busid 2-2
WSL2 Terminal:
sudo modprobe ch341 && sleep 2 && sudo chmod 666 /dev/ttyUSB0
source ~/catkin_ws/devel/setup.bash
Launch System (Open 4 Terminals)
Terminal 1: DoBot Driver
roslaunch dobot_magician_driver dobot_magician.launch
Terminal 2: Camera
roslaunch realsense2_camera rs_camera.launch align_depth:=true
Terminal 3: YOLO Detector
cd /root/project2
python3 yolo_detector.py
Terminal 4: Pickup Controller
cd /root/project2
python3 dobot_pickup_proper.py
Usage
In Terminal 4, select object:
Press 1 for PEN
Press 2 for USB
Press 3 for TOP
Calibration (One-time Setup)
python3 calibration_xyz.py
Press c to capture each calibration point (~30 points total).
