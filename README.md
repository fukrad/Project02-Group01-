# Project02-Group01-

# DoBot Pick-and-Place Project

**Course**: Space Robotics - Project 02 Group 01  
**Team**: Abhijay Sekhar Choudhary, Kai Ikeuchi, Ronan Raikar, Subesh Shanmugam

Vision-based robotic pick-and-place system using YOLOv8 and DoBot Magician.

---

## Prerequisites

- Ubuntu 20.04 (WSL2 or native)
- ROS Noetic
- Python 3.8+

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
sudo modprobe ch341
sleep 2
sudo chmod 666 /dev/ttyUSB0
source ~/catkin_ws/devel/setup.bash
Launch System
Open 4 terminals and run: Terminal 1: DoBot Driver
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
Robot will detect, pick, and place the object.
Calibration (One-time Setup)
python3 calibration_xyz.py
Move DoBot to random positions
Press c to capture calibration point
Press q to skip
Repeat for ~30 points
Calibration saved to calibration_30points_with_rotation.json
Troubleshooting
DoBot driver not starting
sudo modprobe ch341
sudo chmod 666 /dev/ttyUSB0
roslaunch dobot_magician_driver dobot_magician.launch
Camera not detected
rs-enumerate-devices
realsense-viewer
License
MIT

4. Commit message: **"Add setup and usage instructions"**
5. Click **"Commit changes"**

---

**That's it! Simple and focused on running the code.** ✅

Do you want to add `requirements.txt` too, or is the README enough?
what is the license ?? maybe no need   also  no need to include trouble shootinhg
You're right! Let me simplify it even more:
Minimal README (Just How to Run):
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
