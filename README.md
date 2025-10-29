# DoBot Pick-and-Place Project

## Installation

### 1. DoBot Driver
```bash
cd ~/catkin_ws/src
git clone https://github.com/gapaul/dobot_magician_driver.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. RealSense ROS
```bash
sudo apt install ros-noetic-realsense2-camera
```

**Required repositories:**
- DoBot Driver: https://github.com/gapaul/dobot_magician_driver
- RealSense ROS: https://github.com/IntelRealSense/realsense-ros

---

## How to Run

### Setup (WSL2)

**Windows PowerShell (as Admin):**

First, check which USB devices need to be attached:
```powershell
usbipd list
```

Find the devices (look for DoBot and RealSense camera), then attach both:
```powershell
# Attach DoBot (example: busid 2-2)
usbipd attach --wsl --busid 2-2

# Attach RealSense Camera (example: busid 1-4)
usbipd attach --wsl --busid 1-4
```

**WSL2 Terminal:**
```bash
# Load DoBot driver and fix permissions
sudo modprobe ch341 && sleep 2 && sudo chmod 666 /dev/ttyUSB0

# Source ROS workspace
source ~/catkin_ws/devel/setup.bash
```

---

### Launch System (Open 4 Terminals)

**Terminal 1: DoBot Driver**
```bash
roslaunch dobot_magician_driver dobot_magician.launch
```

**Terminal 2: Camera**
```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

**Terminal 3: YOLO Detector**
```bash
cd /root/project2
python3 yolo_detector.py
```

**Terminal 4: Pickup Controller**
```bash
cd /root/project2
python3 dobot_pickup_proper.py
```

---

## Usage
- Press 1 for PEN
- Press 2 for USB
- Press 3 for TOP
- Press 4 for TEST (fixed position)
- Press t for TEST TOP (fixed position)

---

## Calibration

```bash
python3 calibration_xyz.py
```

Press `c` to capture each calibration point (~30 points total).
