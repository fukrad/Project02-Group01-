import rospy
from std_msgs.msg import String
import json
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Pose
import time

# Load calibration
CALIBRATION_FILE = 'calibration_30points_with_rotation.json'
with open(CALIBRATION_FILE, 'r') as f:
    cal_data = json.load(f)

ROTATION_MATRIX = np.array(cal_data['position']['rotation_matrix'])
TRANSLATION_VECTOR = np.array(cal_data['position']['translation_vector'])
ANGLE_OFFSET_DEG = cal_data['rotation']['angle_offset_deg']

class PickerProper:
    def __init__(self):
        rospy.init_node("picker_proper", anonymous=True)

        # Proper ROS Publishers
        self.pose_pub = rospy.Publisher("/dobot_magician/target_end_effector_pose", Pose, queue_size=10)
        self.joint_pub = rospy.Publisher("/dobot_magician/target_joint_states", JointTrajectory, queue_size=10)
        self.gripper_pub = rospy.Publisher("/dobot_magician/target_tool_state", UInt8MultiArray, queue_size=10)
        
        self.latest_detections = {}
        rospy.Subscriber('/object_detections', String, self.detection_callback, queue_size=1)
        print("Waiting for detections...")
        rospy.sleep(3)  # Wait for publishers
        print("Ready!")

    def detection_callback(self, msg):
        try:
            self.latest_detections = json.loads(msg.data)
        except:
            pass

    def cmd_joints(self, j0, j1, j2, j3, duration=2.0):
        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = [j0, j1, j2, j3]
        point.time_from_start = rospy.Duration(duration)
        traj.points.append(point)
        self.joint_pub.publish(traj)
        rospy.sleep(duration)

    def cmd_pose(self, x, y, z, r=0.0, duration=2.0): # x y z + rotaion and  move with 
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        # Convert rotation angle r to quaternion
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = np.sin(r / 2.0)
        pose.orientation.w = np.cos(r / 2.0)
        self.pose_pub.publish(pose)
        rospy.sleep(duration)

    def grip_open(self):
        msg = UInt8MultiArray()
        msg.data = [1, 0]
        self.gripper_pub.publish(msg)
        rospy.sleep(0.5)

    def grip_close(self):
        msg = UInt8MultiArray()
        msg.data = [1, 1]
        self.gripper_pub.publish(msg)
        rospy.sleep(1.0)

    def grip_off(self):
        msg = UInt8MultiArray()
        msg.data = [0, 0]
        self.gripper_pub.publish(msg)
        rospy.sleep(0.3)

    def pick(self, object_name):

        if object_name not in self.latest_detections:
            print(f"{object_name.upper()} not detected")
            return

        obj = self.latest_detections[object_name]
        cam_pos = np.array(obj['position_3d'])

        # Apply position calibration: camera coords → robot coords
        robot_pos = ROTATION_MATRIX @ cam_pos + TRANSLATION_VECTOR
        x, y, z = robot_pos

        # Calculate rotation angle with calibration
        if object_name == 'top':
            r = 0.0
        else:
            cam_angle = obj['angle']
            angle = cam_angle + 90.0 + ANGLE_OFFSET_DEG
            while angle > 180: angle -= 180
            while angle < 0: angle += 180
            r = np.radians(angle)

        print(f"\n=== PICKING {object_name.upper()} ===")
        print(f"Calibrated position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}, R={np.degrees(r):.1f}°")

        # ONE WORKFLOW
        print("Step 1: HOME")
        self.grip_off()
        self.cmd_joints(0.0, 0.3, 0.2, 0.0)

        print("Step 2: ABOVE pick")
        self.cmd_pose(x, y + 0.05, z, r)

        print("Step 3: OPEN")
        self.grip_open()

        print("Step 4: DOWN to pick")
        self.cmd_pose(x, y, z, r)

        print("Step 5: CLOSE")
        self.grip_close()

        print("Step 6: UP")
        self.cmd_pose(x, y + 0.05, z, r)

        print("Step 7: ABOVE drop")
        self.cmd_joints(1.5, 0.5, 0.4, 0)

        print("Step 8: DOWN to drop")
        self.cmd_joints(1.5, 0.8, 0.9, 0)

        print("Step 9: OPEN")
        self.grip_open()

        print("Step 10: UP")
        self.cmd_joints(1.5, 0.5, 0.4, 0)

        print("Step 11: HOME")
        self.cmd_joints(0.0, 0.3, 0.2, 0.0)

        print("Step 12: OFF")
        self.grip_off()
        print("Done!\n")

    def test_fixed_position(self):
        print(f"\n=== TEST FIXED POSITION ===")

        j0 = 0.0
        j1_pick = 0.84
        j2_pick = 0.85
        j3 = 0.0
        j1_drop = 0.8
        j2_drop = 0.9

        print("Step 1: HOME")
        self.grip_off()
        self.cmd_joints(0.0, 0.3, 0.2, 0.0)

        print("Step 2: ABOVE pick")
        self.cmd_joints(j0, 0.5, 0.4, j3)

        print("Step 3: OPEN")
        self.grip_open()

        print("Step 4: DOWN")
        self.cmd_joints(j0, j1_pick, j2_pick, j3)

        print("Step 5: CLOSE")
        self.grip_close()

        print("Step 6: UP")
        self.cmd_joints(j0, 0.5, 0.4, j3)

        print("Step 7: ABOVE drop")
        self.cmd_joints(1.5, 0.5, 0.4, 0)

        print("Step 8: DOWN")
        self.cmd_joints(1.5, j1_drop, j2_drop, 0)

        print("Step 9: OPEN")
        self.grip_open()

        print("Step 10: UP")
        self.cmd_joints(1.5, 0.5, 0.4, 0)

        print("Step 11: HOME")
        self.cmd_joints(0.0, 0.3, 0.2, 0.0)

        print("Step 12: OFF")
        self.grip_off()
        print("Done!\n")

    def test_top_position(self):
        print(f"\n=== TEST TOP POSITION ===")

        j0 = 0.0
        j1_pick = 0.7
        j2_pick = 0.7
        j3 = 0.0
        j1_drop = 0.7
        j2_drop = 0.7

        print("Step 1: HOME")
        self.grip_off()
        self.cmd_joints(0.0, 0.3, 0.2, 0.0)

        print("Step 2: ABOVE pick")
        self.cmd_joints(j0, 0.5, 0.4, j3)

        print("Step 3: OPEN")
        self.grip_open()

        print("Step 4: DOWN")
        self.cmd_joints(j0, j1_pick, j2_pick, j3)

        print("Step 5: CLOSE")
        self.grip_close()

        print("Step 6: UP")
        self.cmd_joints(j0, 0.5, 0.4, j3)

        print("Step 7: ABOVE drop")
        self.cmd_joints(1.5, 0.5, 0.4, 0)

        print("Step 8: DOWN")
        self.cmd_joints(1.5, j1_drop, j2_drop, 0)

        print("Step 9: OPEN")
        self.grip_open()

        print("Step 10: UP")
        self.cmd_joints(1.5, 0.5, 0.4, 0)

        print("Step 11: HOME")
        self.cmd_joints(0.0, 0.3, 0.2, 0.0)

        print("Step 12: OFF")
        self.grip_off()
        print("Done!\n")

    def run(self):
        while not rospy.is_shutdown():
            print("\nWhich object?")
            print("  1 - PEN")
            print("  2 - USB")
            print("  3 - TOP")
            print("  4 - TEST (fixed position 0.84, 0.85)")
            print("  t - TEST TOP (fixed position 0.7, 0.7)")
            print("  q - Quit")
            choice = input(">> ").strip()

            if choice == '1':
                self.pick('pen')
            elif choice == '2':
                self.pick('usb')
            elif choice == '3':
                self.pick('top')
            elif choice == '4':
                self.test_fixed_position()
            elif choice.lower() == 't':
                self.test_top_position()
            elif choice.lower() == 'q':
                break

def main():
    try:
        picker = PickerProper()
        picker.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nStopped")

if __name__ == '__main__':
    main()
