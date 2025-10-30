
import rospy, cv2, numpy as np, time, json, random
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo, JointState
from cv_bridge import CvBridge
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SafeCalibration:
    def __init__(self):
        rospy.init_node('safe_cal')

        self.joint_pub = rospy.Publisher('/dobot_magician/target_joint_states', JointTrajectory, queue_size=10)

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.camera_matrix = None
        self.robot_pose = None
        self.joint_states = None

        rospy.Subscriber('/camera/color/image_raw', Image,
                        lambda m: setattr(self, 'color_image', self.bridge.imgmsg_to_cv2(m, 'bgr8')))
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image,
                        lambda m: setattr(self, 'depth_image', self.bridge.imgmsg_to_cv2(m, 'passthrough')))
        rospy.Subscriber('/camera/color/camera_info', CameraInfo,
                        lambda m: setattr(self, 'camera_matrix', np.array(m.K).reshape(3,3)) if self.camera_matrix is None else None)
        rospy.Subscriber('/dobot_magician/end_effector_poses', PoseStamped,
                        lambda m: setattr(self, 'robot_pose', m.pose.position))
        rospy.Subscriber("/dobot_magician/joint_states", JointState,
                        lambda m: setattr(self, 'joint_states', m.position))

        print('Waiting...')
        for i in range(200):
            if self.color_image is not None and self.robot_pose is not None:
                break
            rospy.sleep(0.1)

        print('Connected!')

        self.camera_points = []
        self.robot_points = []
        self.camera_angles = []
        self.robot_angles = []

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def show_camera(self):
        if self.color_image is not None:
            img = self.color_image.copy()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)

            if ids is not None and 1 in ids:
                cv2.aruco.drawDetectedMarkers(img, corners, ids)
                cv2.putText(img, 'MARKER DETECTED', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            else:
                cv2.putText(img, 'NO MARKER', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

            cv2.imshow('Calibration', img)
            return cv2.waitKey(1) & 0xFF
        return -1

    def move_joints(self, j0, j1, j2, j3):
        print(f'  J0={np.degrees(j0):.1f}° J1={np.degrees(j1):.1f}° J2={np.degrees(j2):.1f}° J3={np.degrees(j3):.1f}°')

        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint']
        point = JointTrajectoryPoint()
        point.positions = [j0, j1, j2, j3]
        point.time_from_start = rospy.Duration(2.0)
        traj.points.append(point)
        self.joint_pub.publish(traj)

        for _ in range(20):
            time.sleep(0.1)
            self.show_camera()

    def detect_marker(self):
        if self.color_image is None or self.depth_image is None:
            return None, None

        gray = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        if ids is not None and 1 in ids:
            idx = np.where(ids == 1)[0][0]
            corner = corners[idx][0]

            cx = int(np.mean(corner[:, 0]))
            cy = int(np.mean(corner[:, 1]))

            depth_region = self.depth_image[max(0,cy-15):min(720,cy+15), max(0,cx-15):min(1280,cx+15)]
            valid_depths = depth_region[depth_region > 0]

            if len(valid_depths) == 0:
                return None, None

            depth = np.median(valid_depths) / 1000.0

            if np.isnan(depth) or depth < 0.1:
                return None, None

            fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
            cx0, cy0 = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

            cam_x = (cx - cx0) * depth / fx
            cam_y = (cy - cy0) * depth / fy
            cam_z = depth + 0.01

            p0, p1 = corner[0], corner[1]
            yaw = np.arctan2(p1[1] - p0[1], p1[0] - p0[0])

            return [cam_x, cam_y, cam_z], yaw

        return None, None

    def capture(self, n, j0, j1, j2, j3):
        print(f'\n[{n}/30]')
        self.move_joints(j0, j1, j2, j3)

        print('  Press C to capture, Q to skip')
        while True:
            key = self.show_camera()
            if key == ord('c') or key == ord('C'):
                break
            elif key == ord('q') or key == ord('Q'):
                return False
            time.sleep(0.05)

        cam_pos, cam_angle = self.detect_marker()

        if cam_pos is None:
            print('   No marker')
            return False

        rob_pos = [self.robot_pose.x, self.robot_pose.y, self.robot_pose.z]
        rob_j3 = self.joint_states[3]

        self.camera_points.append(cam_pos)
        self.robot_points.append(rob_pos)
        self.camera_angles.append(cam_angle)
        self.robot_angles.append(rob_j3)

        print(f'  Cam:{cam_pos[0]:.3f},{cam_pos[1]:.3f},{cam_pos[2]:.3f} Rob_J3:{np.degrees(rob_j3):.1f}°')
        return True

    def run(self):

        cv2.namedWindow('Calibration', cv2.WINDOW_NORMAL)

        positions = []
        for _ in range(30):
            j0 = random.uniform(-0.2, 0.2)
            j1 = random.uniform(0.55, 0.65)
            j2 = random.uniform(0.55, 0.75)
            j3 = random.uniform(-1.22, 1.22)
            positions.append((j0, j1, j2, j3))

        ok = 0
        for i, (j0, j1, j2, j3) in enumerate(positions):
            if self.capture(i+1, j0, j1, j2, j3):
                ok += 1
            if ok >= 30:
                break

        cv2.destroyAllWindows()

        print(f'\nCaptured {ok} points')

        if ok < 10:
            print('Need 10+ points!')
            return

        cam_pos = np.array(self.camera_points) # CAMERA POSIOIN X Y Z 
        rob_pos = np.array(self.robot_points) # ROBOT ENDEFFECT POSTION 

        H = (cam_pos - cam_pos.mean(0)).T @ (rob_pos - rob_pos.mean(0)) # converiance matrix 
        U, S, Vt = np.linalg.svd(H) # singular value decompostion  u ,vt contain torational infomation
        R = Vt.T @ U.T # rotation matrix 
        if np.linalg.det(R) < 0:  # safety check  might be a mirror image 
            Vt[-1] *= -1
            R = Vt.T @ U.T
        t = rob_pos.mean(0) - R @ cam_pos.mean(0) # translation

        pos_errors = [np.linalg.norm(R @ np.array(c) + t - np.array(r)) * 1000
                     for c, r in zip(self.camera_points, self.robot_points)]

        cam_angles = np.array(self.camera_angles)
        rob_angles = np.array(self.robot_angles)
        # Unwrap angles to avoid ±180° jumps
        angle_offsets = rob_angles - cam_angles
        angle_offsets = np.arctan2(np.sin(angle_offsets), np.cos(angle_offsets))  # Wrap to ±π
        angle_offset = np.mean(angle_offsets)  # average of offset 
        angle_errors = np.abs(angle_offsets - angle_offset)   # checking

        print(f'\nPosition: {np.mean(pos_errors):.2f}mm mean')
        print(f'Rotation: J3 {np.degrees(np.min(rob_angles)):.1f}° to {np.degrees(np.max(rob_angles)):.1f}° (span {np.degrees(np.max(rob_angles)-np.min(rob_angles)):.1f}°)')
        print(f'Offset: {np.degrees(angle_offset):.2f}°, Error: {np.degrees(np.mean(angle_errors)):.2f}°')

        data = {
            'num_points': ok,
            'position': {
                'rotation_matrix': R.tolist(),
                'translation_vector': t.tolist(),
                'mean_error_mm': float(np.mean(pos_errors)),
                'max_error_mm': float(np.max(pos_errors)),
                'std_error_mm': float(np.std(pos_errors))
            },
            'rotation': {
                'angle_offset_rad': float(angle_offset),
                'angle_offset_deg': float(np.degrees(angle_offset)),
                'mean_error_deg': float(np.degrees(np.mean(angle_errors))),
                'max_error_deg': float(np.degrees(np.max(angle_errors))),
                'std_error_deg': float(np.degrees(np.std(angle_errors))),
                'j3_span_deg': float(np.degrees(np.max(rob_angles) - np.min(rob_angles)))
            }
        }

        with open('calibration_30points_with_rotation.json', 'w') as f:
            json.dump(data, f, indent=2)

        print('\nSaved!\n')

if __name__ == '__main__':
    try:
        cal = SafeCalibration()
        cal.run()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
