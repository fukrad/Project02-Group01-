import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image  # for ros 
from std_msgs.msg import String    # for ros 
from cv_bridge import CvBridge  # between ros and opencv 
from ultralytics import YOLO
import json
import open3d as o3d

MODEL_PATH = '/root/project2/yolov8n_fair.pt'
CONF_THRESHOLD = 0.5


class YOLODetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)

        print("Loading YOLO...")
        self.model = YOLO(MODEL_PATH)
        dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)
        _ = self.model(dummy_img, conf=0.5, verbose=False)

        self.class_names = {0: 'pen', 1: 'usb', 2: 'top'}
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None

        self.detection_pub = rospy.Publisher('/object_detections', String, queue_size=10)

        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback, queue_size=1)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback, queue_size=1)

        self.fx = 616.0
        self.fy = 616.0
        self.cx = 320.0
        self.cy = 240.0

        for i in range(30):
            if self.color_image is not None:
                break
            rospy.sleep(1.0)

        if self.color_image is None:
            print("Camera timeout!")
            exit(1)


    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def get_3d_position(self, x, y, depth_mm): # px to camera x y z # Converts 2D pixel coordinates and depth to 3D camera coordinates
        if depth_mm == 0:
            return None
        Z = depth_mm / 1000.0
        X = (x - self.cx) * Z / self.fx
        Y = (y - self.cy) * Z / self.fy
        return [X, Y, Z]

    def compute_angle_pcl(self, x1, y1, x2, y2, depth_mm):  # point cloud generation
        points_3d = []
        for py in range(y1, y2, 2):
            for px in range(x1, x2, 2):
                if 0 <= py < depth_mm.shape[0] and 0 <= px < depth_mm.shape[1]:
                    d = depth_mm[py, px]
                    if d > 0:
                        pos = self.get_3d_position(px, py, d)
                        if pos:
                            points_3d.append(pos)

        if len(points_3d) < 10:
            return None

        pcd = o3d.geometry.PointCloud()   # Create an Open3D PointCloud object
        pcd.points = o3d.utility.Vector3dVector(np.array(points_3d)) # Assign the collected 3D points to the PointCloud object.
 
        try:
            obb = pcd.get_oriented_bounding_box() # Calculate the Oriented Bounding Box (OBB) that tightly encloses the point cloud
        except:
            return None

        R = np.array(obb.R)    #Rotation Matrix (R)** which defines the OBB's orientation.
        extent = np.array(obb.extent) # Extract the **extent** (lengths of the sides) of the OBB along its principal axes.

        xy_projections = [] # List to store the projection metrics for each of the three axes.
        for i in range(3):   # 2d length  # Loop through the three principal axes of the OBB
            axis = R[:, i]
            xy_length = np.sqrt(axis[0]**2 + axis[1]**2) * extent[i]
            xy_projections.append(xy_length)

        best_idx = np.argmax(xy_projections)   # longest  # Find the index of the axis with the largest XY projection metric (the **main axis** lying horizontally).
        main_axis = R[:, best_idx]  # set as main  # Set this longest projected axis as the object's primary direction

        angle_rad = np.arctan2(-main_axis[1], main_axis[0])
        angle_deg = np.degrees(angle_rad)

        if angle_deg < 0:
            angle_deg += 180

        return angle_deg

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.color_image is None or self.depth_image is None:
                rate.sleep()
                continue

            frame = self.color_image.copy()
            depth_mm = self.depth_image.copy()

            results = self.model(frame, conf=CONF_THRESHOLD, verbose=False)

            detected = {}

            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    class_name = self.class_names.get(cls, 'unknown')

                    if class_name == 'unknown':
                        continue

                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    depth_val = depth_mm[cy, cx]
                    pos_3d = self.get_3d_position(cx, cy, depth_val)

                    if pos_3d is None:
                        continue

                    if class_name == 'top':
                        angle = None
                    else:
                        angle = self.compute_angle_pcl(x1, y1, x2, y2, depth_mm)

                    color = (0, 165, 255) if class_name == 'pen' else (255, 0, 0) if class_name == 'usb' else (0, 255, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                    if angle is not None:
                        length = 50
                        angle_rad = np.radians(angle)
                        end_x = int(cx + length * np.cos(angle_rad))
                        end_y = int(cy - length * np.sin(angle_rad))
                        cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), (0, 255, 255), 3, tipLength=0.3)

                    label = f"{class_name.upper()} {conf*100:.0f}%"
                    cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                    if conf >= 0.50:
                        if angle is not None:
                            pos_text = f"({pos_3d[0]:+.3f}, {pos_3d[1]:+.3f}, {pos_3d[2]:.3f}m) {angle:.1f}deg"
                        else:
                            pos_text = f"({pos_3d[0]:+.3f}, {pos_3d[1]:+.3f}, {pos_3d[2]:.3f}m)"
                        cv2.putText(frame, pos_text, (x1, y2+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                        detected[class_name] = {
                            'confidence': conf,
                            'position_3d': pos_3d,
                            'angle': angle
                        }

            detection_msg = String()
            detection_msg.data = json.dumps(detected)
            self.detection_pub.publish(detection_msg)

            cv2.imshow('YOLO Detection', frame)
            cv2.waitKey(1)

            rate.sleep()

        cv2.destroyAllWindows()

def main():
    try:
        detector = YOLODetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nStopped")

if __name__ == '__main__':
    main()
