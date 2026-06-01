import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2
from ultralytics import YOLO
import os
import time


class TelloVisionNode(Node):
    def __init__(self):
        super().__init__('tello_vision_node')

        self.camera_topic = '/image_raw'
        self.cmd_vel_topic = '/cmd_vel'

        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.lost_frame_counter = 0

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.bridge = CvBridge()

        self.model = YOLO('/work/best.pt')
        self.gate_count = 0
        self.max_gates = 4

        self.search_start = time.time()

        self.GATE_CLASS_ID = 0
        self.STOP_SIGN_CLASS_ID = 1

        self.camera_offset = 25

        # Control gains
        self.kp_forward = 0.70
        self.kp_yaw = 0.70
        self.kp_z = 0.75

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_d = 0.0

        self.target_size = 0.14
        self.prev_time = time.time()

        self.state = 'INITIALIZING'
        self.is_landing = False

        # Blind fly logic
        self.is_blind_flying = False
        self.blind_fly_start_time = 0.0
        self.blind_fly_duration = 5.5

        self.alignment_counter = 0
        self.REQUIRED_ALIGNMENT_FRAMES = 10

        self.get_logger().info("Autonomy node started!")
        self.takeoff_drone()

    def takeoff_drone(self):
        os.system(
            "ros2 service call /tello_action tello_msgs/TelloAction \"{cmd: 'takeoff'}\""
        )
        self.state = 'TAKEOFF'
        self.takeoff_time = time.time()

    def image_callback(self, msg):
        if self.is_landing:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        if self.state == 'TAKEOFF':
            elapsed_takeoff = time.time() - self.takeoff_time
        now = time.time()
        dt = max(now - self.prev_time, 1e-6)
        self.prev_time = now

        height, width, _ = cv_image.shape

        results = self.model(cv_image, verbose=False, imgsz=480)
        annotated_frame = results[0].plot()

        cmd = Twist()

        # -------------------------
        # BLIND FLY MODE
        # -------------------------
        if self.is_blind_flying:
            if (time.time() - self.blind_fly_start_time) < self.blind_fly_duration:
                cmd.linear.x = 0.45
                self.cmd_vel_pub.publish(cmd)
                self.display_ui(annotated_frame)
                return
            else:
                self.is_blind_flying = False
                self.gate_count += 1
                self.get_logger().info(f"Successfully cleared gate! Count: {self.gate_count}/{self.max_gates}")
                self.state = 'SEARCHING'
                self.search_start = time.time()
                self.alignment_counter = 0

        # -------------------------
        # DETECTION
        # -------------------------
        best_gate = None
        best_stop = None
        max_gate_area = 0.0
        max_stop_area = 0.0

        current_thresh = 0.25 if self.state == 'ALIGNING' else 0.55

        for box in results[0].boxes:
            class_id = int(box.cls[0])
            conf = float(box.conf[0])

            # Confidence filter
            if class_id == self.GATE_CLASS_ID and conf < current_thresh:
                continue
            if class_id == self.STOP_SIGN_CLASS_ID and conf < 0.55:
                continue

            x_center, y_center, bw, bh = box.xywh[0].tolist()
            area = bw * bh

            if class_id == self.GATE_CLASS_ID:
                if area > max_gate_area:
                    max_gate_area = area
                    best_gate = (x_center, y_center, bh, area)

            elif class_id == self.STOP_SIGN_CLASS_ID:
                if area > max_stop_area:
                    max_stop_area = area
                    best_stop = (x_center, y_center, bh, area)

        # -------------------------
        # TARGET SELECTION
        # -------------------------
        target = None
        is_stop = False

        if self.gate_count >= self.max_gates:
            if best_stop is not None:
                target = best_stop
                is_stop = True
        else:
            if best_gate is not None:
                target = best_gate
            elif best_stop is not None and max_stop_area > 35000:
                target = best_stop
                is_stop = True

        # -------------------------
        # CRITICAL FIX: no target = SEARCH only
        # -------------------------
        if target is None:
            self.lost_frame_counter += 1
            if self.lost_frame_counter < 5 and self.state == 'ALIGNING':
                cmd.linear.x = 0.15
                self.cmd_vel_pub.publish(cmd)
                return
            if self.state != 'SEARCHING':
                self.state = 'SEARCHING'
                self.search_start = time.time()
            elapsed = time.time() - self.search_start

            if elapsed < 4.0 : # 1.5 Second right rotation
                cmd.angular.z = -0.40
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
            elif elapsed < 7.0: # 1.5 Second left correction 
                cmd.angular.z = 0.30
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
            elif elapsed < 8.0: # 1.0 Second Flight Forward
                cmd.angular.z = 0.0   
                cmd.linear.x = 0.30
                cmd.linear.y = 0.0
            else:
                self.get_logger().info("Search pattern complete with no targets. Looping...")
                self.search_start = time.time()

            self.cmd_vel_pub.publish(cmd)
            self.display_ui(annotated_frame)
            return

        # -------------------------
        # CONTROL (ALIGNMENT)
        # -------------------------
        x, y, bh, area = target
        self.lost_frame_counter = 0
        self.search_start = time.time()

        error_x = (x - width / 2) / width
        error_y = (y - ((height / 2) - 150)) / height
        s = bh / height
        error_d = self.target_size - s

        # PD control (simple)
        cmd.angular.z = -self.kp_yaw * error_x
        cmd.linear.z = -self.kp_z * error_y
        cmd.linear.x = self.kp_forward * error_d

        # clamp
        cmd.angular.z = max(min(cmd.angular.z, 0.35), -0.35)
        cmd.linear.z = max(min(cmd.linear.z, 0.20), -0.20)
        cmd.linear.x = max(min(cmd.linear.x, 0.3), 0.0)

        # -------------------------
        # ALIGNMENT LOGIC
        # -------------------------
        if self.gate_count >= 2:
            DISTANCE_THRESHOLD = 65000
            self.blind_fly_duration = 3.0
        else:
            DISTANCE_THRESHOLD = 25000

        if abs(error_x) < 0.08 and abs(error_y) < 0.08:
            cmd.linear.x = self.kp_forward * error_d
            if area > DISTANCE_THRESHOLD:
                self.alignment_counter += 1
            else:
                self.alignment_counter = max(0, self.alignment_counter - 1)
        else:
            self.alignment_counter = max(0, self.alignment_counter - 1)
            if area < DISTANCE_THRESHOLD * 0.8:
                cmd.linear.x = 0.20
            else:
                cmd.linear.x = 0.0

        if self.alignment_counter >= self.REQUIRED_ALIGNMENT_FRAMES:
            if is_stop:
                cmd.linear.x = 0.15
                cmd.angular.z = 0.0
            else:
                self.is_blind_flying = True
                self.blind_fly_start_time = time.time()

        # -------------------------
        # STOP SIGN OVERRIDE
        # -------------------------
        if is_stop:
            if area > 100000:
                self.land_drone()
                return

        # -------------------------
        # SEND COMMAND
        # -------------------------
        self.state = 'ALIGNING'
        self.cmd_vel_pub.publish(cmd)
        self.display_ui(annotated_frame)

    def display_ui(self, frame):
        cv2.putText(frame, f"State: {self.state} | Gates: {self.gate_count}/{self.max_gates}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Tello Autonomy", frame)
        cv2.waitKey(1)

    def land_drone(self):
        self.is_landing = True
        os.system(
            "ros2 service call /tello_action tello_msgs/TelloAction \"{cmd: 'land'}\""
        )


def main(args=None):
    rclpy.init(args=args)
    node = TelloVisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
