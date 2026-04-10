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
        
        # Standard Tello driver topics (no simulator prefix)
        self.camera_topic = '/image_raw'
        self.cmd_vel_topic = '/cmd_vel'
        
        self.subscription = self.create_subscription(Image, self.camera_topic, self.image_callback, qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.bridge = CvBridge()

        model_path = os.path.expanduser('~/ros2_ws/best.pt')
        self.model = YOLO(model_path)
        
        self.GATE_CLASS_ID = 0
        self.STOP_SIGN_CLASS_ID = 1

        # Physical offset for Tello camera
        self.y_offset = 0

        # Control gains for real-world physics (PD controller)
        self.kp_yaw = 0.0010
        self.kd_yaw = 0.0005  # Derivative term to prevent oscillation around the center
        
        self.kp_z = 0.002
        self.kd_z = 0.001
        
        # Variables for D-controller (previous frame errors)
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        self.state = 'INITIALIZING'
        self.is_landing = False
        
        # Time-based blind fly logic
        self.is_blind_flying = False
        self.blind_fly_start_time = 0.0
        self.blind_fly_duration = 3.5  # Seconds to fly forward through the gate
        
        self.alignment_counter = 0
        self.REQUIRED_ALIGNMENT_FRAMES = 8

        self.get_logger().info("Autonomy node started!")
        self.takeoff_drone()

    def takeoff_drone(self):
        os.system("ros2 service call /tello_action tello_msgs/TelloAction \"{cmd: 'takeoff'}\" &")
        # After takeoff, it will naturally fall into the 'SEARCHING' state if no target is found
        self.state = 'TAKEOFF'

    def image_callback(self, msg):
        if self.is_landing:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        # Calculate dynamic resolution and center points
        height, width, _ = cv_image.shape
        dynamic_center_x = width // 2

        target_y = height // 2

        results = self.model(cv_image, verbose=False)
        annotated_frame = results[0].plot()
        cmd = Twist()

        # Time-based forward flight (Blind fly through gate)
        if self.is_blind_flying:
            if (time.time() - self.blind_fly_start_time) < self.blind_fly_duration:
                cmd.linear.x = 0.35   # Speed for flying through the gate
                cmd.linear.z = 0.0    # Maintain current altitude
                self.state = 'COMMIT_FLY_THROUGH'
                self.cmd_vel_pub.publish(cmd)
                self.display_ui(annotated_frame)
                return
            else:
                self.is_blind_flying = False
                self.state = 'SEARCHING'
                self.alignment_counter = 0

        best_gate, max_gate_area = None, 0
        best_stop_sign, max_stop_area = None, 0

        # Find the largest gate and largest stop sign in the frame
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            x_center, y_center, box_width, box_height = box.xywh[0].tolist()
            area = box_width * box_height
            
            if class_id == self.STOP_SIGN_CLASS_ID:
                if area > max_stop_area:
                    max_stop_area, best_stop_sign = area, (x_center, y_center, area)
            elif class_id == self.GATE_CLASS_ID:
                if area > max_gate_area:
                    max_gate_area, best_gate = area, (x_center, y_center, area)

        target, is_target_stop_sign = None, False
        if best_stop_sign and max_stop_area > 35000: 
            target, is_target_stop_sign = best_stop_sign, True
        elif best_gate:
            target = best_gate
        elif best_stop_sign:
            target, is_target_stop_sign = best_stop_sign, True

        # Target found: Align and navigate
        if target is not None:
            x_center, y_center, area = target
            
            adjusted_y_center = y_center + 250
            
            error_x = dynamic_center_x - x_center
            error_y = target_y - adjusted_y_center

            # PD-Controller
            der_x = error_x - self.prev_error_x
            der_y = error_y - self.prev_error_y

            if abs(error_x) > 25:
                cmd.angular.z = float((self.kp_yaw * error_x) + (self.kd_yaw * der_x))
            
            if abs(error_y) > 25:
                cmd.linear.z = float((self.kp_z * error_y) + (self.kd_z * der_y))
            
            # Update previous errors for the next frame
            self.prev_error_x = error_x
            self.prev_error_y = error_y

            # Clamp velocities to safe limits for real Tello
            cmd.angular.z = max(min(cmd.angular.z, 0.20), -0.20)
            cmd.linear.z = max(min(cmd.linear.z, 0.25), -0.25)

            if is_target_stop_sign:
                if area > 130000:
                    self.state = 'LANDING'
                    self.cmd_vel_pub.publish(Twist()) 
                    self.land_drone()
                else:
                    self.state = 'APPROACHING_STOP'
                    cmd.linear.x = 0.10 if abs(error_x) < 60 else 0.0
            else:
                if area > 10000:
                    if abs(error_x) < 40 and abs(error_y) < 40:
                        self.alignment_counter += 1
                        self.state = f'SETTLING ({self.alignment_counter})'
                        cmd.linear.x = 0.0  
                        
                        if self.alignment_counter >= self.REQUIRED_ALIGNMENT_FRAMES:
                            self.get_logger().info("Centered. Committing!")
                            self.is_blind_flying = True
                            self.blind_fly_start_time = time.time() 
                    else:
                        self.alignment_counter = 0
                        self.state = 'STRICT_ALIGNMENT'
                        cmd.linear.x = 0.02 
                else:
                    self.alignment_counter = 0
                    self.state = 'NAVIGATING'
                    cmd.linear.x = 0.15 if abs(error_x) < 60 else 0.05
        
        # No target found: Spin to search
        else:
            self.alignment_counter = 0
            self.state = 'SEARCHING'
            # Yaw rotation to find the next target (positive is usually counter-clockwise)
            cmd.angular.z = 0.4  

        if not self.is_landing:
            self.cmd_vel_pub.publish(cmd)

        self.display_ui(annotated_frame)

    def display_ui(self, frame):
        cv2.putText(frame, f"State: {self.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Tello Autonomy", frame)
        cv2.waitKey(1)

    def land_drone(self):
        self.is_landing = True
        os.system("ros2 service call /tello_action tello_msgs/TelloAction \"{cmd: 'land'}\" &")

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
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
