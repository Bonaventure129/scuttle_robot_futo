#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from std_msgs.msg import String  # Needed for Voice Command
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # --- Parameters ---
        # Change this to what you want to track: 'bottle', 'cup', 'cell phone', etc.
        self.target_class = "person" 
        self.conf_threshold = 0.5
        
        # --- Performance Tuning ---
        self.process_every_n_frames = 3
        self.frame_counter = 0
        self.ai_resolution = (320, 240)
        
        # --- AI Model ---
        self.get_logger().info(f"Loading YOLOv8n... (Target: {self.target_class})")
        self.model = YOLO("yolov8n.pt")
        
        # --- ROS Setup ---
        self.bridge = CvBridge()
        
        # Subscribe to COMPRESSED image (Fixes colors & Lag)
        self.sub = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.image_callback, 
            10
        )
        
        # Publishers
        self.pub_center = self.create_publisher(Point, '/object_center', 10)
        self.pub_debug_img = self.create_publisher(Image, '/object_debug', 10)
        self.pub_name = self.create_publisher(String, '/object_name', 10) # For Voice Node

    def image_callback(self, msg):
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        try:
            # Decode Compressed Image (Fixes Pink/Green issue)
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Decoding Error: {e}")
            return

        if frame is None:
            return

        # Resize for Speed
        small_frame = cv2.resize(frame, self.ai_resolution)
        height, width, _ = small_frame.shape
        
        # Run AI Inference
        results = self.model(small_frame, verbose=False, conf=self.conf_threshold)
        
        best_target = None
        closest_to_center = 1.0 

        # Find the object closest to the center
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                
                if class_name == self.target_class:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = int((x1 + x2) / 2)
                    norm_x = (cx - (width / 2)) / (width / 2)
                    
                    if abs(norm_x) < closest_to_center:
                        closest_to_center = abs(norm_x)
                        best_target = box

        if best_target is not None:
            # Extract Box Coordinates
            x1, y1, x2, y2 = map(int, best_target.xyxy[0])
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            
            # Get the Name
            cls_id = int(best_target.cls[0])
            name = self.model.names[cls_id]
            
            # Publish Name for Voice Node
            name_msg = String()
            name_msg.data = name
            self.pub_name.publish(name_msg)

            # Draw Box & Label
            label = f"LOCKED: {name}"
            cv2.rectangle(small_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(small_frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(small_frame, label, (x1, y1-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Publish Position
            norm_x = (cx - (width / 2)) / (width / 2)
            obj_area = (x2 - x1) * (y2 - y1)
            norm_size = obj_area / (width * height)

            point_msg = Point()
            point_msg.x = float(norm_x)
            point_msg.y = float(norm_size)
            point_msg.z = 1.0 
            self.pub_center.publish(point_msg)
        else:
            point_msg = Point()
            point_msg.z = 0.0
            self.pub_center.publish(point_msg)

        # Publish Debug Image
        self.pub_debug_img.publish(self.bridge.cv2_to_imgmsg(small_frame, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()