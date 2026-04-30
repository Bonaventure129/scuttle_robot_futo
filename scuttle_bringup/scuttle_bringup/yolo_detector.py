#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage # We MUST use CompressedImage for Python
from geometry_msgs.msg import Point
from std_msgs.msg import String 
import cv2
import numpy as np
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # --- Parameters ---
        self.target_class = "phone" 
        self.conf_threshold = 0.5
        
        # --- Performance Tuning ---
        self.process_every_n_frames = 3
        self.frame_counter = 0
        self.ai_resolution = (320, 240) # Keep small for speed
        
        # --- AI Model ---
        self.get_logger().info(f"Loading YOLOv8n... (Target: {self.target_class})")
        self.model = YOLO("yolov8n.pt")
        
        # --- ROS Setup ---
        # NOTE: We keep this as CompressedImage. 
        # 'Theora' packets cannot be decoded by cv2.imdecode in Python.
        self.sub = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.image_callback, 
            10
        )
        
        # Publishers
        self.pub_center = self.create_publisher(Point, '/object_center', 10)
        self.pub_name = self.create_publisher(String, '/object_name', 10) 

    def image_callback(self, msg):
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        try:
            # 1. Decode JPEG (Compressed) -> OpenCV Image
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Decoding Error: {e}")
            return

        if frame is None:
            return

        # 2. Resize for Speed (Crucial for Pi 4)
        small_frame = cv2.resize(frame, self.ai_resolution)
        height, width, _ = small_frame.shape
        
        # 3. Run AI Inference
        results = self.model(small_frame, verbose=False, conf=self.conf_threshold)
        
        best_target = None
        closest_to_center = 1.0 

        # 4. Find the object closest to the center
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
            
            # Get the Name
            cls_id = int(best_target.cls[0])
            name = self.model.names[cls_id]
            
            # Publish Name for Voice Node
            name_msg = String()
            name_msg.data = name
            self.pub_name.publish(name_msg)
            
            # Publish Position (X and Size)
            norm_x = (cx - (width / 2)) / (width / 2)
            obj_area = (x2 - x1) * (y2 - y1)
            norm_size = obj_area / (width * height)

            point_msg = Point()
            point_msg.x = float(norm_x)
            point_msg.y = float(norm_size)
            point_msg.z = 1.0 
            self.pub_center.publish(point_msg)
            
            self.get_logger().info(f"Found {name} at x={norm_x:.2f}")

        else:
            point_msg = Point()
            point_msg.z = 0.0
            self.pub_center.publish(point_msg)

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