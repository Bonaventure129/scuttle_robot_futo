#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage 
from geometry_msgs.msg import Point
from std_msgs.msg import String 
import cv2
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # --- Parameters ---
        self.target_class = "phone" 
        self.conf_threshold = 0.5
        
        # --- Performance Tuning ---
        # Kept at 5 to prevent the PyTorch model from completely freezing the Pi 4
        self.process_every_n_frames = 5  
        self.frame_counter = 0
        self.ai_resolution = (320, 240) 
        
        # --- AI Model ---
        self.get_logger().info(f"Loading standard YOLOv8n... (Target: {self.target_class})")
        # CHANGED: Reverted to the standard PyTorch .pt model
        self.model = YOLO("yolov8n.pt") 
        self.bridge = CvBridge()
        
        # --- Subscriptions ---
        self.sub = self.create_subscription(
            Image, 
            '/image_raw', 
            self.image_callback, 
            10
        )
        
        # --- Publishers ---
        self.pub_center = self.create_publisher(Point, '/object_center', 10)
        self.pub_name = self.create_publisher(String, '/object_name', 10) 
        self.pub_image = self.create_publisher(CompressedImage, '/yolo/image_result/compressed', 10)

    def image_callback(self, msg):
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n_frames != 0:
            return

        try:
            # 1. Convert standard ROS Image -> OpenCV Image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        if frame is None:
            return

        # 2. Resize for Speed 
        small_frame = cv2.resize(frame, self.ai_resolution)
        height, width, _ = small_frame.shape
        
        # 3. Run AI Inference
        results = self.model(small_frame, verbose=False, conf=self.conf_threshold)
        
        best_target = None
        closest_to_center = 1.0 

        # 4. Process and Draw Boxes
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                
                # Extract Box Coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Default box color is Green
                color = (0, 255, 0)

                # Target calculation and Red highlighting
                if class_name == self.target_class:
                    color = (0, 0, 255) # Red for target
                    cx = int((x1 + x2) / 2)
                    norm_x = (cx - (width / 2)) / (width / 2)
                    
                    if abs(norm_x) < closest_to_center:
                        closest_to_center = abs(norm_x)
                        best_target = box

                # Draw the bounding box and label directly onto the frame
                cv2.rectangle(small_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(small_frame, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 5. Publish Navigation Targets
        if best_target is not None:
            x1, y1, x2, y2 = map(int, best_target.xyxy[0])
            cx = int((x1 + x2) / 2)
            cls_id = int(best_target.cls[0])
            name = self.model.names[cls_id]
            
            # Publish Name 
            name_msg = String()
            name_msg.data = name
            self.pub_name.publish(name_msg)
            
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

        # 6. Publish the Annotated Image as a Compressed JPEG
        try:
            img_msg = CompressedImage()
            img_msg.header = msg.header # Preserve the original camera timestamp and frame_id
            img_msg.format = "jpeg"
            
            # Manually encode to JPEG with quality 40 to save CPU
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
            success, encoded_image = cv2.imencode('.jpg', small_frame, encode_param)
            
            if success:
                img_msg.data = encoded_image.tobytes()
                self.pub_image.publish(img_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error publishing compressed image: {e}")

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