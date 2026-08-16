import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time

class AutoDatasetGen(Node):
    def __init__(self):
        super().__init__('auto_dataset_gen')
        self.subscription = self.create_subscription(Image, '/camera', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.count = 0
        self.img_dir = os.path.expanduser('~/fyp_drone/dataset/images/train')
        self.lbl_dir = os.path.expanduser('~/fyp_drone/dataset/labels/train')
        self.last_save = time.time()
        self.get_logger().info("Auto-Dataset Generator Started! Move the drone/obstacle around.")

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        
        # Detect red obstacle using color thresholding
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        
        mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        annotated_frame = frame.copy()
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500: # Ignore tiny noise
                x, y, bw, bh = cv2.boundingRect(c)
                
                # Draw preview bounding box
                cv2.rectangle(annotated_frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                
                # Save frame every 0.5 seconds up to 50 samples
                if time.time() - self.last_save > 0.5 and self.count < 50:
                    img_name = f"red_barrier_{self.count:03d}"
                    cv2.imwrite(os.path.join(self.img_dir, f"{img_name}.jpg"), frame)
                    
                    # Convert to normalized YOLO format: class_id x_center y_center width height
                    x_center = (x + bw / 2.0) / w
                    y_center = (y + bh / 2.0) / h
                    norm_bw = bw / float(w)
                    norm_bh = bh / float(h)
                    
                    with open(os.path.join(self.lbl_dir, f"{img_name}.txt"), "w") as f:
                        f.write(f"0 {x_center:.6f} {y_center:.6f} {norm_bw:.6f} {norm_bh:.6f}\n")
                    
                    self.count += 1
                    self.last_save = time.time()
                    self.get_logger().info(f"Saved sample {self.count}/50")

        cv2.putText(annotated_frame, f"Captured: {self.count}/50", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Auto-Dataset Collector", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDatasetGen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
