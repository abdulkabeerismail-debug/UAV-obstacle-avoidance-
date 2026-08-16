import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import math
import random

# --- RRT Node Class ---
class RRTNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class DroneVisionControl(Node):
    def __init__(self):
        super().__init__('drone_vision_control')
        # Subscriptions and Publishers
        self.subscription = self.create_subscription(Image, '/camera', self.listener_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        
        # YOLO Model loaded from your directory
        self.model = YOLO('/home/macayalwrist/fyp_drone/weights/multi_obstacle_best.pt')

        # FSM and RRT Tracking Variables
        self.flight_phase = "TAKEOFF"
        self.frame_counter = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.rrt_path = []
        self.current_waypoint_index = 0

    # Callback to update the drone's real-time coordinates
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    # RRT Path Generation Algorithm
    def compute_rrt_path(self, start_pos, goal_pos, obstacle_pos, safe_radius):
        print("Computing RRT Path...")
        tree = [RRTNode(start_pos[0], start_pos[1])]
        goal = RRTNode(goal_pos[0], goal_pos[1])
        
        max_iter = 500
        step_size = 0.5
        
        for _ in range(max_iter):
            # Randomly sample space
            rand_x = random.uniform(start_pos[0] - 5, goal_pos[0] + 5)
            rand_y = random.uniform(start_pos[1] - 5, goal_pos[1] + 5)
            
            nearest_node = min(tree, key=lambda n: math.hypot(n.x - rand_x, n.y - rand_y))
            theta = math.atan2(rand_y - nearest_node.y, rand_x - nearest_node.x)
            
            new_node = RRTNode(nearest_node.x + step_size * math.cos(theta),
                               nearest_node.y + step_size * math.sin(theta))
            new_node.parent = nearest_node
            
            # Check collision against the red brick
            dist_to_obs = math.hypot(new_node.x - obstacle_pos[0], new_node.y - obstacle_pos[1])
            if dist_to_obs >= safe_radius:
                tree.append(new_node)
                
                if math.hypot(new_node.x - goal.x, new_node.y - goal.y) < step_size:
                    goal.parent = new_node
                    tree.append(goal)
                    break
                    
        # Extract path
        path = []
        current = goal
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
            
        path.reverse()
        return path

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, verbose=False)
        cmd = Twist()
        self.frame_counter += 1
        
        # YOLO Detection Logic
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                conf = box.conf[0]

                if conf > 0.90:
                    box_width = x2 - x1
                    box_height = y2 - y1
                    box_area = box_width * box_height
                    
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                    cv2.putText(frame, f"Area: {int(box_area)}", (int(x1), int(y1)-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    # Trigger RRT Calculation 
                    if box_area > 50000 and self.flight_phase == "CRUISE":
                        self.flight_phase = "CALCULATE_RRT"
                        self.frame_counter = 0 

        # --- UPDATED FSM ---
        if self.flight_phase == "TAKEOFF":
            cmd.linear.z = 0.25  # Lowered climb speed to stay locked on the brick
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if self.frame_counter > 60: # Reduced timer to stop the climb earlier
                self.flight_phase = "CRUISE"
                
        elif self.flight_phase == "CRUISE":
            cmd.linear.x = 0.5
            cmd.linear.z = 0.0  
            cmd.angular.z = 0.0

        elif self.flight_phase == "CALCULATE_RRT":
            # Halt the drone instantly
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0
            
            # Map the obstacle (Approximating it is 2 meters directly ahead)
            obstacle_x = self.current_x + 2.0 
            obstacle_y = self.current_y 
            
            # Goal is safely past the obstacle
            goal_x = obstacle_x + 3.0 
            goal_y = obstacle_y
            
            # Generate the tree
            self.rrt_path = self.compute_rrt_path((self.current_x, self.current_y), (goal_x, goal_y), (obstacle_x, obstacle_y), safe_radius=1.5)
            
            if len(self.rrt_path) > 1:
                self.current_waypoint_index = 1 # Skip current position
                self.flight_phase = "FOLLOW_RRT_PATH"
            else:
                print("RRT Failed! Commencing Emergency Landing.")
                self.flight_phase = "LAND"

        elif self.flight_phase == "FOLLOW_RRT_PATH":
            # Dynamic waypoint tracking using proportional control
            if self.current_waypoint_index < len(self.rrt_path):
                target_x, target_y = self.rrt_path[self.current_waypoint_index]
                
                err_x = target_x - self.current_x
                err_y = target_y - self.current_y
                distance_to_target = math.hypot(err_x, err_y)
                
                if distance_to_target > 0.3:
                    cmd.linear.x = 0.4 * (err_x / distance_to_target)
                    cmd.linear.y = 0.4 * (err_y / distance_to_target)
                    cmd.linear.z = 0.0  # STRICTLY LOCK ALTITUDE to prevent floating up
                else:
                    self.current_waypoint_index += 1 # Move to next node in the tree
            else:
                print("Obstacle Cleared. Resuming Cruise.")
                self.flight_phase = "CRUISE"

        elif self.flight_phase == "LAND":
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = -0.2 
            cmd.angular.z = 0.0

        self.publisher_.publish(cmd)
        cv2.imshow("Autonomous Perception & Control", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DroneVisionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
