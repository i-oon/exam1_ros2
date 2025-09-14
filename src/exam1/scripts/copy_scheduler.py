#!/usr/bin/python3

import os
import rclpy
import numpy as np
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_msgs.msg import String, Bool
from turtlesim.srv import Spawn


class TurtleCopy(Node):
    def __init__(self):
        super().__init__('copy')
        
        self.turtle_name = self.get_namespace().strip('/')
        parts = self.turtle_name.split("/")
        self.turtlesim_plus2_namespace = parts[0] if parts else ""
        self.turtle2_namespace = parts[1] if len(parts) > 1 else ""
        
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.mouse_pose = None
        self.target = None
        self.turtle_queue = []
        self.turtle_completed = False
        self.final_pos_reached = False
        self.all_turtles_completed = False
        self.yaml_files_ready = False
        self.initialized = False 
        self.target_reached = False

        self.save_dir = os.path.expanduser('~/multiverse-mission-ioon-beam/src/exam1')
        self.turtle_file_map = {
            "Foxy": 1,
            "Noetic": 2,
            "Humble": 3,
            "Iron": 4
        }

        self.declare_parameter('sampling_frequency', 100.0)
        self.frequency = self.get_parameter('sampling_frequency').value

        # Publishers to local controller (same namespace)
        self.target_pub = self.create_publisher(Point, 'controller/target', 10)
        self.mode_pub = self.create_publisher(String, 'controller/mode', 10)
        
        # Original publishers
        self.status_pub = self.create_publisher(String, f'turtle_status', 10)
        self.transition_pose_pub = self.create_publisher(Point, '/transition_pose', 10)
        self.release_eraser_turtle_pub = self.create_publisher(Bool, '/release_eraser_turtle', 10)
        
        # Subscribers
        self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.create_subscription(Point, f'/{self.turtlesim_plus2_namespace}/mouse_position', self.mouse_position_callback, 10)
        self.create_subscription(String, 'controller/status', self.controller_status_callback, 10)

        # Status tracking for other turtles
        self.other_turtle_names = ["Foxy", "Noetic", "Humble", "Iron"]
        self.other_turtle_status = {}
        for name in self.other_turtle_names:
            if name != self.turtle2_namespace: 
                self.create_subscription(String, f'/{self.turtlesim_plus2_namespace}/{name}/turtle_status', 
                                       lambda msg, name=name: self.status_callback(name, msg), 10)

        self.control_timer = self.create_timer(1/50.0, self.timer_callback)  # Fixed frequency
        self.yaml_check_timer = self.create_timer(0.5, self.check_yaml_files)

        self.spawn_turtle_client = self.create_client(Spawn, f'/{self.turtlesim_plus2_namespace}/spawn_turtle')
        self.spawn_pizza_client = self.create_client(GivePosition, f'/{self.turtlesim_plus2_namespace}/spawn_pizza')

    def controller_status_callback(self, msg):
        """Handle status updates from local controller"""
        if msg.data == 'target_reached':
            self.target_reached = True

    def send_target(self, x, y):
        """Send target position to local controller"""
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        self.target_pub.publish(msg)

    def send_mode(self, mode):
        """Send control mode to local controller"""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    def check_yaml_files(self):  
        if self.yaml_files_ready:
            return
        
        if self.turtle2_namespace not in self.turtle_file_map:
            self.get_logger().error(f"Unknown turtle name: {self.turtle2_namespace}")
            return
        
        # Check for all 4 YAML files before any turtle starts
        required_files = [f'saved_pizzas_{i}.yaml' for i in range(1, 5)]
        
        all_files_exist = True
        missing_files = []
        
        for required_file in required_files:
            file_path = os.path.join(self.save_dir, required_file)
            if not os.path.exists(file_path):
                all_files_exist = False
                missing_files.append(required_file)
        
        if all_files_exist:
            self.get_logger().info(f"All YAML files detected!")
            self.yaml_files_ready = True
            self.load_positions()
            self.initialized = True
            # Stop the YAML checking timer
            self.yaml_check_timer.cancel()
    
    def load_positions(self):
        if self.turtle2_namespace not in self.turtle_file_map:
            self.get_logger().error(f"Unknown turtle name: {self.turtle2_namespace}")
            return
        
        file_num = self.turtle_file_map[self.turtle2_namespace]
        file_path = os.path.join(self.save_dir, f'saved_pizzas_{file_num}.yaml')
        
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
                self.turtle_queue = data['pizzas'].copy()
                self.get_logger().info(f"Loaded {len(data['pizzas'])} positions for {self.turtle2_namespace}")
                self.get_logger().info(f"{self.turtle2_namespace} Queue: {self.turtle_queue}")
        except Exception as e:
            self.get_logger().error(f"Failed to load positions for {self.turtle2_namespace}: {str(e)}")

    def status_callback(self, turtle_name, msg):
        self.other_turtle_status[turtle_name] = msg.data

    def timer_callback(self):
        if not self.initialized:
            return

        # Publish this turtle's status
        status_msg = String()
        if self.turtle_completed:
            status_msg.data = "tasks_completed"
        else:
            status_msg.data = "working"
        self.status_pub.publish(status_msg)

        # Check if all turtles completed their tasks
        all_completed = self.turtle_completed and all(
            status in ["tasks_completed"] 
            for status in self.other_turtle_status.values()
        )
        
        if all_completed and not self.all_turtles_completed:
            self.all_turtles_completed = True
            self.get_logger().info(f"{self.turtle2_namespace}: All turtles completed! Now following mouse position")

        # Phase 1: Execute YAML tasks
        if not self.turtle_completed:
            if not self.turtle_queue:
                # No more tasks, mark as completed
                self.turtle_completed = True
                self.send_mode('stop')  # Stop local controller
                self.get_logger().info(f"{self.turtle2_namespace} has no more tasks, waiting for others")
                return
            
            # Execute current YAML task
            target = self.turtle_queue[0]
            self.move_to_target(target)
            return

        # Phase 2: Wait for all turtles to complete
        if self.turtle_completed and not self.all_turtles_completed:
            self.send_mode('stop')  # Stop local controller
            return

        # Phase 3: All turtles completed - follow mouse
        if self.all_turtles_completed and self.mouse_pose is not None:
            target = self.mouse_pose
            self.move_to_target(target)
            return

    def move_to_target(self, target):
        # Send target to local controller and wait for completion
        if not self.target_reached:
            self.send_target(target['x'], target['y'])
        
        # Check if controller reached the target
        if self.target_reached:
            if not self.turtle_completed:
                self.spawn_pizza(target['x'], target['y'])
                self.turtle_queue.pop(0)
                self.release_eraser_turtle(False)
                self.target_reached = False
                self.get_logger().info(f"{self.turtle2_namespace} reached position!")
            elif self.turtle_completed:
                self.send_mode('stop')  # Stop local controller
                self.release_eraser_turtle(True)
                msg = Point()
                msg.x = target['x']
                msg.y = target['y']
                self.transition_pose_pub.publish(msg)

    def release_eraser_turtle(self, status):
        msg = Bool()
        msg.data = status
        self.release_eraser_turtle_pub.publish(msg)
        
    def pose_callback(self, msg):
        self.robot_pose = np.array([msg.x, msg.y, msg.theta])

    def mouse_position_callback(self, msg):
        self.mouse_pose = {'x': msg.x, 'y': msg.y}

    def spawn_pizza(self, x, y):
        if not self.spawn_pizza_client.service_is_ready():
            self.get_logger().warn("Spawn service not available")
            return

        req = GivePosition.Request()
        req.x = x
        req.y = y
        
        future = self.spawn_pizza_client.call_async(req)
        future.add_done_callback(lambda future: self.pizza_spawned_callback(future, x, y))

    def pizza_spawned_callback(self, future, x, y):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f'Failed to spawn pizza: {str(e)}')
    

def main(args=None):
    rclpy.init(args=args)
    node = TurtleCopy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()