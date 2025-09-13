#!/usr/bin/python3

import os
import rclpy
import numpy as np
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from geometry_msgs.msg import Twist, Point
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
        
        self.kp_linear = 4.0
        self.kp_angular = 8.0

        self.save_dir = os.path.expanduser('~/multiverse-mission-ioon-beam/src/exam1')
        self.turtle_file_map = {
            "Foxy": 1,
            "Noetic": 2,
            "Humble": 3,
            "Iron": 4
        }

        self.declare_parameter('sampling_frequency', 100.0)
        self.frequency = self.get_parameter('sampling_frequency').value
        self.declare_parameter('tolerance', 0.1)
        self.tolerance = self.get_parameter('tolerance').value

                # Declare dynamic parameters with descriptors
        kp_linear_descriptor = ParameterDescriptor(
            description='Linear proportional gain for turtle control'
        )
        kp_angular_descriptor = ParameterDescriptor(
            description='Angular proportional gain for turtle control'
        )
        max_pizzas_descriptor = ParameterDescriptor(
            description='Maximum number of pizzas allowed'
        )
        
        self.declare_parameter('kp_linear', 4.0, kp_linear_descriptor)
        self.declare_parameter('kp_angular', 8.0, kp_angular_descriptor)
        self.declare_parameter('max_pizzas', 10, max_pizzas_descriptor)
        
        # Get initial parameter values
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_pizzas = self.get_parameter('max_pizzas').value
        
        # Set up parameter callback for dynamic parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, f'turtle_status', 10)
        self.transition_pose_pub = self.create_publisher(Point, '/transition_pose', 10)
        self.release_eraser_turtle_pub = self.create_publisher(Bool, '/release_eraser_turtle', 10)
        self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.create_subscription(Point, f'/{self.turtlesim_plus2_namespace}/mouse_position', self.mouse_position_callback, 10)

        self.other_turtle_names = ["Foxy", "Noetic", "Humble", "Iron"]
        self.other_turtle_status = {}
        for name in self.other_turtle_names:
            if name != self.turtle2_namespace: 
                self.create_subscription(String, f'/{self.turtlesim_plus2_namespace}/{name}/turtle_status', 
                                       lambda msg, name=name: self.status_callback(name, msg), 10)

        self.control_timer = self.create_timer(1/self.frequency, self.timer_callback)
        self.yaml_check_timer = self.create_timer(0.5, self.check_yaml_files)

        self.spawn_turtle_client = self.create_client(Spawn, f'/{self.turtlesim_plus2_namespace}/spawn_turtle')
        self.spawn_pizza_client = self.create_client(GivePosition, f'/{self.turtlesim_plus2_namespace}/spawn_pizza')

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

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

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
                self.cmdvel(0.0, 0.0)
                self.get_logger().info(f"{self.turtle2_namespace} has no more tasks, waiting for others")
                return
            
            # Execute current YAML task
            target = self.turtle_queue[0]
            self.move_to_target(target)

            return

        # Phase 2: Wait for all turtles to complete
        if self.turtle_completed and not self.all_turtles_completed:
            self.cmdvel(0.0, 0.0)
            return

        # Phase 3: All turtles completed - follow mouse
        if self.all_turtles_completed and self.mouse_pose is not None:
            target = self.mouse_pose
            self.move_to_target(target)
            return

    def move_to_target(self, target):
        pose = self.robot_pose
        diff_x = target['x'] - pose[0]
        diff_y = target['y'] - pose[1]
        d = np.sqrt(diff_x**2 + diff_y**2)
        theta_d = np.arctan2(diff_y, diff_x)
        e_theta = theta_d - pose[2]
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        vx = self.kp_linear * d
        w = self.kp_angular * e_theta
        self.cmdvel(vx, w)

        if d < self.tolerance and abs(e_theta) < self.tolerance:
            if not self.turtle_completed:
                self.spawn_pizza(target['x'], target['y'])
                self.turtle_queue.pop(0)
                self.release_eraser_turtle(False)
                self.get_logger().info(f"{self.turtle2_namespace} reached position!")
            elif self.turtle_completed:
                self.cmdvel(0.0, 0.0)
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
    
    def parameter_callback(self, params):
        successful = True
        for param in params:
            if param.name == 'kp_linear':
                if param.type_ == Parameter.Type.DOUBLE:
                    old_value = self.kp_linear
                    self.kp_linear = param.value
                    self.get_logger().info(f'kp_linear changed from {old_value} to {param.value}')
                else:
                    successful = False
            elif param.name == 'kp_angular':
                if param.type_ == Parameter.Type.DOUBLE:
                    old_value = self.kp_angular
                    self.kp_angular = param.value
                    self.get_logger().info(f'kp_angular changed from {old_value} to {param.value}')
                else:
                    successful = False  
            elif param.name == 'max_pizzas':
                if param.type_ == Parameter.Type.INTEGER:
                    old_value = self.max_pizzas
                    self.max_pizzas = param.value
                    self.get_logger().info(f'max_pizzas changed from {old_value} to {param.value}')
                else:
                    successful = False
            else:
                self.get_logger().warn(f'Unknown or invalid parameter: {param.name}')
                successful = False

        return SetParametersResult(successful=successful)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCopy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()