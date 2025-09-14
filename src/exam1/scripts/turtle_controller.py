#!/usr/bin/python3

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from std_msgs.msg import String


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        
        # Get namespace to determine which turtle this controller manages
        self.namespace = self.get_namespace().strip('/')
        self.get_logger().info(f"Controller initialized for namespace: {self.namespace}")
        
        # Control parameters - each controller instance has its own
        self.kp_linear = 4.0
        self.kp_angular = 8.0
        self.tolerance = 0.1
        self.frequency = 100.0
        
        # Declare parameters with descriptors
        self.declare_parameter('kp_linear', 4.0, ParameterDescriptor(description='Linear proportional gain'))
        self.declare_parameter('kp_angular', 8.0, ParameterDescriptor(description='Angular proportional gain'))
        self.declare_parameter('tolerance', 0.1, ParameterDescriptor(description='Target reaching tolerance'))
        self.declare_parameter('sampling_frequency', 100.0, ParameterDescriptor(description='Control loop frequency'))
        
        # Get parameter values
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.tolerance = self.get_parameter('tolerance').value
        self.frequency = self.get_parameter('sampling_frequency').value
        
        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # State variables for this turtle
        self.turtle_pose = np.array([0.0, 0.0, 0.0])
        self.current_target = {'x': 0.0, 'y': 0.0, 'active': False}
        self.control_mode = 'auto'  # 'auto' or 'manual'
        self.manual_command = {'linear': 0.0, 'angular': 0.0}
        
        # Publishers - relative to this node's namespace
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'controller/status', 10)
        
        # Subscribers - relative to this node's namespace  
        self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.create_subscription(Point, 'controller/target', self.target_callback, 10)
        self.create_subscription(Twist, 'controller/manual_cmd', self.manual_cmd_callback, 10)
        self.create_subscription(String, 'controller/mode', self.mode_callback, 10)
        
        # Control timer
        self.control_timer = self.create_timer(1/self.frequency, self.control_loop)
        
        self.get_logger().info(f"Single Turtle Controller ready for {self.namespace}")

    def pose_callback(self, msg):
        """Update turtle pose"""
        self.turtle_pose = np.array([msg.x, msg.y, msg.theta])

    def target_callback(self, msg):
        """Set movement target"""
        self.current_target = {
            'x': msg.x,
            'y': msg.y,
            'active': True
        }
        # Switch to auto mode when target is set
        self.control_mode = 'auto'
        self.get_logger().info(f"New target set: ({msg.x:.2f}, {msg.y:.2f})")

    def manual_cmd_callback(self, msg):
        """Handle manual control commands"""
        self.manual_command = {
            'linear': msg.linear.x,
            'angular': msg.angular.z
        }
        # Switch to manual mode when manual command received
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.control_mode = 'manual'
        else:
            # Stop command - could switch back to auto if there's an active target
            if not self.current_target['active']:
                self.control_mode = 'auto'

    def mode_callback(self, msg):
        """Set control mode"""
        if msg.data in ['auto', 'manual', 'stop']:
            if msg.data == 'stop':
                self.control_mode = 'auto'
                self.current_target['active'] = False
                self.manual_command = {'linear': 0.0, 'angular': 0.0}
                self.get_logger().info("Controller stopped")
            else:
                self.control_mode = msg.data
                self.get_logger().info(f"Control mode set to: {msg.data}")

    def publish_cmd_vel(self, linear_vel, angular_vel):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(msg)

    def publish_status(self, status):
        """Publish controller status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def control_loop(self):
        """Main control loop"""
        
        if self.control_mode == 'manual':
            # Manual control mode
            cmd = self.manual_command
            self.publish_cmd_vel(cmd['linear'], cmd['angular'])
            self.publish_status('manual_control')
            
        elif self.control_mode == 'auto':
            # Automatic control mode
            if self.current_target['active']:
                success = self.move_to_target()
                if success:
                    self.current_target['active'] = False
                    self.publish_status('target_reached')
                    self.get_logger().info(f"Target reached!")
                else:
                    self.publish_status('moving_to_target')
            else:
                # No active target, stop
                self.publish_cmd_vel(0.0, 0.0)
                self.publish_status('idle')

    def move_to_target(self):
        """Move turtle to target using proportional control"""
        target = self.current_target
        pose = self.turtle_pose
        
        # Calculate position error
        diff_x = target['x'] - pose[0] 
        diff_y = target['y'] - pose[1]
        distance = np.sqrt(diff_x**2 + diff_y**2)
        
        # Calculate desired heading
        theta_desired = np.arctan2(diff_y, diff_x)
        heading_error = theta_desired - pose[2]
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        # Proportional control
        linear_vel = self.kp_linear * distance
        angular_vel = self.kp_angular * heading_error
        
        # Publish velocities
        self.publish_cmd_vel(linear_vel, angular_vel)
        
        # Check if target reached
        if distance < self.tolerance and abs(heading_error) < self.tolerance:
            self.publish_cmd_vel(0.0, 0.0)
            return True
            
        return False

    def parameter_callback(self, params):
        """Handle dynamic parameter changes"""
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
            elif param.name == 'tolerance':
                if param.type_ == Parameter.Type.DOUBLE:
                    old_value = self.tolerance
                    self.tolerance = param.value
                    self.get_logger().info(f'tolerance changed from {old_value} to {param.value}')
                else:
                    successful = False

        return SetParametersResult(successful=successful)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()