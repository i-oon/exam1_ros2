#!/usr/bin/python3
import rclpy
import os
import numpy as np
import yaml
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from std_msgs.msg import Bool
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point
from std_srvs.srv import Empty
from controller_interfaces.srv import SetParam


class TurtleEraser(Node):
    def __init__(self):
        super().__init__('turtle_eraser')

        self.turtle_queue = []
        self.eraser_pose = None
        self.eraser_copy_pose = None
        self.teleop_pose = None
        self.copy_target_pose = None
        
        self.kp_linear = 3.0
        self.kp_angular = 15.0
        self.tolerance = 0.1
        
        # Simple state flags - following TurtleCopy pattern  
        self.is_turtle_erase_spawned = False
        self.teleop_tasks_completed = False
        self.copy_mode = False
        self.copy_turtles_eaten = False
        self.mission_completed = False
        
        # Simple publishers and subscribers - same pattern as TurtleCopy
        self.cmd_vel_pub = self.create_publisher(Twist, '/teleop_window/Eraser/cmd_vel', 10)
        self.cmd_vel_copy_pub = self.create_publisher(Twist, '/copy_window/Eraser/cmd_vel', 10)
        self.create_subscription(Pose, '/teleop_window/Eraser/pose', self.pose_callback, 10)
        self.create_subscription(Pose, '/copy_window/Eraser/pose', self.pose_copy_callback, 10)
        self.create_subscription(Pose, '/teleop_window/Teleop/pose', self.teleop_pose_callback, 10)
        self.create_subscription(Bool, '/release_eraser_turtle', self.release_erase_turtle_callback, 10)
        self.create_subscription(Point, '/transition_pose', self.get_transition_pose, 10)
        
        # Service clients - only difference: add copy window eat service
        self.eat_pizza_client = self.create_client(Empty, '/teleop_window/Eraser/eat')
        self.eat_pizza_copy_client = self.create_client(Empty, '/copy_window/Eraser/eat')
        self.spawn_turtle_in_teleop_client = self.create_client(Spawn, '/teleop_window/spawn_turtle')
        self.spawn_turtle_in_copy_client = self.create_client(Spawn, '/copy_window/spawn_turtle')
        self.kill_client = self.create_client(Kill, '/teleop_window/remove_turtle')
        self.kill_copy_client = self.create_client(Kill, '/copy_window/remove_turtle')
        
        self.create_service(SetParam, 'set_param', self.set_param_callback)
        
        # Timer - same frequency pattern as TurtleCopy
        self.create_timer(0.01, self.timer_callback)  # 100Hz like TurtleCopy
        
    def get_transition_pose(self, msg):
        self.copy_target_pose = np.array([msg.x, msg.y])
        
    def teleop_pose_callback(self, msg):
        self.teleop_pose = np.array([msg.x, msg.y, msg.theta])
     
    def pose_callback(self, msg):
        self.eraser_pose = np.array([msg.x, msg.y, msg.theta])

    def pose_copy_callback(self, msg):
        self.eraser_copy_pose = np.array([msg.x, msg.y, msg.theta])
        
    def publish_cmd_vel(self, v, w, use_copy_window=False):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        
        if use_copy_window:
            self.cmd_vel_copy_pub.publish(msg)
        else:
            self.cmd_vel_pub.publish(msg)

    
    def release_erase_turtle_callback(self, msg):
        if msg.data and not self.is_turtle_erase_spawned:
            self.is_turtle_erase_spawned = True
            self.load_positions()
            self.spawn_turtle_in_teleop()
    
    def load_positions(self):
        save_dir = os.path.expanduser('~/multiverse-mission-ioon-beam/src/exam1')
        for i in range(1, 5):
            file_path = os.path.join(save_dir, f'saved_pizzas_{i}.yaml')
            try:
                with open(file_path, 'r') as f:
                    data = yaml.safe_load(f)
                    self.turtle_queue += data['pizzas'].copy()
            except Exception as e:
                self.get_logger().warn(f"Could not load saved_pizzas_{i}.yaml: {str(e)}")
        
        self.get_logger().info(f"Total pizzas to eat: {len(self.turtle_queue)}")
    
    def spawn_turtle_in_teleop(self):
        while not self.spawn_turtle_in_teleop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        req = Spawn.Request()
        req.x = 1.0
        req.y = 1.0
        req.theta = 0.0
        req.name = 'Eraser'
        
        self.spawn_turtle_in_teleop_client.call_async(req)
        self.get_logger().info('Eraser spawned in teleop window')

    def move_to_target(self, target, use_copy_window=False):
        # Select appropriate pose based on window
        if use_copy_window:
            pose = self.eraser_copy_pose
        else:
            pose = self.eraser_pose
        
        # Check if pose is available
        if pose is None:
            return False
        
        # Calculate position and orientation errors
        diff_x = target['x'] - pose[0]
        diff_y = target['y'] - pose[1]
        d = np.sqrt(diff_x**2 + diff_y**2)
        theta_d = np.arctan2(diff_y, diff_x)
        e_theta = theta_d - pose[2]
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        # Apply control law
        vx = self.kp_linear * d
        w = self.kp_angular * e_theta
        self.publish_cmd_vel(vx, w, use_copy_window)

        # Check if target is reached
        return d < self.tolerance and abs(e_theta) < self.tolerance

    def eat_pizza(self, use_copy_window=False):
        req = Empty.Request()
        if use_copy_window:
            self.eat_pizza_copy_client.call_async(req)
            self.get_logger().info("Eating with copy service")
        else:
            self.eat_pizza_client.call_async(req)
            self.get_logger().info("Eating with regular service")


    def kill_turtle(self, name, use_copy_window=False):
        req = Kill.Request()
        req.name = name
        
        if use_copy_window:
            self.kill_copy_client.call_async(req)
        else:
            self.kill_client.call_async(req)

    def timer_callback(self):
        if self.mission_completed:
            self.publish_cmd_vel(0.0, 0.0, use_copy_window=False)
            return
            
        if not self.is_turtle_erase_spawned:
            self.publish_cmd_vel(0.0, 0.0, use_copy_window=False)
            return

        # Phase 1: Teleop mode - eat pizzas then hunt teleop turtle
        if not self.teleop_tasks_completed:
            if self.eraser_pose is None:
                return
                
            # Sub-phase 1a: Eat all pizzas
            if self.turtle_queue:
                target = self.turtle_queue[0]
                if self.move_to_target(target, use_copy_window=False):
                    self.eat_pizza(use_copy_window=False)
                    self.turtle_queue.pop(0)
                    self.get_logger().info(f"Pizza eaten! {len(self.turtle_queue)} remaining")
                return
            
            # Sub-phase 1b: Hunt teleop turtle
            if self.teleop_pose is not None:
                teleop_target = {'x': self.teleop_pose[0], 'y': self.teleop_pose[1]}
                if self.move_to_target(teleop_target, use_copy_window=False):
                    self.kill_turtle('Teleop', use_copy_window=False)
                    self.kill_turtle('Eraser', use_copy_window=False)
                    self.teleop_tasks_completed = True
                    self.get_logger().info("Teleop tasks completed!")
                    # Wait a bit then spawn in copy
                    self.create_timer(0.5, self.spawn_in_copy_delayed)
                return

        # Phase 2: Copy mode 
        if self.copy_mode:
            if self.eraser_pose is None or self.copy_target_pose is None:
                return
                
            # Sub-phase 2a: Eat copy turtles at spawn position
            if not self.copy_turtles_eaten:                    
                # Eat 4 copy turtles
                self.kill_turtle('Foxy', use_copy_window=True)
                self.kill_turtle('Noetic', use_copy_window=True)
                self.kill_turtle('Humble', use_copy_window=True)
                self.kill_turtle('Iron', use_copy_window=True)
                self.copy_turtles_eaten = True
                self.get_logger().info("Copy turtles eaten! Now eating copy pizzas...")
                # Reload positions for copy pizzas
                self.turtle_queue = []
                self.load_positions()
            
            # Sub-phase 2b: Eat all copy pizzas
            if self.turtle_queue:
                target = self.turtle_queue[0]
                if self.move_to_target(target, use_copy_window=True):
                    self.eat_pizza(use_copy_window=True)
                    self.turtle_queue.pop(0)
                    self.get_logger().info(f"Copy pizza eaten! {len(self.turtle_queue)} remaining")
                return
            else:
                self.mission_completed = True
                self.kill_turtle('Eraser', use_copy_window=True)
                self.get_logger().info("Mission completed!")
                return

    def spawn_in_copy_delayed(self):
        if not self.copy_mode and self.copy_target_pose is not None:
            while not self.spawn_turtle_in_copy_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for copy spawn service...')
            
            req = Spawn.Request()
            req.x = self.copy_target_pose[0]
            req.y = self.copy_target_pose[1]
            req.theta = 0.0
            req.name = 'Eraser'
            
            self.spawn_turtle_in_copy_client.call_async(req)
            self.copy_mode = True
            self.get_logger().info('Eraser spawned in copy window!')

    def set_param_callback(self, request, response):
        if request.name == 'kp_linear':
            self.kp_linear = request.value
            response.success = True
        elif request.name == 'kp_angular':
            self.kp_angular = request.value
            response.success = True
        else:
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TurtleEraser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()