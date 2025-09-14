#!/usr/bin/python3
import rclpy
import os
import numpy as np
import yaml
import sys
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from std_msgs.msg import Bool, String
from turtlesim.msg import Pose
from geometry_msgs.msg import Point
from std_srvs.srv import Empty


class TurtleEraser(Node):
    def __init__(self):
        super().__init__('turtle_eraser')

        self.turtle_queue = []
        self.eraser_pose = None
        self.eraser_copy_pose = None
        self.teleop_pose = None
        self.copy_target_pose = None
        
        # Task management flags
        self.is_turtle_erase_spawned = False
        self.teleop_tasks_completed = False
        self.copy_mode = False
        self.copy_turtles_eaten = False
        self.mission_completed = False
        
        # Target tracking for both controllers
        self.teleop_target_reached = False
        self.copy_target_reached = False
        
        # Publishers to different controller instances
        self.teleop_target_pub = self.create_publisher(Point, '/teleop_window/Eraser/controller/target', 10)
        self.teleop_mode_pub = self.create_publisher(String, '/teleop_window/Eraser/controller/mode', 10)
        self.copy_target_pub = self.create_publisher(Point, '/copy_window/Eraser/controller/target', 10)
        self.copy_mode_pub = self.create_publisher(String, '/copy_window/Eraser/controller/mode', 10)
        
        # Subscribers
        self.create_subscription(Pose, '/teleop_window/Eraser/pose', self.pose_callback, 10)
        self.create_subscription(Pose, '/copy_window/Eraser/pose', self.pose_copy_callback, 10)
        self.create_subscription(Pose, '/teleop_window/Teleop/pose', self.teleop_pose_callback, 10)
        self.create_subscription(Bool, '/release_eraser_turtle', self.release_erase_turtle_callback, 10)
        self.create_subscription(Point, '/transition_pose', self.get_transition_pose, 10)
        
        # Controller status subscribers
        self.create_subscription(String, '/teleop_window/Eraser/controller/status', self.teleop_status_callback, 10)
        self.create_subscription(String, '/copy_window/Eraser/controller/status', self.copy_status_callback, 10)
        
        # Service clients
        self.eat_pizza_client = self.create_client(Empty, '/teleop_window/Eraser/eat')
        self.eat_pizza_copy_client = self.create_client(Empty, '/copy_window/Eraser/eat')
        self.spawn_turtle_in_teleop_client = self.create_client(Spawn, '/teleop_window/spawn_turtle')
        self.spawn_turtle_in_copy_client = self.create_client(Spawn, '/copy_window/spawn_turtle')
        self.kill_client = self.create_client(Kill, '/teleop_window/remove_turtle')
        self.kill_copy_client = self.create_client(Kill, '/copy_window/remove_turtle')
        
        # Timer
        self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info("Eraser Scheduler initialized")
        
    def get_transition_pose(self, msg):
        self.copy_target_pose = np.array([msg.x, msg.y])
        
    def teleop_pose_callback(self, msg):
        self.teleop_pose = np.array([msg.x, msg.y, msg.theta])
     
    def pose_callback(self, msg):
        self.eraser_pose = np.array([msg.x, msg.y, msg.theta])

    def pose_copy_callback(self, msg):
        self.eraser_copy_pose = np.array([msg.x, msg.y, msg.theta])

    def teleop_status_callback(self, msg):
        """Handle status from teleop eraser controller"""
        if msg.data == 'target_reached':
            self.teleop_target_reached = True

    def copy_status_callback(self, msg):
        """Handle status from copy eraser controller"""
        if msg.data == 'target_reached':
            self.copy_target_reached = True

    def send_teleop_target(self, x, y):
        """Send target to teleop eraser controller"""
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        self.teleop_target_pub.publish(msg)

    def send_copy_target(self, x, y):
        """Send target to copy eraser controller"""
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        self.copy_target_pub.publish(msg)

    def send_teleop_mode(self, mode):
        """Send mode to teleop eraser controller"""
        msg = String()
        msg.data = mode
        self.teleop_mode_pub.publish(msg)

    def send_copy_mode(self, mode):
        """Send mode to copy eraser controller"""
        msg = String()
        msg.data = mode
        self.copy_mode_pub.publish(msg)

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
            self.send_teleop_mode('stop')
            self.send_copy_mode('stop')
            return
            
        if not self.is_turtle_erase_spawned:
            return

        # Phase 1: Teleop mode - eat pizzas then hunt teleop turtle
        if not self.teleop_tasks_completed:
            if self.eraser_pose is None:
                return
                
            # Sub-phase 1a: Eat all pizzas
            if self.turtle_queue:
                target = self.turtle_queue[0]
                if not self.teleop_target_reached:
                    self.send_teleop_target(target['x'], target['y'])
                else:
                    self.eat_pizza(use_copy_window=False)
                    self.turtle_queue.pop(0)
                    self.teleop_target_reached = False
                    self.get_logger().info(f"Pizza eaten! {len(self.turtle_queue)} remaining")
                return
            
            # Sub-phase 1b: Hunt teleop turtle
            if self.teleop_pose is not None:
                if not self.teleop_target_reached:
                    self.send_teleop_target(self.teleop_pose[0], self.teleop_pose[1])
                else:
                    self.kill_turtle('Teleop', use_copy_window=False)
                    self.kill_turtle('Eraser', use_copy_window=False)
                    self.teleop_tasks_completed = True
                    self.get_logger().info("Teleop tasks completed!")
                    # Wait a bit then spawn in copy
                    self.create_timer(0.5, self.spawn_in_copy_delayed)
                return

        # Phase 2: Copy mode 
        if self.copy_mode:
            if self.eraser_copy_pose is None or self.copy_target_pose is None:
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
                if not self.copy_target_reached:
                    self.send_copy_target(target['x'], target['y'])
                else:
                    self.eat_pizza(use_copy_window=True)
                    self.turtle_queue.pop(0)
                    self.copy_target_reached = False
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


def main(args=None):
    rclpy.init(args=args)
    node = TurtleEraser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()