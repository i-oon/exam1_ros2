#!/usr/bin/python3

import os
import rclpy
import numpy as np
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
# from controller_interfaces.srv import SetParam, SetMaxPizzas

class TurtleTeleop(Node):
    def __init__(self):
        super().__init__('turtle_teleop')
        
        full_ns = self.get_namespace().strip("/") 
        parts = full_ns.split("/")
        turtlesim_plus1_namespace = parts[0] if parts else ""

        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.melodic_pose = np.array([0.0, 0.0, 0.0])

        self.kp_linear = 4.0
        self.kp_angular = 8.0

        self.current_key = None
        self.pizza_queue = []
        self.static_queue = []
        self.max_pizzas = 10
        self.saved_pizzas = set()
        self.clearing_active = False
        self.target = None

        self.save_count = 0
        self.max_saves = 4
        self.save_locked = False

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
        
        self.cmd_vel_pub = self.create_publisher(Twist, f'cmd_vel', 10)
        self.create_subscription(Pose, f'pose', self.pose_callback, 10)
        self.create_subscription(String, '/keyboard', self.key_callback, 10)

        self.control_timer = self.create_timer(1/self.frequency, self.timer_callback)

        self.spawn_pizza_client = self.create_client(GivePosition, f'/{turtlesim_plus1_namespace}/spawn_pizza')
        self.eat_pizza_client = self.create_client(Empty, 'eat')
        # self.cli_gain_service = self.create_service(SetParam, 'set_gain', self.cli_gain_callback)
        # self.cli_max_pizzas_service = self.create_service(SetMaxPizzas, 'set_max_pizzas', self.cli_max_pizzas_callback)
        self.spawn_turtle_client = self.create_client(Spawn, f'/{turtlesim_plus1_namespace}/spawn_turtle')
        self.kill_turtle_client = self.create_client(Kill, f'/{turtlesim_plus1_namespace}/remove_turtle')

        self.kp_linear = 3.0
        self.kp_angular = 7.0

        self.move_dirs = {
            'w': [3.0, 0.0],
            's': [-3.0, 0.0],
            'a': [0.0, 3.0],
            'd': [0.0, -3.0],
        }

        self.lin_step = 1.0
        self.ang_step = 1.0

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def timer_callback(self):
        if self.current_key == "None":
            self.cmdvel(0.0, 0.0)

        if self.clearing_active:
            self.clear_pizzas()

        if self.current_key in self.move_dirs:
            v, w = self.move_dirs[self.current_key]
            self.cmdvel(v, w)
        elif self.current_key == 'p':
            self.spawn_pizza(self.robot_pose[0], self.robot_pose[1])
            self.current_key = "None" 
        elif self.current_key == 'o':
            self.save_pizzas()
            self.current_key = "None" 
        elif self.current_key == 'c':
            self.current_key = "None" 
            if not self.clearing_active:
                self.start_clearing()
        elif self.current_key == 'f':
            self.move_dirs['w'][0] += self.lin_step
            self.move_dirs['s'][0] -= self.lin_step
            self.current_key = "None"
        elif self.current_key == 'v':
            self.move_dirs['w'][0] -= self.lin_step
            self.move_dirs['s'][0] += self.lin_step
            self.current_key = "None"
        elif self.current_key == 'g':
            self.move_dirs['a'][1] -= self.ang_step
            self.move_dirs['d'][1] += self.ang_step
            self.current_key = "None"
        elif self.current_key == 'b':
            self.move_dirs['a'][1] += self.ang_step
            self.move_dirs['d'][1] -= self.ang_step
            self.current_key = "None"

    def key_callback(self, msg):
        self.current_key = msg.data
        print(self.current_key)

    def pose_callback(self, msg):
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.robot_pose[2] = msg.theta

    def spawn_pizza(self, x, y):
        if len(self.static_queue) >= self.max_pizzas:
            self.get_logger().warn(f"Cannot spawn more than {self.max_pizzas} pizzas")
            return

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
            self.pizza_queue.append((x, y))
            self.static_queue.append((x, y))
            self.get_logger().info(f"Current number of pizza: {len(self.static_queue)} / {self.max_pizzas}")
        except Exception as e:
            self.get_logger().error(f'Failed to spawn pizza: {str(e)}')

    def save_pizzas(self):
        if self.save_locked:
            self.get_logger().warn("Save locked! Reset to save again (press 'r')")
            return

        if not self.pizza_queue:
            self.get_logger().info("No pizzas to save")
            return
        
        self.save_directory = os.path.expanduser('~/multiverse-mission-ioon-beam/src/exam1')
        os.makedirs(self.save_directory, exist_ok=True)
        
        data = {
            'pizzas': [
                {'x': float(x), 'y': float(y)}
                for x, y in self.pizza_queue
            ]
        }
        
        filename = f"saved_pizzas_{self.save_count + 1}.yaml"
        file_path = os.path.join(self.save_directory, filename)
        
        try:
            with open(file_path, 'w') as f:
                yaml.dump(data, f)

            self.saved_pizzas = {(p[0], p[1]) for p in self.pizza_queue}  
            self.save_count += 1
            self.get_logger().info(f"Saved {len(self.pizza_queue)} pizzas to {file_path}")
            self.pizza_queue.clear()

            if self.save_count >= self.max_saves:
                self.save_locked = True
                self.get_logger().warn("MAX SAVES REACHED! Press 'r' to reset")
        
        except Exception as e:
            self.get_logger().error(f"Failed to save pizzas: {str(e)}")


    def start_clearing(self):
        if not self.pizza_queue:
            self.get_logger().info("No pizzas to clear")
            return
        
        self.clearing_active = True
        self.target = self.pizza_queue[0]
        self.get_logger().info(f"Starting to clear pizza")

    def clear_pizzas(self):
        if not self.clearing_active:
            return 
        
        target = self.pizza_queue[0]
        diff_x = target[0] - self.robot_pose[0]
        diff_y = target[1] - self.robot_pose[1]
        d = np.sqrt(diff_x**2 + diff_y**2)
        theta_d = np.arctan2(diff_y, diff_x)
        e_theta = theta_d - self.robot_pose[2]
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        vx = self.kp_linear * d
        w = self.kp_angular * e_theta
        self.cmdvel(vx, w)

        if d < self.tolerance and abs(e_theta) < self.tolerance:
            self.eat_pizza()

    def eat_pizza(self):
        if not self.eat_pizza_client.service_is_ready():
            self.get_logger().warn("Eat service not available")
            self.clearing_active = False
            self.target = None
            return

        eat_request = Empty.Request()
        future = self.eat_pizza_client.call_async(eat_request)
        future.add_done_callback(self.eat_pizza_callback)

    def eat_pizza_callback(self, future):
        try:
            future.result()
            self.pizza_queue.pop(0)
            self.static_queue.pop(0)
            self.get_logger().info(f"Current number of pizza: {len(self.static_queue)} / {self.max_pizzas}")

            if len(self.pizza_queue) > 0:
                    self.clear_pizzas()
            else:
                self.clearing_active = False
                
        except Exception as e:
            self.get_logger().error(f'Failed to eat pizza: {str(e)}')
            self.clearing_active = False  

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

        return SetParametersResult(successful=successful)
    
    # def cli_gain_callback(self, request:SetParam.Request, response:SetParam.Response):
    #     self.kp_linear = request.kp_linear.data
    #     self.kp_angular = request.kp_angular.data
    #     self.get_logger().info(f'Gain set to: {request.kp_linear.data}, {request.kp_angular.data}')
    #     return response
    
    # def cli_max_pizzas_callback(self, request:SetMaxPizzas.Request, response:SetMaxPizzas.Response):
    #     self.new_max_pizzas = request.max_pizzas.data
    #     if (self.new_max_pizzas > self.max_pizzas):
    #         self.max_pizzas = self.new_max_pizzas
    #         response.log.data = f"{self.max_pizzas}, success"
    #         self.get_logger().info(f'Max pizza set to: {request.max_pizzas.data}')
    #     else:
    #         response.log.data = f"{self.max_pizzas}, failed"
    #     return response


def main(args=None):
    rclpy.init(args=args)
    node = TurtleTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
