import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    # Namespace configuration
    turtlesim_plus1_namespace = 'teleop_window'
    turtlesim_plus2_namespace = 'copy_window'
    turtle1_namespace = 'Teleop'
    sampling_frequency = 100.0
    
    # Shared parameter sets
    standard_controller_params = {
        'kp_linear': 4.0,
        'kp_angular': 8.0,
        'tolerance': 0.1,
        'sampling_frequency': sampling_frequency,
    }
    
    eraser_controller_params = {
        'kp_linear': 3.0,
        'kp_angular': 15.0,
        'tolerance': 0.1,
        'sampling_frequency': sampling_frequency
    }
    
    
    teleop_scheduler_params = {
        'max_pizzas': 10,
        'sampling_frequency': sampling_frequency
    }
    copy_scheduler_params = {
        'sampling_frequency': sampling_frequency
        }
    
    
    # Copy turtle names and spawn configuration
    copy_turtle_names = ['Foxy', 'Noetic', 'Humble', 'Iron']
    spawn_position = {'x': -5.44, 'y': -5.44, 'theta': 0.0}

    def create_controller_node(namespace, params):
        """Helper function to create controller nodes with consistent configuration"""
        return Node(
            package='exam1',
            executable='turtle_controller.py',
            name='turtle_controller',
            namespace=namespace,
            output='screen',
            parameters=[params],
        )

    def create_scheduler_node(executable, namespace, params):
        """Helper function to create scheduler nodes with consistent configuration"""
        return Node(
            package='exam1',
            executable=executable,
            name=executable.replace('.py', ''),
            namespace=namespace,
            output='screen',
            parameters = [params]
        )

    def create_spawn_command(sim_namespace, turtle_name, position):
        """Helper function to create spawn service calls"""
        return ExecuteProcess(
            cmd=['ros2', 'service', 'call', 
                f'/{sim_namespace}/spawn_turtle', 
                'turtlesim/srv/Spawn', 
                f'{{"x": {position["x"]}, "y": {position["y"]}, "theta": {position["theta"]}, "name": "{turtle_name}"}}'], 
            output='screen',
        )

    def create_kill_command(sim_namespace, turtle_name):
        """Helper function to create kill service calls"""
        return ExecuteProcess(
            cmd=['ros2', 'service', 'call', f'/{sim_namespace}/remove_turtle', 
                'turtlesim/srv/Kill', f'{{name: \'{turtle_name}\'}}'],
            output='screen'
        )

    # Initialize launch description
    launch_description = LaunchDescription()
    
    # TurtleSim Plus nodes
    turtlesim_plus1 = Node(
        package='turtlesim_plus',
        executable='turtlesim_plus_node.py',
        name='turtlesim',
        namespace=turtlesim_plus1_namespace,
        output='screen',
    )

    turtlesim_plus2 = Node(
        package='turtlesim_plus',
        executable='turtlesim_plus_node.py',
        name='turtlesim',
        namespace=turtlesim_plus2_namespace,
        output='screen',
    )
    
    # Add turtlesim plus windows
    launch_description.add_action(turtlesim_plus1)
    launch_description.add_action(turtlesim_plus2)

    # Controller Nodes
    
    # Teleop turtle controller
    launch_description.add_action(create_controller_node(f'{turtlesim_plus1_namespace}/{turtle1_namespace}', standard_controller_params))

    # Copy turtle controllers
    for turtle_name in copy_turtle_names:
        launch_description.add_action(create_controller_node(f'{turtlesim_plus2_namespace}/{turtle_name}', standard_controller_params))

    # Eraser controllers
    eraser_namespaces = [
        f'{turtlesim_plus1_namespace}/Eraser',
        f'{turtlesim_plus2_namespace}/Eraser'
    ]
    
    for namespace in eraser_namespaces:
        launch_description.add_action(create_controller_node(namespace, eraser_controller_params))

    # Scheduler Nodes
    
    # Teleop scheduler
    launch_description.add_action(create_scheduler_node('teleop_scheduler.py',f'{turtlesim_plus1_namespace}/{turtle1_namespace}', teleop_scheduler_params))
    # Copy schedulers
    for turtle_name in copy_turtle_names:
        launch_description.add_action(create_scheduler_node('copy_scheduler.py',f'{turtlesim_plus2_namespace}/{turtle_name}', copy_scheduler_params))
    # Eraser scheduler
    launch_description.add_action(create_scheduler_node('eraser_scheduler.py', '/', copy_scheduler_params))

    # Service calls to set up turtles
    
    # Kill and spawn teleop turtle
    launch_description.add_action(create_kill_command(turtlesim_plus1_namespace, 'turtle1'))
    launch_description.add_action(create_spawn_command(turtlesim_plus1_namespace, turtle1_namespace, {'x': 7.0, 'y': 7.0, 'theta': 0.0}))
    # Kill turtle1 in copy window
    launch_description.add_action(create_kill_command(turtlesim_plus2_namespace, 'turtle1'))
    # Spawn copy turtles
    for turtle_name in copy_turtle_names:
        launch_description.add_action(
            create_spawn_command(turtlesim_plus2_namespace, turtle_name, spawn_position)
        )

    return launch_description


def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        sys.exit()


if __name__ == "__main__":
    main()