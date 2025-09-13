import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    turtlesim_plus1_namespace = 'teleop_window'
    turtlesim_plus2_namespace = 'copy_window'
    turtle1_namespace = 'Teleop'
        
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

    turtle_teleop = Node(
        package='exam1',
        executable='turtle_teleop.py',
        name='teleop_node',
        namespace=f'{turtlesim_plus1_namespace}/{turtle1_namespace}',
        output='screen',
        parameters=[{
            'sampling_frequency': 50.0,
            'tolerance': 0.1
            }],
    )

    turtle_copy_Foxy = Node(
        package='exam1',
        executable='turtle_copy.py',
        name='copy_node',
        namespace=f'{turtlesim_plus2_namespace}/Foxy',
        output='screen',
        parameters=[{
            'sampling_frequency': 50.0,
            'tolerance': 0.1
            }],
    )

    turtle_copy_Noetic = Node(
        package='exam1',
        executable='turtle_copy.py',
        name='copy_node',
        namespace=f'{turtlesim_plus2_namespace}/Noetic',
        output='screen',
        parameters=[{
            'sampling_frequency': 50.0,
            'tolerance': 0.1
            }],
    )

    turtle_copy_Humble = Node(
        package='exam1',
        executable='turtle_copy.py',
        name='copy_node',
        namespace=f'{turtlesim_plus2_namespace}/Humble',
        output='screen',
        parameters=[{
            'sampling_frequency': 50.0,
            'tolerance': 0.1
            }],
    )

    turtle_copy_Iron = Node(
        package='exam1',
        executable='turtle_copy.py',
        name='copy_node',
        namespace=f'{turtlesim_plus2_namespace}/Iron',
        output='screen',
        parameters=[{
            'sampling_frequency': 50.0,
            'tolerance': 0.1
            }],
    )

    turtle_eraser = Node(
        package='exam1',
        executable='turtle_eraser.py',
        name='eraser_node',
        namespace=f'{turtlesim_plus1_namespace}/Eraser',
        output='screen'
    )

    kill_turtle1 = ExecuteProcess(
        cmd = ['ros2', 'service', 'call', f'/{turtlesim_plus1_namespace}/remove_turtle', 'turtlesim/srv/Kill', '{name: \'turtle1\'}'],
        output='screen'
    )

    spawn_turtle1 = ExecuteProcess(
        cmd = ['ros2', 'service', 'call', f'/{turtlesim_plus1_namespace}/spawn_turtle', 'turtlesim/srv/Spawn','{x: 7.0, y: 7.0, theta: 0.0, name: ' + turtle1_namespace + '}'],
        output='screen'
    )

    kill_turtle2 = ExecuteProcess(
        cmd = ['ros2', 'service', 'call', f'/{turtlesim_plus2_namespace}/remove_turtle', 'turtlesim/srv/Kill', '{name: \'turtle1\'}'],
        output='screen'
    )

    spawn_Foxy = ExecuteProcess(
        cmd=['ros2', 'service', 'call', 
            f'/{turtlesim_plus2_namespace}/spawn_turtle', 
            'turtlesim/srv/Spawn', 
            '{"x": -5.44, "y": -5.44, "theta": 0.0, "name": "Foxy"}'], 
        output='screen',
    )

    spawn_Noetic = ExecuteProcess(
        cmd=['ros2', 'service', 'call', 
            f'/{turtlesim_plus2_namespace}/spawn_turtle', 
            'turtlesim/srv/Spawn', 
            '{"x": -5.44, "y": -5.44, "theta": 0.0, "name": "Noetic"}'], 
        output='screen',
    )

    spawn_Humble = ExecuteProcess(
        cmd=['ros2', 'service', 'call', 
            f'/{turtlesim_plus2_namespace}/spawn_turtle', 
            'turtlesim/srv/Spawn', 
            '{"x": -5.44, "y": -5.44, "theta": 0.0, "name": "Humble"}'], 
        output='screen',
    )

    spawn_Iron = ExecuteProcess(
        cmd=['ros2', 'service', 'call', 
            f'/{turtlesim_plus2_namespace}/spawn_turtle', 
            'turtlesim/srv/Spawn', 
            '{"x": -5.44, "y": -5.44, "theta": 0.0, "name": "Iron"}'], 
        output='screen',
    )

    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(turtlesim_plus1)
    launch_description.add_action(turtlesim_plus2)
    launch_description.add_action(turtle_teleop)
    launch_description.add_action(turtle_copy_Foxy)
    launch_description.add_action(turtle_copy_Noetic)
    launch_description.add_action(turtle_copy_Humble)
    launch_description.add_action(turtle_copy_Iron)
    launch_description.add_action(turtle_eraser) 
    launch_description.add_action(kill_turtle1)
    launch_description.add_action(spawn_turtle1)
    launch_description.add_action(kill_turtle2)
    launch_description.add_action(spawn_Foxy)
    launch_description.add_action(spawn_Noetic)
    launch_description.add_action(spawn_Humble)
    launch_description.add_action(spawn_Iron)

    return launch_description


def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()


if __name__ == "__main__":
    main()



