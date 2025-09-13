# The steps for testing

1. **Clone the repository**:
   ```bash
   git clone https://github.com/fibo-github-classroom/multiverse-mission-ioon-beam.git
    ```

2. **Build and source the workspace**:
   ```bash
   cd multiverse-mission-ioon-beam
   colcon build
   source install/setup.bash
    ```

3. **Run launch file**:
    There is turtlesim_plus sceen 1 and 2, turtle_teleop, turtle_copy and turtle_eraser
   ```bash
   ros2 launch exam1 turtle_bringup.launch.py
   ```

4. **Run teleop_key node**:
   ```bash
   ros2 run exam1 teleop_key.py
   ```

