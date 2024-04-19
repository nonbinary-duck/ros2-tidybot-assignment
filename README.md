# Villanelle O'Reilly Autonomous Mobile Robots Tidybot assignment solution

## Dependencies 

Please install the nav2 stack and slam-toolbox:
```bash
sudo apt install ros-humble-tf2-tools ros-humble-tf2-py ros-humble-tf-transformations ros-humble-tf2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-moveit ros-humble-nav2-simple-commander
```

Also install python packages:
```bash
pip3 install opencv-python
sudo pip3 install transforms3d
```

## Usage

Then build with colcon:
```bash
colcon build --symlink-install
source install/setup.bash
```

To run the simulator please use:
```bash
# The basic world (with green cubes)
ros2 launch tidybot_solution sim_and_stack.launch.py
# One of the level 2 worlds
ros2 launch tidybot_solution sim_and_stack.launch.py world:="level_2_1.world"
# Add red cubes too
ros2 launch tidybot_solution sim_and_stack.launch.py world:="level_2_1.world" add_red_cubes:="True"
# List params
ros2 launch tidybot_solution sim_and_stack.launch.py --show-args
```

And to run the solution please use:
```bash
ros2 launch tidybot_solution solution.launch.py
```
