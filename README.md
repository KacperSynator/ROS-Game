# ROS-Game

## Dependancies
```bash
sudo apt install cmake libsdl2-dev g++ libncurses5-dev libncursesw5-dev
```

## Run
```bash
# from ros2 workspace
. install/local_setup.bash 
colcon build

# in first terminal
ros2 run ros_game keyboard_reader

# in second terminal
ros2 run ros_game game_display
```
