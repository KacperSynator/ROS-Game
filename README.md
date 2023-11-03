# ROS-Game
A simple app using three ROS nodes named:
- `keyboard_reader` reads keyboard input from terminal
- `game_display` shows the game in a window
- `game_controller` handles game logic

Currently there is no purpose in game. Player can just move around the map.  
To move the player use "awsd" in terminal running `keyboard_reader` node.

## Getting started
### Dependancies
```bash
sudo apt install cmake libsdl2-dev g++ libncurses5-dev libncursesw5-dev
```

### Run
Build the app first
```bash
# from ros2 workspace
. install/local_setup.bash 
colcon build --packages-select ros_game
```
Then run the app in three separate terminals, remember to source the environment
```bash
# in first terminal
. install/local_setup.bash 
ros2 run ros_game keyboard_reader
# to exit this node press 'q'
```

```bash
# in second terminal
. install/local_setup.bash 
ros2 run ros_game game_display
# to exit use ctrl + c
```

```bash
# in third terminal (run always after game_display)
. install/local_setup.bash 
ros2 run ros_game game_controller
# to exit use ctrl + c
```
