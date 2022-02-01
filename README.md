# Home Service Robot

Udacity's Robotics Software Engineer Nanodegree Program

<p align="center"><img src="./images/vid_homeservice_rviz_window.gif"></p>

### Directory Tree and contents

```
.
├── README.md
├── images
│   ├── ... ...
├── CMakeLists.txt
├── add_markers
│   └── src
│       ├── add_markers.cpp
│       └── add_markers_test.cpp
├── map
│   ├── Maze_World.world
│   ├── maze_world.pgm
│   ├── map.yaml
├── pick_objects
│   └── src
│       ├── pick_objects.cpp
│       └── pick_objects_test.cpp
├── Rviz
│   └── default.rviz
├── scripts
│   ├── add_marker.sh
│   ├── home_service.sh
│   ├── pick_objects.sh
│   ├── test_navigation.sh
│   └── test_slam.sh
├── slam_gmapping
│   ├── gmapping
│   |── ... ...
├── turtlebot
│   |── turtlebot_teleop
│   |── ... ...
├── turtlebot_interactions
│   |── turtlebot_rviz_launchers
│   |── ... ...
|── turtlebot_simulator
│   ├── turtlebot_gazebo
│   |── ... ...

```

This directory represents the main project's `src` folder structure with following contents

* README.md: this file.
* **images** - folder with images and videos for this report
* **add_markers** - add marker C++ node
* **map** - map and gazebo world files
* **pick_objects** - pick-objects C++ node
* **Rviz** - folder with rViz configurations used with some launch scripts
* **scripts** - shell scripts
	* `add_marker.sh` - script for testing add_marker concept with `add_markers_test.cpp`
	* `home_service.sh` - main script for the home-service-robot
	* `pick_objects.sh` - script for testing pick_objects concept with `pick_objects_test`
	* `test_navigation.sh` - script for testing navigation
	* `test_slam.sh` - script for performing SLAM and preparing map
* **slam_gmapping** - official ROS package with `gmapping_demo.launch` file
* **turtlebot** - official ROS package with `keyboard_teleop.launch` file
* **turtlebot_interactions** - official ROS package with `view_navigation.launch` file
* **turtlebot_simulator** - official ROS package with `turtlebot_world.launch` file


### Clone and Build

Create Catkin workspace

```
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```

clone repository:

```
cd ~/catkin_ws/src/
git clone https://github.com/koushik-elite/Home-Service-Robot.git
```

Install dependencies:

```
rosdep -i install gmapping -y
rosdep -i install turtlebot_teleop -y
rosdep -i install turtlebot_rviz_launchers -y
rosdep -i install turtlebot_gazebo -y
```

`NOTE`: If any of the official packages give error,  delete associated folder and clone with src folder using appropriate line from here:

```
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git
```

build workspace

```
cd ~/catkin_ws/
catkin_make
```
### Launch Home service bot

```
cd ~/catkin_ws/src/scripts/
chmod +x home_service.sh
./home_service.sh
```

###Note: 
i edited the map which is created using slam_gmapping using Gimp/Image editor

