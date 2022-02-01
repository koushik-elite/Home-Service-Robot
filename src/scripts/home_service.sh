
#!/bin/sh

# launch turtlebot_world.launch to deploy turtlebot environment
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0';
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../../src/map/Maze_World.world" & 

sleep 10

# launch amcl_demo.launch for localization
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../../src/map/map.yaml" &

sleep 10

# launch view_navigation for rviz
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosrun rviz rviz -d $(pwd)/../../src/Rviz/default.rviz" &

sleep 10
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosrun add_markers add_markers;bash" &

sleep 10
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosrun pick_objects pick_objects;bash"