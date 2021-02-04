xterm -e "cd /home/robond/workspace/catkin_ws/;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x 4 -y -3 -z 0 -R 0 -P 0 -Y 0';
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/workspace/catkin_ws/src/map/apartment_ball.world" &
sleep 5

xterm -e "cd /home/robond/workspace/catkin_ws/;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/robond/workspace/catkin_ws/src/map/map.yaml" &
sleep 5

xterm -e "cd /home/robond/workspace/catkin_ws/;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10

xterm -e "cd /home/robond/workspace/catkin_ws/;
source devel/setup.bash;
rosrun add_markers add_markers" &

xterm -e "cd /home/robond/workspace/catkin_ws/;
source devel/setup.bash;
rosrun pick_objects pick_objects" &