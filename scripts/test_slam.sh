xterm -e "cd /home/robond/workspace/catkin_ws/;
source devel/setup.bash;
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/workspace/catkin_ws/src/map/apartment_ball.world" &
sleep 10

xterm -e "cd /home/robond/workspace/catkin_ws/;
source devel/setup.bash;
roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 10

xterm -e "cd /home/robond/workspace/catkin_ws/;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10

xterm -e "cd /home/robond/workspace/catkin_ws/;
source devel/setup.bash;
roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 10
