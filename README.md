# home-service-robot
Udacity Robotics Nanodegree Final Project

# Setup Instructions
## Requirements
- Gazebo 7.x
- ROS Kinetic
- Ubuntu 16.4

## ROS Packages
- `$ sudo apt-get install ros-kinetic-navigation`
- `$ sudo apt-get install ros-kinetic-map-server`
- `$ sudo apt-get install ros-kinetic-move-base`
- `$ sudo apt-get install ros-kinetic-amcl`

## Other
- `sudo apt-get install xterm`

## Local Build & Test Scripts
Create a catkin workspace
- `$ mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src`
- `$ catkin_init_workspace`

Clone and build this repo into the source folder
- `$ git clone https://github.com/davidsmithxc/home-service-robot ~/catkin_ws/src`
- `$ cd ~/catkin_ws && catkin_make`

There are several test scripts in the `/scripts` folder
- **NOTE 1** All scripts must be runt from the `~/catkin_ws` folder!
- **Note 2** Be sure to run `source devel/setup.sh` first.
- `test_slam.sh` can be used to manually test SLAM.
- `test_navigation.sh` can be used to check the navigation stack using the 2D Nav Goal commands, e.g. in Rviz.
- `add_markers.sh` can be used to check the functionality of publishing a marker in Rviz.

# Home service robot "simulation"
This section describes the final portion of the assignment - to simulate a home service robot pick up and drop off.

## Nodes
### add_markers
- This node functions to add markers using the /visualization_marker topic.
- These markers visually represent the object to be picked up as part of the simulation.
- The state machine which controls the flow of the simulation is also embedded in this node.
- This node subscribes to `/move_base/status` and uses the status message to determine when the robot has reached the current goal.

#### simulation state machine
The simulation state machine consists of a simple list of actions and the conditions to transition from one state to the next. With each transition, certain side actions occur.

- Init: Show marker A at pick up zone; set as first goal for navigation.
- Pickup: Hide marker A at pick up zone; wait 5 seconds for "picking up" time.
- Transport: Hide marker B at drop off zone, triggering new goal with transparent object
- Dropoff: Hide marker B at drop off zone; wait 5 seconds for "setting down time."
- Done: Show marker B at drop off zone; idle.

### pick_objects
- This node is relatively simple and waits for a `/visualiation_marker`
- The new message in `/visualization_marker` is used to create a new 2D Nav Goal

## Home Service Robot "Simulation"
The final script will launch the described nodes and play the home service robot simulation.
- `/scripts/home_service.sh`
