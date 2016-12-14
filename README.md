# kamtoa-simulation
# SOM-O Phase

Kamtoa Simulation
> Current developing phase : SOM-O (ส้มโอ)


### Structure 
Core - structure is consisted of 

* som_o_driver - ROSCPP Polling Serial implementation of hiveground's Ganglia motor driver board
* som_o_base  -  Hardwares , Sensors utilities such as laser filter , safety controller 
* som_o_bringup - Launch files and entry point to run whole project 
* som_o_description - Description file of som_o Robot and gazebo settings
* som_o_gazebo - Simulation environment , launchfiles for spawning things in simulated world 
* som_o_navigation - Autonomous navigation stack ( ROS Navigation Stack ) 
* som_o_ros - Metapackage 
* som_o_teleop - Teleoperation by keyboard , joystick and Obstacle profile avoidance controller 
* kamtoa_deprecated - Old files from previous phase 
* kamtoa_map_manager - [Migrating] Map and POI manager
* kamtoa_smarthome_controller - Gazebo Environment Controller 


### Dependencies
SOM-O requires 
* [ROS - Kinetic kame - Full Desktop install ](http://wiki.ros.org/kinetic/Installation/Ubuntu) To run
* YOCS_VELOCITY_SMOOTHER 
* MOVE_BASE
* 

### Bringing - Up [Simulation]
1. Run the simulation 
```sh
roslaunch som_o_gazebo gazebo_som_o.launch
```
2. Run the Robot Perception and Autonomous Navigation Stack (move_base) [(optional)args map]
```sh
roslaunch som_o_navigation sim_nav.launch
```
3. Run Teleoperation Node 
```sh
roslaunch som_o_teleop teleop_joystick.launch 
```
4. Run the POI Manager Stack 
```sh
roslaunch kamtoa_map_manager map_manager.launch 
```
5. Press Reload at the Rviz plugin (POI Manager) to get the list of POI

### Bringing - Up [Real Robot]
1. Run the robot base 
WARNING : all of the sensors access must be granted first (sudo chmod 777 /dev/ttyUSBX) X=id number
RP-Lidar = ttyUSB1
Motor Driver = ttyUSB0
```sh
roslaunch som_o_bringup som_o_bringup.launch 
```
2. Run the Robot Perception and Autonomous Navigation Stack (move_base) [(optional)args map]
Don't forget to select map (611 , 6_floor , whiz-ex(default) )
```sh
roslaunch som_o_navigation nav.launch map:="6_floor"
```
3. Run Teleoperation Node 
```sh
roslaunch som_o_teleop teleop_joystick.launch 
```
