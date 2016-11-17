/******************************
 *  Node For SOM-O Motor Driver Board !
 *  Author : Theppasith Nisitsukcharoen
 *  Date : 17-Nov-2016
 *******************************/

#include "som_o_driver/controller.h"
 #include <string>
 #include <iostream>
 #include <ros/ros.h>
 #include <termios.h>

 // ROS Data Structure
 #include <geometry_msgs/Twist.h>
 #include <geometry_msgs/Vector3.h>
 #include <tf/transform_broadcaster.h>
 #include <nav_msgs/Odometry.h>


som_o::Controller *controller;
double              linear_x, angular_z,vl,vr;
int                 speed,left_dir , right_dir , encoder_request_button;



void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){

   linear_x = msg->linear.x;
   angular_z = msg->angular.z;

   if(linear_x == -0.00)linear_x = 0;
   if(angular_z == -0.00)angular_z = 0;

   speed = angular_z * 400;
}



int main(int argc, char **argv){
    // Node Entry
    ros::init(argc, argv, "som_o_driver_board");
    // Node Handle (Private namespace ~)
    ros::NodeHandle nh("~");

    // Serial parameter default settings
    std::string     port    =   "/dev/ttyUSB0";
    int32_t         baud    =   115200;

    // Get Paramter from ROS Parameter Server (if Exist)
    nh.param<std::string>("port", port, port);
    nh.param<int32_t>("baud_rate",baud ,baud);

    // Prompt User After Settings are completed
    ROS_INFO("[SOM-O-Driver] Create connection to Port : %s" , port.c_str());
    ROS_INFO("[SOM-O-Driver] with Baud Rate : %d" , baud);

    // Subscribe to Joystick Command
    ros::Subscriber sub = nh.subscribe("/cmd_vel",1,cmd_vel_callback);

    // Serial Controller Initiated !
    controller = new som_o::Controller(port.c_str(),baud);

    // Spinner which poll for callback

    bool initialize = false;

    while (ros::ok()) {
      ROS_DEBUG("Attempting connection to %s at %i baud.", port.c_str(), baud);
      if(!controller->is_connected()){
        controller->connect();
      }
      if (controller->is_connected()) {
        // Single time Initializer
        controller->sendCommand(controller->setVelCmd(200));
        controller->read();
        ros::spinOnce();
      } else {
        ROS_DEBUG("Problem connecting to serial device.");
        ROS_ERROR_STREAM_ONCE("Problem connecting to port " << port << ". Trying again every 1 second.");
        sleep(1);
      }
    }
    //End Loop
    //spinner.stop();
    //controller->send_stop();
    return 0;


}
