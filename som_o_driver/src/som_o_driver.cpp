/******************************
 *  Node For SOM-O Motor Driver Board !
 *  Hiveground Co.,Ltd
 *  Author : Theppasith Nisitsukcharoen , Sukhum Sattaratnamai
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


som_o::Controller   *controller;
double              linear_x, angular_z,vl,vr;
int                 leftSpeed,rightSpeed,speed,max_effort;


void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
   linear_x = msg->linear.x;
   angular_z = msg->angular.z;
   if(linear_x == -0.00)linear_x = 0;
   if(angular_z == -0.00)angular_z = 0;

   double vel_left  = (linear_x - angular_z);
   double vel_right = (linear_x + angular_z);

   if( fabs(vel_left) > 1.0 )
    {
      vel_right /= fabs(vel_left);
      vel_left /= fabs(vel_left);
    }

    if( fabs(vel_right) > 1.0 )
    {
      vel_left /= fabs(vel_right);
      vel_right /= fabs(vel_right);
    }

    // Assign Power to each wheels
    leftSpeed  = -1 * vel_left  * max_effort ;
    rightSpeed =      vel_right * max_effort ;
}

void update(){
    // Generate ROS Data over here
    // By Reading the Controller's Data
    // controller->getEncL (getters) , controller->enc_l( pub var) 
    // And PUBLISH !!!
}

void main_loop(){
    // READ
    controller->sendCommand(controller->setEncVelRead('L'));
    controller->readEncVel_L();
    controller->sendCommand(controller->setEncVelRead('R'));
    controller->readEncVel_R();

    // UPDATE ROS Data
    update();

    // WRITE
    controller->sendCommand(controller->setVelCmdL(leftSpeed));
    controller->readVelCmd();
   
    controller->sendCommand(controller->setVelCmdR(rightSpeed));
    controller->readVelCmd();

}

int main(int argc, char **argv){
    // Node Entry
    ros::init(argc, argv, "som_o_driver_board");
    // Node Handle (Private namespace ~)
    ros::NodeHandle nh("~");

    // Serial parameter default settings
    std::string     port    =   "/dev/ttyUSB0";
    int32_t         baud    =   115200;
    double          loop_rate = 100.0; //20
    
    // Default Effort for the driver Board [ 0 - 500 ]
    max_effort= 220;

    // Get Paramter from ROS Parameter Server (if Exist)
    nh.param<std::string>("port", port, port);
    nh.param<int32_t>("baud_rate",baud ,baud);
    nh.param<double>("loop_rate",loop_rate ,loop_rate);
    nh.param<int>("max_effort",max_effort,max_effort);

    // Prompt User After Settings are completed
    ROS_INFO("[Driver] Create connection to Port : %s" , port.c_str());
    ROS_INFO("[Driver] with Baud Rate : %d" , baud);

    // Subscribe to Joystick Command
    ros::Subscriber sub = nh.subscribe("/cmd_vel",1,cmd_vel_callback);

    // Serial Controller Initiated !
    controller = new som_o::Controller(port.c_str(),baud);

    // Loop update Rate
    ros::Rate rate(loop_rate);

    while (ros::ok()) {
      ROS_DEBUG("Attempting connection to %s at %i baud.", port.c_str(), baud);
      if(!controller->is_connected()){
        controller->connect();
      }
      if (controller->is_connected()) {
        // Board is connect - do the main loop
          main_loop();
          
      } else {
        ROS_DEBUG("Problem connecting to serial device.");
        ROS_ERROR_STREAM_ONCE("Problem connecting to port " << port << ". Trying again every 1 second.");
        sleep(1);
      }
      ros::spinOnce();
      rate.sleep();
    }
    controller->stop();
    return 0;
}
