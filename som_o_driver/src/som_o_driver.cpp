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

// Robot default profile
#define CENTER_TO_WHEEL  0.082f  //82  MM
#define WHEEL_RADIUS     0.04f   //40  MM 
#define TICK_METER       262236  //
#define WHEEL_SEPERATION 0.164f  // 164 MM

// Global robot profile
double center_to_wheel , wheel_radius , max_wheel_rpm ;

// Robot data 
double    pos_x = 0 ;
double    pos_y = 0 ;
double    th    = 0 ;
double    linear;  //linear velocity
double    angular; //angular velocity 
ros::Time current_time;
ros::Time last_time;
int       last_tick_l;
int       last_tick_r;
int       tick_l,vel_l;
int       tick_r,vel_r;
ros::Publisher odom_pub;


// Serial Controller
som_o::Controller   *controller;

// Variable for Speed Control
double              linear_x, angular_z,vl,vr;
int                 leftSpeed,rightSpeed,speed,max_effort;


void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){

   // Receive linear and angular velocity from upper layer
   linear_x   = msg->linear.x;
   angular_z  = msg->angular.z;
   
   // Fix negative zero generated from joy node 
   linear_x   = ((fabs(linear_x) < 0.0005)? 0.000: linear_x);
   angular_z  = ((fabs(angular_z) < 0.0005)? 0.000: angular_z);

   // Assign speed to the particular wheel 
   double left_wheel_vel  = (linear_x - angular_z*WHEEL_SEPERATION/2)/WHEEL_RADIUS;  // rad /sec
   double right_wheel_vel = (linear_x + angular_z*WHEEL_SEPERATION/2)/WHEEL_RADIUS;  // rad /sec
	
   // Speed binding to effort (0-100%)
   // 1 rad/s = 9.5 rpm ~ 10 rpm 
   double vel_left  = (float)left_wheel_vel / (max_wheel_rpm);             // round/min
   double vel_right = (float)right_wheel_vel/ (max_wheel_rpm);             // round/min

   // Speed bounding 
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
    leftSpeed  = 1 * vel_left  * max_effort ;
    rightSpeed = -1 * vel_right * max_effort ;

   //ROS_INFO("maxeff : %d Assign : %d , %d ",max_effort , leftSpeed ,rightSpeed );

}

void update(){
    static tf::TransformBroadcaster odom_broadcaster;

    // Get current timestamp
    current_time = ros::Time::now();

    // Generate ROS Data 
    // By Reading the Controller's Data
    int left_tick  =      controller->getTickL();
    int right_tick = -1 * controller->getTickR();

    vel_l =     controller->getVelL();
    vel_r = -1 *controller->getVelR();

    double dt       = (current_time - last_time).toSec();

    // Delta of Encoders
    double deltaLeft       = left_tick - last_tick_l;
    double deltaRight      = right_tick - last_tick_r;

    // Convert Ticks To Meter
    double deltaLeft_meter  = deltaLeft /TICK_METER;
    double deltaRight_meter = deltaRight/TICK_METER;

    // Memories the Encoder Value in this State
    last_tick_r  = right_tick;
    last_tick_l   = left_tick;

    // Calculate actual distance traveled 
    double actualDistance   = (deltaRight_meter + deltaLeft_meter)/2;
    double theta            = (deltaRight_meter - deltaLeft_meter)/(WHEEL_SEPERATION);

    // [ODOM] Calculate Velocities
    linear = actualDistance / dt;
    angular = theta / dt;

    if(actualDistance != 0)
    {
        // [TF+ODOM] Calculate Distance Traveled in (x,y) Format 
        double temp_x =  cos(theta) * actualDistance;
        double temp_y = -sin(theta) * actualDistance;
        // Calculate Final position
        pos_x += ( cos(th) * temp_x - sin(th) * temp_y );
        pos_y += ( sin(th) * temp_x + cos(th) * temp_y );
    }
  
    if(theta !=0)
    {
        th += theta;
    }

    // Publish TF for the moving Part
    /// since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    /// first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = pos_x;
    odom_trans.transform.translation.y = pos_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // ODOM
    /// next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    /// set the position
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    /// set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = linear;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = angular;

    /// publish the message
    odom_pub.publish(odom);  

    last_time = current_time;

}

void main_loop(){
    // Read data from the controllers
    controller->sendCommand(controller->setEncVelRead('L'));
    controller->readEncVel_L();
    controller->sendCommand(controller->setEncVelRead('R'));
    controller->readEncVel_R();

    // Update ROS data (Publishing things)
    update();

    // Write command received from Upper layer 
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
    
    // Default Effort for the driver Board [ 0 - 20000 ]
    max_effort = 20000;

    // Get Parameter from ROS Parameter Server (if Exist)
    nh.param<std::string>("port", port, port);
    nh.param<int32_t>("baud_rate",baud ,baud);
    nh.param<double>("loop_rate",loop_rate ,loop_rate);
    nh.param<int>("max_effort",max_effort,max_effort);
    
    max_wheel_rpm = (float)max_effort / 1000.0f ;
    //std::cout << "MAX WHEE : " << max_wheel_rpm <<std::endl;

    // Get Robot Parameter from ROS Parameter Server (if Exist)
    nh.param<double>("wheel_saparation",center_to_wheel ,CENTER_TO_WHEEL);
    nh.param<double>("wheel_radius",wheel_radius ,WHEEL_RADIUS);

    

    // Prompt User After Settings are completed
    ROS_INFO("[Driver] Create connection to Port : %s" , port.c_str());
    ROS_INFO("[Driver] with Baud Rate : %d" , baud);

    // Subscribe to command velocity
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",1,cmd_vel_callback);

    // Odometry Publisher
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);

    // Serial Controller Initiated !
    controller = new som_o::Controller(port.c_str(),baud);

    // Loop update Rate
    ros::Rate rate(loop_rate);

    while (ros::ok()) {
      ROS_DEBUG("Attempting connection to %s at %i baud.", port.c_str(), baud);
      // Attempting to connect the driver board
      // Connect to Serial controller
      if(!controller->is_connected())
      {
        controller->connect();
      }

      // Connected to Serial controller -> Do main routine.
      if (controller->is_connected()) 
      {
          main_loop();
      }
      // Cannot connect -> Report 
      else 
      {
        ROS_DEBUG("Problem connecting to serial device.");
        ROS_ERROR_STREAM_ONCE("Problem connecting to port " << port << ". Trying again every 1 second.");
        sleep(1); // linux sleep second
      }

      // Maintain loop speed and callback polling 
      ros::spinOnce();
      rate.sleep();
    }
    // Out loop -> Stop everything
    controller->stop();
    return 0;
}
