/******************************
 *  ROS Node
 *  Publisher 20Hz
 *  14-Sept-2016
 *******************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <actionlib_msgs/GoalID.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>

/****************************************************************/
// Global Variables

std::string               twist_pub_topic_name_,joystick_sub_topic_name_,laser_sub_topic_name_;
const double              a0 = 1;
double                    vel_current;
double                    vel_max , vel_min;
double                    vel_next;
geometry_msgs::Twist      twist;
/****************************************************************/

void   joyCallback(const sensor_msgs::Joy::ConstPtr& joy){



}
void   laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scans){



}
double velocity_function(float distance){


}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "publisher20hz");

    // Parameters
    twist_pub_topic_name_ = "cmd_vel";
    joystick_sub_topic_name_ = "joy";
    laser_sub_topic_name_ = "scan";

    // Node Handler
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher twist_pub_ = nh.advertise<geometry_msgs::Twist>(twist_pub_topic_name_, 1);

    // Subscriber
    ros::Subscriber joy_sub_         = nh.subscribe<sensor_msgs::Joy>(joystick_sub_topic_name_, 10,
                                         &joyCallback);
    ros::Subscriber laserscan_sub_   = nh.subscribe<sensor_msgs::LaserScan>(laser_sub_topic_name_, 10,
                                         &laserScanCallback);

    // Set the loop_rate
    ros::Rate loop_rate(20);

    // Begin the loop
    while (ros::ok())
    {
      // Create a message
          // Create as a global message geometry_msgs::Twist twist;

      // Edit Message payloads
          // Payload is set by the callback from joystick

      // Log file
      ROS_INFO("%f", twist.linear.x);

      // Publish the message
      twist_pub_.publish(twist);

      ros::spinOnce();

      loop_rate.sleep();

    }
}
