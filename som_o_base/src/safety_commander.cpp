/******************************
 *  Safety controller [CMD_VEL_CUT] !
 *  Author : Theppasith Nisitsukcharoen , Sukhum Sattaratnamai
 *  Date : 17-Nov-2016
 *******************************/
#include <string>
#include <iostream>
#include <ros/ros.h>

// ROS Data Structure
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h> 

geometry_msgs::Twist last_cmd_vel , received_cmd_vel;
bool stop_flag = false;

void move_base_stat_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    if( msg->status_list.size() == 0 ){
        stop_flag = false;
        return;
    }
    bool canContinue = msg->status_list[0].status == 2 ||  msg->status_list[0].status == 1 ||  msg->status_list[0].status == 4 ||  msg->status_list[0].status == 6;
    if( canContinue ){
        stop_flag = false;
    }else{
        stop_flag = true;
    }
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    received_cmd_vel.linear.x = msg->linear.x;
    received_cmd_vel.angular.z = msg->angular.z;
}

void move_base_cancel_callback(const actionlib_msgs::GoalID::ConstPtr& msg){
    stop_flag = true;
}

int main(int argc, char **argv){

    // Node Entry
    ros::init(argc, argv, "som_o_safety_commander");
    // Node Handle (Private namespace ~)
    ros::NodeHandle nh;

    // Subscribe to command velocity
    ros::Subscriber cmd_vel_sub         = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",1,cmd_vel_callback);

    // Move base sub 
    ros::Subscriber move_base_stat_sub    = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status",1,move_base_stat_callback);
    ros::Subscriber move_base_cancel_sub  = nh.subscribe<actionlib_msgs::GoalID>("/move_base/cancel",1,move_base_cancel_callback);

    // Odometry Publisher
    ros::Publisher last_cmd_vel_pub     = nh.advertise<geometry_msgs::Twist>("/som_o/cmd_vel", 10);

    // Loop update Rate
    ros::Rate rate(50);

    while (ros::ok()) {

        if(stop_flag){
            last_cmd_vel = *new geometry_msgs::Twist();
            ROS_INFO(" FUCK STOP STOP STOP ");
        }else{
            last_cmd_vel = received_cmd_vel;
        }

      last_cmd_vel_pub.publish(last_cmd_vel); 

      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}
