/******************************
 *  Point Transporter and Navigator
 *  Receive Custom Goal Message from Upper layer
 *  and traslate to "goalMessage"
 *  Author : Theppasith Nisitsukcharoen
 *  29-Sept-2016
 *******************************/

#include "kamtoa_navigation/pointTransporter.hpp"
#include <vector>
#include <iostream>
#include <string>


//Prototype For MoveBaseClient Actionlib
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient ac("move_base", true);

move_base_msgs::MoveBaseGoal  marked_current_pos;
move_base_msgs::MoveBaseGoal  marked_goal;

void goalDoneCallback_state(const actionlib::SimpleClientGoalState &state,
                            const move_base_msgs::MoveBaseResultConstPtr &result){


}
void goalActiveCallback(){
  //ROS_INFO("Goal active! Hurray!");
}


void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
  //ROS_INFO("Getting feedback! How cool is that?");
}

void set_goal(){
    move_base_msgs::MoveBaseGoal goal;

    // Send a goal to the robot
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Set the goal
    goal.target_pose.pose.position.x    = marked_goal.target_pose.pose.position.x;
    goal.target_pose.pose.position.y    = marked_goal.target_pose.pose.position.y;
    goal.target_pose.pose.orientation.x = marked_goal.target_pose.pose.orientation.x;
    goal.target_pose.pose.orientation.y = marked_goal.target_pose.pose.orientation.y;
    goal.target_pose.pose.orientation.z = marked_goal.target_pose.pose.orientation.z;
    goal.target_pose.pose.orientation.w = marked_goal.target_pose.pose.orientation.w;

    ac.sendGoal(goal,
                boost::bind(&goalDoneCallback_state, _1, _2),
                boost::bind(&goalActiveCallback),
                boost::bind(&goalFeedbackCallback, _1));
}


// Current Location Manager
void markCurrentLocation(){

    // TF Variables
    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Time stamp for this trasformation
    ros::Time now = ros::Time::now();

    // Listen to the transformation (TF) of map and base_footprint
    try {
        listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return;
    }

    // Store value to current Session (use copy constructor)
    marked_current_pos.target_pose.pose.position.x    = transform.getOrigin().x();
    marked_current_pos.target_pose.pose.position.y    = transform.getOrigin().y();
    marked_current_pos.target_pose.pose.orientation.x = transform.getRotation().x();
    marked_current_pos.target_pose.pose.orientation.y = transform.getRotation().y();
    marked_current_pos.target_pose.pose.orientation.z = transform.getRotation().z();
    marked_current_pos.target_pose.pose.orientation.w = transform.getRotation().w();
}


 //Callback Received Goal From Upper Level



 int main(int argc, char** argv)
 {
     ros::init(argc, argv, "transporter_to_point");

     ros::spin();
 }
