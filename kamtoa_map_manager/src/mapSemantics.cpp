/******************************
 *  Map Semantics Manager Node
 *  - Translate any human command navigation
      action on to robot language
    - The only place to resolve the index number
      into the target goal position
 *  Author : Theppasith Nisitsukcharoen
 *  29-Sept-2016
 *******************************/

/******************************
  Update 3-Nov-2016
  @Provided Services
  1. List All POI
    - return poi[]
  2. Resolve POI (by index)
    - return move_base_goal

  @Messages
  1. poi.msg - a single poi

*******************************/

// DEPRECATED - this is for Console app testing
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "kamtoa_map_manager/fileReader.hpp"
#include <vector>
#include <iostream>
#include <string>

#include <visualization_msgs/Marker.h>


// POI Array
std::vector<move_base_msgs::MoveBaseGoal> poi_array;
std::vector<std::string>   poi_array_name;
ros::Publisher goToPub;
ros::Publisher marker_pub;

// Provide Service to Give the users Location List



void list_poi(){
  std::cout << "========================" << std::endl;
  std::cout << " POI Lists ." <<std::endl;
  std::cout << "========================" << std::endl;
  std::cout << " No.     Name." << std::endl;
  std::cout << "========================" << std::endl;
  for(int i = 0 ; i < poi_array.size() ; i++){
      std::cout << " " << i << "\t" ;
      std::cout << poi_array_name[i] << std::endl;
  }
  std::cout << "========================" << std::endl;
}

void goTo(int poi_number){

    move_base_msgs::MoveBaseGoal msg;

    msg.target_pose.header.frame_id = "/map";
    msg.target_pose.pose.position.x    = poi_array[poi_number].target_pose.pose.position.x;
    msg.target_pose.pose.position.y    = poi_array[poi_number].target_pose.pose.position.y ;
    tf::Quaternion q = tf::createQuaternionFromRPY( 0 ,0, 0);
    msg.target_pose.pose.orientation.x = q.x();
    msg.target_pose.pose.orientation.y = q.y();
    msg.target_pose.pose.orientation.z = q.z();
    msg.target_pose.pose.orientation.w = q.w();

    goToPub.publish(msg);

}


int main(int argc, char** argv)
{
     ros::init(argc, argv, "Map_Semantics_Manager_Node");
     ros::NodeHandle n;

     // Publishers
     goToPub = n.advertise<move_base_msgs::MoveBaseGoal>("/kamtoa/goal", 10);
     marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

     // Default Routine
     // Read the POIs from file
     ROS_INFO("Read the waypoint from file : waypoint.csv ");
     FileReader::read_waypoint_from_file("/waypoints/waypoint.csv",
                              &poi_array , &poi_array_name);



     while(ros::ok()){

        list_poi();
        std::cout << "enter poi : " << std::endl;
        int in;
        std::cin >> in;

        goTo(in);

        ros::spinOnce();
     }

}
