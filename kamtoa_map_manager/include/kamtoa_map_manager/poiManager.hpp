/******************************
 *  Map Semantics Manager Node
 *  - Translate any human command navigation
      action on to robot language
    - The only place to resolve the index number
      into the target goal position
 *  Author : Theppasith Nisitsukcharoen
 *  3-Nov-2016
 *******************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <vector>
#include <iostream>
#include <string>

// Visualization Marker to RVIZ
#include <visualization_msgs/Marker.h>
// Custom POI Message files template to be sent.
#include <kamtoa_map_manager/poi.h>
// Custom Services headers
#include <kamtoa_map_manager/listPoi.h>
#include <kamtoa_map_manager/loadPoi.h>
#include <kamtoa_map_manager/resolvePoi.h>
#include <kamtoa_map_manager/gotoPoi.h>
// CSV File Reader Utility
#include "kamtoa_map_manager/fileReader.hpp"

class POIManager
{
    public:
    POIManager();
    ~POIManager();
    std::vector<move_base_msgs::MoveBaseGoal>   poi_array;
    std::vector<std::string>                    poi_array_name;

    // Function for services implementation goes here
    bool poi_callback(kamtoa_map_manager::listPoi::Request &req,
                      kamtoa_map_manager::listPoi::Response &res);
    bool load_poi_callback(kamtoa_map_manager::loadPoi::Request &req,
                      kamtoa_map_manager::loadPoi::Response &res);
    bool resolve_poi_callback(kamtoa_map_manager::resolvePoi::Request &req,
                      kamtoa_map_manager::resolvePoi::Response &res);
    bool goto_poi_callback(kamtoa_map_manager::gotoPoi::Request &req,
                      kamtoa_map_manager::gotoPoi::Response &res);


    void load_poi_from_file(std::string path);


    private:
    ros::NodeHandle   nh_;          // NodeHandle
    ros::Publisher    goalPub;      // Publisher for sending goal
    std::string       goal_topic;   // Topic name to send move_base_goal_msg to
    std::string       poi_path;     // POI file path

};

// Class Constructor
POIManager::POIManager(void)
{
    // Assign default parameters
    goal_topic    =   "/kamtoa/goal";
    poi_path      =   "/waypoints/waypoint.csv";

    // Get parameters from parameter server
    nh_.param("goal_topic" , goal_topic , goal_topic );
    nh_.param("poi_path", poi_path , poi_path);

    // Publisher for sending goal
    goalPub = nh_.advertise<move_base_msgs::MoveBaseGoal>(goal_topic, 10);

    // Initialize the poi loading
    // ROOT DIRECTORY : kamtoa_navigation package
    ROS_INFO("[POI Manager] Load POI based on root package : kamtoa_navigation ");
    load_poi_from_file("/waypoints/waypoint.csv");

}

// Class Destructor
POIManager::~POIManager(void)
{
    std::cout << "POIManager destructor called" <<std::endl;
}

bool POIManager::poi_callback(
    kamtoa_map_manager::listPoi::Request &req,
    kamtoa_map_manager::listPoi::Response &res)
{
    ROS_INFO("[POI Manager] Serving POI Lookup !");

    for(int i = 0 ; i < this->poi_array.size() ; i++){
        kamtoa_map_manager::poi test_poi;
        test_poi.id = i;
        test_poi.poi_name = poi_array_name[i];
        res.poi_list.push_back(test_poi);
    }

    res.file_path = poi_path;

    return true;
}

bool POIManager::load_poi_callback(
    kamtoa_map_manager::loadPoi::Request &req,
    kamtoa_map_manager::loadPoi::Response &res)
{

    ROS_INFO("[POI Manager] Serving POI Load from file !");
    std::cout << req.path << std::endl;
    load_poi_from_file(req.path);

    return true;
}

void POIManager::load_poi_from_file(std::string path){

    // [Reuseabillity] Clear value inside the vectors
    this->poi_array.clear();
    this->poi_array_name.clear();

    // FileReader
    FileReader::read_waypoint_from_file(path, &(this->poi_array) , &(this->poi_array_name));
    // Set Current path
    poi_path = path;
}


bool POIManager::resolve_poi_callback(
    kamtoa_map_manager::resolvePoi::Request &req,
    kamtoa_map_manager::resolvePoi::Response &res)
{
    move_base_msgs::MoveBaseGoal msg;

    msg.target_pose.header.frame_id = "/map";
    msg.target_pose.pose.position.x    = this->poi_array[req.id].target_pose.pose.position.x ;
    msg.target_pose.pose.position.y    = this->poi_array[req.id].target_pose.pose.position.y ;
    tf::Quaternion q = tf::createQuaternionFromRPY( 0 ,0, 0);
    msg.target_pose.pose.orientation.x = q.x();
    msg.target_pose.pose.orientation.y = q.y();
    msg.target_pose.pose.orientation.z = q.z();
    msg.target_pose.pose.orientation.w = q.w();

    res.resolveGoal = msg;

    return true;
}

bool POIManager::goto_poi_callback(kamtoa_map_manager::gotoPoi::Request &req,
                  kamtoa_map_manager::gotoPoi::Response &res)
{
    move_base_msgs::MoveBaseGoal msg;

    msg.target_pose.header.frame_id = "/map";
    msg.target_pose.pose.position.x    = this->poi_array[req.id].target_pose.pose.position.x ;
    msg.target_pose.pose.position.y    = this->poi_array[req.id].target_pose.pose.position.y ;
    tf::Quaternion q = tf::createQuaternionFromRPY( 0 ,0, 0);
    msg.target_pose.pose.orientation.x = q.x();
    msg.target_pose.pose.orientation.y = q.y();
    msg.target_pose.pose.orientation.z = q.z();
    msg.target_pose.pose.orientation.w = q.w();

    ROS_INFO("[POI Manager] Goto %d Published !", req.id);

    goalPub.publish(msg);

    return true;
}
