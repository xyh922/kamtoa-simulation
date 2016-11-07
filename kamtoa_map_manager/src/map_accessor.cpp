#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <vector>
#include <iostream>
#include <string>

nav_msgs::OccupancyGrid *map;
double pos_x,pos_y,res,map_x,map_y;
void onReceiveMeta(const nav_msgs::MapMetaData::ConstPtr &meta_receive){
    pos_x = meta_receive->origin.position.x;
    pos_y = meta_receive->origin.position.y;
    res = meta_receive->resolution;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  ros::Subscriber map_meta = node.subscribe<nav_msgs::MapMetaData>("/map_metadata", 30,onReceiveMeta);
 
  ros::Rate rate(10.0);
  while (ros::ok()){
    
    try{
      listener.lookupTransform("/map", "/base_footprint",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    unsigned int grid_x = (unsigned int)((transform.getOrigin().x() - pos_x) / res);
    unsigned int grid_y = (unsigned int)((transform.getOrigin().y() - pos_y)/ res);
    ROS_INFO("GRID CALCULATION (%d,%d)",grid_x,grid_y);
  
 
    ros::spinOnce();

    rate.sleep();
  }
  return 0;
};
