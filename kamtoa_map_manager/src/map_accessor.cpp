#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <vector>
#include <iostream>
#include <string>

nav_msgs::OccupancyGrid *map;
// Real World position and Map Resolution 
double pos_x,pos_y,res,map_x,map_y;
// Grid Position with respect to Real World
unsigned int grid_x,grid_y;

void onReceiveMeta(const nav_msgs::MapMetaData::ConstPtr &meta_receive){
    pos_x   = meta_receive->origin.position.x;
    pos_y   = meta_receive->origin.position.y;
    res     = meta_receive->resolution;
}

//Request map as Service Caller 


int main(int argc, char** argv){
    // Init node
    ros::init(argc, argv, "Grid_Accessor_node");
    ros::NodeHandle node;
    tf::StampedTransform transform;
    tf::TransformListener listener;
    ros::Subscriber map_meta = node.subscribe<nav_msgs::MapMetaData>("/map_metadata", 30,onReceiveMeta);
    
    // Implement Get Map Service ( Single time usage )
    ros::ServiceClient mapClient = node.serviceClient<nav_msgs::GetMap>("/static_map");
    // Service slave
    nav_msgs::GetMap srv;
    // Call Service!
    if (mapClient.call(srv))
    {
        ROS_INFO("Map Service called Successfully !");
        nav_msgs::OccupancyGrid& map_const(srv.response.map);
        map = &map_const;
    }
    else
    {
        ROS_ERROR("Failed to call service = GETMAP");
        return 1;
    }

    // Loop Rate 
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

        grid_y = (unsigned int)((transform.getOrigin().x() - pos_x) / res);
        grid_x = (unsigned int)((transform.getOrigin().y() - pos_y) / res);
        ROS_INFO("GRID CALCULATION (%d,%d) =  %d",grid_x,grid_y,map->data[grid_x*map->info.height + grid_y]);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
