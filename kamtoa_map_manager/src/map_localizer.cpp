#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <iostream>
#include <string>
cv::Mat reader,reader2,imageMap,drawing;
nav_msgs::OccupancyGrid *map;
// Real World position and Map Resolution 
double pos_x,pos_y,res,map_x,map_y;
// Grid Position with respect to Real World
unsigned int grid_x,grid_y;

using namespace cv;

void onReceiveMeta(const nav_msgs::MapMetaData::ConstPtr &meta_receive){
    pos_x   = meta_receive->origin.position.x;
    pos_y   = meta_receive->origin.position.y;
    res     = meta_receive->resolution;
}

//Request map as Service Caller 
void mouse_callback_draw(int event, int x, int y, int flags, void* userdata)
{
     int cell_value = (int)reader.at<schar>(y,x);

     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          std::cout << "(" << x << "," << y << ")= " << cell_value << std::endl;          
     } 
}

int main(int argc, char** argv){
    // Init node
    ros::init(argc, argv, "Grid_localizer_Node");
    ros::NodeHandle node;
    tf::StampedTransform transform;
    tf::TransformListener listener;
    ros::Subscriber map_meta = node.subscribe<nav_msgs::MapMetaData>("/map_metadata", 30,onReceiveMeta);
    
    // Load Map Localization 
    
    std::string path = ros::package::getPath("kamtoa_navigation")+"/maps/"+"fin.pgm";
    reader = cv::imread(path,CV_LOAD_IMAGE_UNCHANGED);
    reader2 = cv::imread(path,CV_LOAD_IMAGE_COLOR);
    imageMap = cv::Mat(reader.rows,reader.cols,CV_8UC1);
    drawing = cv::Mat(reader.rows,reader.cols,CV_8UC3);
    reader.copyTo(imageMap);
    reader2.copyTo(drawing);
    cv::namedWindow( "draw", WINDOW_AUTOSIZE );
    cv::setMouseCallback("draw", mouse_callback_draw , NULL);

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
        reader2.copyTo(drawing);
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

        unsigned int row = grid_y;
        unsigned int col = 1024-grid_x;
        
        cv::circle(drawing ,cv::Point(row,col) , 5 , cv::Scalar(0,0,255) , -1);

        cv::imshow("draw",drawing);
        cv::waitKey(5);
        ROS_INFO("GRID CALCULATION (%d,%d) =  %d",col,row,reader.at<uchar>(col,row));
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
