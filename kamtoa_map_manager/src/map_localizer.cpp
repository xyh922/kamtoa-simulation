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


cv::Vec3b setBrush(int roomNum){
    switch(roomNum){
        case 0:
            return cv::Vec3b(255,255,255);
            break;
        case 10:
            return cv::Vec3b(255,0,0);
            break;
        case 20:
            return cv::Vec3b(0,255,0);
            break;
        case 30:
            return cv::Vec3b(0,0,255);
            break;
        case 40:
            return cv::Vec3b(160,40,210);
            break;
        case 50:
            return cv::Vec3b(255,175,130);
            break;
        case 60:
            return cv::Vec3b(0,255,255);
            break;
        case 70:
            return cv::Vec3b(255,255,0);
            break;
        case 80:
            return cv::Vec3b(255,0,255);
            break;
        case 90:
            return cv::Vec3b(255,225,255);
            break;

        default:
            return cv::Vec3b(255,255,255);
        break;
    }
}

std::string roomname(int id){
    switch(id){
        case 10 :
        return "Entrance / Dinner desk";
        case 20 :
        return "Living Room / TV";
        case 30 :
        return "Kitchen Area";
        case 40 :
        return "Hallway Area";
        case 50 :
        return "Mini Bedroom";
        case 60 :
        return "Bedroom 1";
        case 70 :
        return "Bedroom 2";
        case 80 :
        return "Mini Toilet";
        case 90 :
        return "Main Toilet";
        case 0 :
        return "WALL";
        case 205 :
        return "UNKNOWN";
        case 254 :
        return "UNDEFINED";

    }
}

void editDrawing(){

  for(unsigned int row = 0; row < drawing.rows; ++row){
      for(unsigned int col = 0; col < drawing.cols; ++col){
          int readCell = (int)drawing.at<cv::Vec3b>(row,col)[0];
          if(readCell == 255 || readCell == 205 || readCell == 0)continue;
          cv::Vec3b color = setBrush(readCell);
          drawing.at<cv::Vec3b>(row,col) = color;
      }
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
    editDrawing();
    drawing.copyTo(reader2);

    cv::namedWindow( "draw", WINDOW_AUTOSIZE );
    cv::resizeWindow( "draw" , 512 , 512);
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

        cv::circle(drawing ,cv::Point(row,col) , 5 , cv::Scalar(0,0,0) , -1);
        cv::circle(drawing ,cv::Point(row,col) , 3 , cv::Scalar(100,255,30) , -1);
        cv::imshow("draw",drawing);
        cv::waitKey(5);
        int room_value = (int) reader.at<uchar>(col,row);
        ROS_INFO("GRID(%d,%d) =  %d = %s",col,row,room_value,roomname(room_value).c_str());

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
