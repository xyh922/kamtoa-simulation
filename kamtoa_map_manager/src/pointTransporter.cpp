/******************************
 *  Point Transporter and Navigator
 *  Receive Custom Goal Message from Upper layer
 *  and traslate to "goalMessage"
 *  Author : Theppasith Nisitsukcharoen
 *  29-Sept-2016
 *******************************/

#include "kamtoa_map_manager/pointTransporter.hpp"
#include <vector>
#include <iostream>
#include <string>

// [IN] : Move_base Goal
// [OUT] : Feedback from move_base

int main(int argc, char** argv)
{
     ros::init(argc, argv, "transporter_to_point");

     // Create point to point transportation object
     PointTransporter pt;
     ROS_INFO("Point Transport Node Initialized ");

     ros::spin();
}
