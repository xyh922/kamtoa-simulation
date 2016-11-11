/******************************
 *  Map Semantics Manager Node
 *  - Translate any human command navigation
      action on to robot language
    - The only place to resolve the index number
      into the target goal position
 *  Author : Theppasith Nisitsukcharoen
 *  3-Nov-2016

  @Provided Services
  1. List All POI
    - return poi[]
  2. Load POI From Root path : kamtoa_navigation Package
  3. Resolve POI (by index)
    - return move_base_goal ?
  4. GoTo POI by poi's index_id

  @Messages
  1. poi.msg - a single poi
*******************************/


#include "kamtoa_map_manager/poiManager.hpp"
#include <vector>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
     ros::init(argc, argv, "Map_Semantics_Manager_Node");
     // Create POI Manager Node Server
     POIManager poi_manager;
     ros::NodeHandle n;

     // Provide Services
     ros::ServiceServer getPoiService = n.advertiseService("/get_poi_list", &POIManager::poi_callback, &poi_manager);
     // Load New POI From file
     ros::ServiceServer loadPoiService = n.advertiseService("/load_poi_file", &POIManager::load_poi_callback, &poi_manager);
     // Resolve poi by index
     ros::ServiceServer resolvePoiService = n.advertiseService("/resolve_poi", &POIManager::resolve_poi_callback, &poi_manager);
     // Go to POI Service
     ros::ServiceServer gotoPoiService = n.advertiseService("/goto_poi", &POIManager::goto_poi_callback, &poi_manager);

     ROS_INFO("POI Manager Server initialized !");

     // Spinning for Callbacks
     ros::spin();

}
