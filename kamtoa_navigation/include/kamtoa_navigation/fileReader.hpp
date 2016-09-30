/******************************
 *  CSV File Reader
 *  To Read the waypoints file (CSV Format)
 *  Author : Theppasith Nisitsukcharoen
 *  30-Sept-2016
 *******************************/
 #include <sstream>
 #include <fstream>
 #include <vector>
 #include <iostream>
 #include <string>
 #include <cstdlib>
 #include <termios.h>

void read_waypoint_from_file(std::string filename ,
    std::vector<move_base_msgs::MoveBaseGoal> *poi_array ,
    std::vector<std::string> *poi_array_name )
{

    std::vector<std::string> tokenized;
    std::string path = ros::package::getPath("kamtoa_navigation")+ filename;
    std::ifstream inFile(path.c_str());


    //First line is Waypoint Counts
    std::string line;
    getline(inFile,line);
    int waypoint_count = atoi(line.c_str());

    ROS_INFO("Waypoint Counts : %d" , waypoint_count);

    // Begin reading following lines
    while(getline(inFile,line)){
     // New Line
       std::stringstream strstr(line);
       std::string word = "";

     // First Column is the Place Name
       getline(strstr,word,',');
       poi_array_name->push_back(word);
     // Gathering location parameters
       while(getline(strstr,word,',')){
           tokenized.push_back(word);
       }
    }

    //Count All tokenized word
    size_t point_amount = (size_t)( tokenized.size() / 6.0);

    //Number must be matched with the header of the file
    if(point_amount == waypoint_count)
    {
      ROS_INFO("Waypoint file integrity check PASSED! ");
    }
    else
    {
      ROS_WARN("File Error");
      return;
    }

    // Split Tokenized word onto pose messages
    std::vector<std::string>::iterator word_it;
    word_it = tokenized.begin();

    // Create move_base GOAL and Push into vector !
    for(int point_index = 0  ; point_index < tokenized.size() ; point_index++){
       move_base_msgs::MoveBaseGoal newPoint;
       newPoint.target_pose.pose.position.x    = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.position.y    = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.x = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.y = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.z = std::atof((word_it++)->c_str());
       newPoint.target_pose.pose.orientation.w = std::atof((word_it++)->c_str());
       poi_array->push_back(newPoint);
       if(word_it == tokenized.end())break;
    }
 }
