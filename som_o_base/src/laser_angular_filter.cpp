#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
using namespace std;

// index 0 => X-
/*
  Model : RP-Lidar A2
  start at 0 front 
  go -3.15 to the left (counter-clockwise)
          -3.15  0  +3.15
            \    |    /
             \   |   /
              \  |  /
               \ | /
                \|/
                 O
              /     \
             /       \
            /         \
           /  radian   \
     -1.57            1.57           

*/

ros::Publisher filtered_scan_pub;
ros::Subscriber input_scan_sub;
sensor_msgs::LaserScan filtered_scan;
double lower_angle_ = -3.15;
double pole_left = -1.57;
double pole_right = 1.57;
double upper_angle_ = 3.15;

bool my_update(const sensor_msgs::LaserScan& input_scan)
{
  filtered_scan.ranges.resize(input_scan.ranges.size());
  filtered_scan.intensities.resize(input_scan.intensities.size());
  double start_angle   = input_scan.angle_min;
  double current_angle = input_scan.angle_min;
  ros::Time start_time = input_scan.header.stamp;
  unsigned int count = 0;
  lower_angle_ = input_scan.angle_min;
  upper_angle_ = input_scan.angle_max;

  //loop through the scan and truncate the beginning and the end of the scan as necessary
  for (unsigned int i = 0; i < input_scan.ranges.size(); ++i){
    //wait until we get to our desired starting angle
    //Mark Invalid !
    if(start_angle < lower_angle_)
    {
      start_angle += input_scan.angle_increment;
      current_angle += input_scan.angle_increment;
      start_time += ros::Duration(input_scan.time_increment);
    }
    else 
    {
    // Invalid Data = INF  
      if(current_angle < pole_right && current_angle > pole_left){
          filtered_scan.ranges[count] = std::numeric_limits<float>::infinity();
      }
      else
      {
        // Valid Data
      filtered_scan.ranges[count] = input_scan.ranges[i];
      }
      //make sure  that we don't update intensity data if its not available
      if (input_scan.intensities.size() > i)
        filtered_scan.intensities[count] = input_scan.intensities[i];
      count++;
      //check if we need to break out of the loop, basically if the next increment will put us over the threshold
      if (current_angle + input_scan.angle_increment > upper_angle_) {
        break;
      }
      current_angle += input_scan.angle_increment;
    }
  }
 
  //make sure to set all the needed fields on the filtered scan
  filtered_scan.header.frame_id = input_scan.header.frame_id;
  filtered_scan.header.stamp = start_time;
  filtered_scan.angle_min = start_angle;
  filtered_scan.angle_max = current_angle;
  filtered_scan.angle_increment = input_scan.angle_increment;
  filtered_scan.time_increment = input_scan.time_increment;
  filtered_scan.scan_time = input_scan.scan_time;
  filtered_scan.range_min = input_scan.range_min;
  filtered_scan.range_max = input_scan.range_max;
 
  filtered_scan.ranges.resize(count);

  if (input_scan.intensities.size() >= count)
    filtered_scan.intensities.resize(count);

  ROS_DEBUG("Filtered out %d points from the laser scan.", (int)input_scan.ranges.size() - (int)count);
  return true;
}

void input_scan_sub_callback(const sensor_msgs::LaserScan& input_scan)
{
  my_update(input_scan);
  filtered_scan_pub.publish(filtered_scan);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "Angular_Bound_Laser_filter");
  ros::NodeHandle n1, n2;
  input_scan_sub = n1.subscribe("/scan", 10, input_scan_sub_callback);
  filtered_scan_pub = n2.advertise<sensor_msgs::LaserScan>("scan_filtered", 50);
  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}