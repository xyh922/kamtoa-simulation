/******************************
 *  ROS Node
 *  to Create Velocity Function
 *  related to the LaserScan in front of Robot
 *  bottom = 0 degree , top = 180 degree
 *  0-359 Readings
 *  14-Sept-2016
 *******************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <actionlib_msgs/GoalID.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>

/****************************************************************/
class VelocityProfiler
{
     public:
     VelocityProfiler();
     geometry_msgs::Twist twist;
     geometry_msgs::Twist next_twist;
     void   publishTwist();

     bool   isActivate();
     private:
     void   joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
     void   laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scans);
     double velocity_function(float distance);

     ros::NodeHandle   nh_;
     ros::Publisher    twist_pub_;
     ros::Subscriber   joy_sub_,laserscan_sub_;

     // Params
     std::string       twist_pub_topic_name_;
     std::string       joystick_sub_topic_name_;
     std::string       laser_sub_topic_name_;

     // Teleop Boolean Switch
     bool              off_teleop;
     int               startAngle , endAngle;
     int               linear_   , angular_ , deadman_;
     double            l_scale_  , a_scale_;
     float             global_nearest;

     bool              activate;
};
 /****************************************************************/

VelocityProfiler::VelocityProfiler()
{
     twist_pub_topic_name_ = "cmd_vel";
     joystick_sub_topic_name_ = "joy";
     laser_sub_topic_name_ = "scan";
     startAngle = 170;
     endAngle = 190;
     nh_.param("twist_pub_topic", twist_pub_topic_name_);
     nh_.param("joystick_sub_topic", joystick_sub_topic_name_);

     nh_.param("axis_linear",    linear_,1);
     nh_.param("button_deadman_switch",    deadman_,2);
     nh_.param("axis_angular",   angular_, 3  );
     nh_.param("scale_angular",  a_scale_, a_scale_  );
     nh_.param("scale_linear",   l_scale_, l_scale_  );
     nh_.param("start_angle",  startAngle);
     nh_.param("end_angle",   endAngle);

     //Publisher and Subscriber
     twist_pub_       = nh_.advertise<geometry_msgs::Twist>(twist_pub_topic_name_, 1);
     joy_sub_         = nh_.subscribe<sensor_msgs::Joy>(joystick_sub_topic_name_, 10,
                                          &VelocityProfiler::joyCallback, this);
     laserscan_sub_   = nh_.subscribe<sensor_msgs::LaserScan>(laser_sub_topic_name_, 10,
                                          &VelocityProfiler::laserScanCallback, this);
}


void VelocityProfiler::publishTwist()
{
      twist_pub_.publish(twist);
}

bool VelocityProfiler::isActivate(){
      return activate;
}

void VelocityProfiler::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

      // Edit cmd_vel payload according to joystick value
      twist.linear.x    = joy->axes[linear_];

      // Deadman 's Switch
      int deadman_triggered,manual_deadman;
      deadman_triggered = joy->axes[deadman_];
      manual_deadman =  joy->buttons[4];

      // Deadman Triggered Activated
      if(deadman_triggered == -1 && !manual_deadman){
        // get the LaserScan most nearest distance (in bound angle) which we calculated before
        float nearest = global_nearest;

        twist.linear.x =  velocity_function(nearest) * joy->axes[linear_];

        activate = true; //off_teleop = false;
      }
      // Deadman Triggered OFF
      else if (deadman_triggered != -1 && !manual_deadman){
          twist = *new geometry_msgs::Twist();
          //twist_pub_.publish(*new geometry_msgs::Twist());        //Publish 0,0,0 (stop)
          activate = false; //off_teleop = true;                                      //Put the Teleop Off
          return;
      }

      if(deadman_triggered == 1 && manual_deadman){
        //Read Velocities Value From Joystick
        twist.angular.z = a_scale_*joy->axes[angular_];
        twist.linear.x = l_scale_*joy->axes[linear_];
        activate = true; //off_teleop = false;
      }else if(deadman_triggered == 1 && !manual_deadman){
        twist = *new geometry_msgs::Twist();
        //twist_pub_.publish(*new geometry_msgs::Twist());        //Publish 0,0,0 (stop)
        activate = false; //off_teleop = true;                                      //Put the Teleop Off
        return;
      }


}

void VelocityProfiler::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scans)
{

      float local_nearest = 100.0;
      for(int i = startAngle ; i < endAngle ; i++){
          local_nearest = std::min(scans->ranges[i] , local_nearest);
      }
      //std::cout << "Minimum : " << local_nearest << std::endl;
      global_nearest = local_nearest;

      // For Console Test
      // velocity_function(global_nearest);

}

double VelocityProfiler::velocity_function(float distance)
{
      //std::cout << "Vel Func Call : " << distance << std::endl;
      return 1;
}
int main(int argc, char** argv)
{
     ros::init(argc, argv, "Velocity_Profiler_Node");
     VelocityProfiler vel_profiler;

     // Set the loop_rate
     ros::Rate loop_rate(20);

     // Begin the loop
     while (ros::ok())
     {
       // Create a message
           // Create as a global message geometry_msgs::Twist twist;

       // Edit Message payloads
           // Payload is set by the callback from joystick

       // Log file to console
       if(vel_profiler.isActivate()){
          ROS_INFO("[Activated] %f", vel_profiler.twist.linear.x);
       }else{
          ROS_INFO("[Off] %f", vel_profiler.twist.linear.x);
       }
       // Publish the message
       if(vel_profiler.isActivate()){
          vel_profiler.publishTwist();
       }

       ros::spinOnce();

       loop_rate.sleep();

     }
}
