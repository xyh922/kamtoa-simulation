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
     void                 publishTwist();
     bool                 isActivate();
     void                 setVelocityInRange();
     geometry_msgs::Twist current_twist;
     geometry_msgs::Twist set_point_twist;
     ros::Time            last_time;

     private:
     void   joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
     void   laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scans);
     double velocity_function(float distance);


     ros::NodeHandle   nh_;
     ros::Publisher    twist_pub_,setpoint_pub_;
     ros::Subscriber   joy_sub_,laserscan_sub_;

     // Params
     std::string       twist_pub_topic_name_;
     std::string       joystick_sub_topic_name_;
     std::string       laser_sub_topic_name_;

     // Teleop Boolean Switch
     int               startAngle , endAngle;
     int               linear_   , angular_ , deadman_;
     double            l_scale_  , a_scale_;
     float             global_nearest;

     bool              activate;
     // Velocity Params
     double            acc;


};
 /****************************************************************/

VelocityProfiler::VelocityProfiler()
{
     twist_pub_topic_name_    = "/kamtoa/cmd_vel";
     joystick_sub_topic_name_ = "joy";
     laser_sub_topic_name_    = "scan";
     startAngle               = 170;
     endAngle                 = 190;
     acc                      = 0;

     nh_.param("twist_pub_topic", twist_pub_topic_name_,twist_pub_topic_name_);
     nh_.param("joystick_sub_topic", joystick_sub_topic_name_,joystick_sub_topic_name_);

     nh_.param("axis_linear",    linear_,1);
     nh_.param("button_deadman_switch",    deadman_,2);
     nh_.param("axis_angular",   angular_, 3  );
     nh_.param("scale_angular",  a_scale_, a_scale_  );
     nh_.param("scale_linear",   l_scale_, l_scale_  );

     // Angle Params
     nh_.param("start_angle",  startAngle,startAngle);
     nh_.param("end_angle",   endAngle,endAngle);

     // Acceleration
     nh_.param("acc",  acc , acc);

     std::cout << "Acc : " << acc <<std::endl;

     //Publisher and Subscriber
     twist_pub_       = nh_.advertise<geometry_msgs::Twist>(twist_pub_topic_name_, 1);
     setpoint_pub_     = nh_.advertise<geometry_msgs::Twist>("/setpoint", 1);

     joy_sub_         = nh_.subscribe<sensor_msgs::Joy>(joystick_sub_topic_name_, 10,
                                          &VelocityProfiler::joyCallback, this);
     laserscan_sub_   = nh_.subscribe<sensor_msgs::LaserScan>(laser_sub_topic_name_, 10,
                                          &VelocityProfiler::laserScanCallback, this);
}

void VelocityProfiler::setVelocityInRange(){

     // Get the delta time
     double delta = (ros::Time::now() - last_time).toSec();

     double current_linear_vel = current_twist.linear.x;
     double current_angular_vel = current_twist.angular.z; // in case

     // Set points
     double set_point_linear_vel = set_point_twist.linear.x;
     double set_point_angular_vel = set_point_twist.angular.z;

     double maxVel2 = velocity_function(global_nearest);

    if( set_point_linear_vel > maxVel2 )
      set_point_linear_vel = maxVel2;

     double minVel = current_linear_vel - acc*delta;
     double maxVel = current_linear_vel + acc*delta;

     current_linear_vel = (set_point_linear_vel < minVel) ? minVel : (set_point_linear_vel > maxVel)? maxVel : set_point_linear_vel;

     // Pack the Payload to be sent soon
     current_twist.linear.x = current_linear_vel;
     current_twist.angular.z = set_point_angular_vel;

}

void VelocityProfiler::publishTwist()
{
      ROS_INFO ("Published : %f ", current_twist.linear.x);
      setpoint_pub_.publish(set_point_twist);
      twist_pub_.publish(current_twist);
}

bool VelocityProfiler::isActivate(){
      return activate;
}

void VelocityProfiler::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

      // Deadman 's Switch
      bool deadman_triggered =  (joy->axes[deadman_] == -1); // -1 For Logitech F710 Series
      bool manual_deadman    =  (joy->buttons[4] == 1); // For Movement without velocity function

      // Nearest Distance
      float nearest = global_nearest;

      // Deadman Triggered Activated
      if(deadman_triggered && !manual_deadman){
          // Set the setpoint
          set_point_twist.linear.x =  l_scale_*joy->axes[linear_]; // 0.4
          set_point_twist.angular.z = 0;
          activate = true;
      }
      else if (deadman_triggered != -1 && !manual_deadman){
          current_twist.linear.x  = 0;                      //twist = * new geometry_msgs::Twist();
          current_twist.angular.z = 0;
          activate = false;                         //Put the Teleop Off
          return;
      }

      // Manual Controller
      if(!deadman_triggered && manual_deadman){
          current_twist.angular.z = a_scale_*joy->axes[angular_];
          current_twist.linear.x  = l_scale_*joy->axes[linear_];
          activate = false;
      }else if(!deadman_triggered && !manual_deadman){
          current_twist.linear.x  = 0;                      //twist = * new geometry_msgs::Twist();
          current_twist.angular.z = 0;
          activate = false;
          return;
      }
}

void VelocityProfiler::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scans)
{

      float local_nearest = 100.0;
      for(int i = startAngle ; i < endAngle ; i++){
          local_nearest = std::min(scans->ranges[i] , local_nearest);
      }

      global_nearest = local_nearest;

}

double VelocityProfiler::velocity_function(float distance)
{
      double slope = 0.4;
      return slope* distance;
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
         // Bound the value ( exceed the acceleration or not ? )
         if(vel_profiler.isActivate())vel_profiler.setVelocityInRange();

         // Publish the message
         vel_profiler.publishTwist();

         // Last Time
         vel_profiler.last_time = ros::Time::now();

         ros::spinOnce();
         loop_rate.sleep();

     }
}
