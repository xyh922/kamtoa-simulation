#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <actionlib_msgs/GoalID.h>
#include <iostream>

/*************************************************************************************/
class KamtoaJoystick
{
    public:
    KamtoaJoystick();

    private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle   nh_;
    ros::Publisher    twist_pub_,auto_stop_pub_;
    ros::Subscriber   joy_sub_;

    std::string            twist_pub_topic_name_,joystick_sub_topic_name_;
    bool              stop_state;
    int               linear_   , angular_ , deadman_ , goalCancel_;
    double            l_scale_  , a_scale_;
    bool              off_teleop;
};
/*************************************************************************************/

KamtoaJoystick::KamtoaJoystick() :
        linear_(1),
        angular_(2),
        deadman_(3),
        goalCancel_(7)
{
    twist_pub_topic_name_       = "cmd_vel";
    joystick_sub_topic_name_    = "joy";

    nh_.param("axis_linear",    linear_,  linear_   );
    nh_.param("button_deadman_switch",    deadman_,  deadman_   );
    nh_.param("axis_angular",   angular_, angular_  );
    nh_.param("scale_angular",  a_scale_, a_scale_  );
    nh_.param("scale_linear",   l_scale_, l_scale_  );
    nh_.param("twist_pub_topic", twist_pub_topic_name_ , twist_pub_topic_name_);
    nh_.param("goal_cancel_button", goalCancel_,goalCancel_);
    nh_.param("joystick_sub_topic", joystick_sub_topic_name_);

    // Teleop Boolean Switch
    off_teleop  = false;

    //Publisher and Subscriber
    twist_pub_  = nh_.advertise<geometry_msgs::Twist>(twist_pub_topic_name_, 1);
    joy_sub_    = nh_.subscribe<sensor_msgs::Joy>(joystick_sub_topic_name_, 10, &KamtoaJoystick::joyCallback, this);

    //Navigation Stopper
    auto_stop_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
}

void KamtoaJoystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //Goal Nav Cancel Button
    int goal_cancel_button;

    //Geometry Joystick Control
    geometry_msgs::Twist twist;

    int deadman_triggered;

    //Read Velocities Value From Joystick
    twist.angular.z = a_scale_*joy->axes[angular_];
    twist.linear.x = l_scale_*joy->axes[linear_];

    deadman_triggered = joy->axes[deadman_];
    goal_cancel_button = joy->buttons[goalCancel_];

    if(goalCancel_ == 1){
        twist_pub_.publish(*new geometry_msgs::Twist());        //Publish 0,0,0 (stop)
        auto_stop_pub_.publish(*new actionlib_msgs::GoalID());  //Publish Goal Cancel Message
        off_teleop = true;
    }

    if(deadman_triggered == -1) //Deadman Triggered Activated
    {
        off_teleop = false;
        twist_pub_.publish(twist);
    }
    else if (deadman_triggered != -1 && !off_teleop)
    {
        twist_pub_.publish(*new geometry_msgs::Twist());        //Publish 0,0,0 (stop)
        auto_stop_pub_.publish(*new actionlib_msgs::GoalID());  //Publish Goal Cancel Message
        off_teleop = true;                                      //Put the Teleop Off
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Kamtoa_Joystick_Teleop_Node");
    KamtoaJoystick teleop_dumbo;
    ros::spin();
}
