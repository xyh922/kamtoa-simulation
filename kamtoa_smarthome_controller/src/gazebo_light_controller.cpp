/******************************
 *  ROS Node to send gazebo light message
 *  Author : Theppasith Nisitsukcharoen
 *  26-Jul-2016
 *******************************/
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/msgs/light.pb.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

class LightController
{
    public:
    LightController();

    private:
    void light_msg_callback(const geometry_msgs::Twist::ConstPtr& msg);

    ros::NodeHandle                 nh_;
    ros::Publisher                  light_response_pub;
    ros::Subscriber                 light_sub;
    gazebo::transport::PublisherPtr gazebo_pub;

};

LightController::LightController()
{

        //ROS Publisher
        light_response_pub  = nh_.advertise<geometry_msgs::Twist>("smarthome/response",1);
        //ROS Subscriber
        light_sub           = nh_.subscribe<geometry_msgs::Twist>
                                      ("smarthome/light",
                                      10,
                                      &LightController::light_msg_callback,
                                      this);

        //Gazebo
        // Create our node for communication
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();
        // Publish to a Gazebo topic
        gazebo_pub = node->Advertise<gazebo::msgs::Light>("~/light/modify");
        // Wait for a subscriber to connect
        gazebo_pub->WaitForConnection();

}


void LightController::light_msg_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
        //Create light Message
        gazebo::msgs::Light lightMsg;
        lightMsg.set_name("spot");
        lightMsg.set_range(50);

        gazebo_pub->Publish(lightMsg);
}


int main(int argc, char **argv)
{
        ros::init(argc,argv,"gazebo_smarthome_light_controller");
        // Load gazebo
        gazebo::client::setup(argc, argv);
        ROS_INFO("[Smarthome-Sim] Light Controller Initialized ! ");
        ros::spin();
        // Make sure to shut everything down.
        gazebo::client::shutdown();
}
