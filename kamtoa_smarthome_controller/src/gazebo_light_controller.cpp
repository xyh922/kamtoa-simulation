/******************************
 *  ROS Node
 *  to send gazebo light message
 *  and provide server service for light controlling services
 *  Author : Theppasith Nisitsukcharoen
 *  26-Jul-2016
 *******************************/
// Gazebo Headers
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/msgs/light.pb.h>
// ROS Headers
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
// Custom messages
#include "kamtoa_smarthome_controller/smarthome_action.h"
#include "kamtoa_smarthome_controller/smarthome_response.h"

class LightController
{
    public:
    LightController();
    ~LightController();

    private:
    void light_msg_callback(const kamtoa_smarthome_controller::smarthome_action::ConstPtr& msg);

    ros::NodeHandle                 nh_;
    ros::Publisher                  light_response_pub;
    ros::Subscriber                 light_sub;
    gazebo::transport::PublisherPtr gazebo_pub;

};

LightController::LightController()
{
    // ROS Publisher
    light_response_pub  = nh_.advertise<kamtoa_smarthome_controller::smarthome_response>
                                  ("smarthome/light/response",
                                  1);
    // ROS Subscriber
    light_sub           = nh_.subscribe<kamtoa_smarthome_controller::smarthome_action>
                                  ("smarthome/light/action",
                                  10,
                                  &LightController::light_msg_callback,
                                  this);
    // Gazebo
    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    // Publish to a Gazebo topic
    gazebo_pub = node->Advertise<gazebo::msgs::Light>("~/light/modify");
    // Wait for a subscriber to connect
    gazebo_pub->WaitForConnection();

}

LightController::~LightController()
{
    std::cout << "Destructor Activated" << std::endl;
}


void LightController::light_msg_callback(const kamtoa_smarthome_controller::smarthome_action::ConstPtr& msg)
{
    /*
      Example CLI Command : 
      rostopic pub /smarthome/light/action kamtoa_smarthome_controller/smarthome_action '{target: "spot", action: 1}'
    */

    std::cout << "Received Message" << std::endl;
    std::cout << "Target : " << msg->target <<std::endl;
    std::cout << "Action : " << msg->action <<std::endl;

    //Create light Message and publish to gazebo server
    gazebo::msgs::Light lightMsg;
    lightMsg.set_name(msg->target);
    switch (msg->action) {
      case 1:
          lightMsg.set_range(50);
          gazebo_pub->Publish(lightMsg);
          break;
      case 0:
          lightMsg.set_range(0);
          gazebo_pub->Publish(lightMsg);
          break;
      default:
          // Do nothing;
          break;
    }
}


int main(int argc, char **argv)
{
        ros::init(argc,argv,"gazebo_smarthome_light_controller");
        // Load gazebo
        gazebo::client::setup(argc, argv);
        ROS_INFO("[Smarthome-Sim] Light Controller Initialized ! ");
        LightController lightController;

        ROS_INFO("[Smarthome-Sim] Light Controller Instance Create Successful ! ");
        while(ros::ok()){
            ros::spin();
        }
        // Make sure to shut everything down.
        gazebo::client::shutdown();
        return 0;
}
