/******************************
*  ROS Node to send gazebo custom message
*  Author : Theppasith Nisitsukcharoen
*  22-Jul-2016
*******************************/
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/msgs/light.pb.h>
//#include <ros/ros.h>
#include <iostream>

/////////////////////////////////////////////////
int main(int argc, char **argv)
{

        //ros::init(argc,argv,"Light_GAZEBO_INVOKER_NODE");
        //ros::NodeHandle nh;

        // Load gazebo
        gazebo::client::setup(argc, argv);
        // Create our node for communication
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

        std::cout << "init complete" <<std::endl;

        // Publish to a Gazebo topic
        gazebo::transport::PublisherPtr pub =
                node->Advertise<gazebo::msgs::Light>("~/light/modify");

        // Wait for a subscriber to connect
        pub->WaitForConnection();

        bool trigger = false;
        // Publisher loop...replace with your own code.
        while (true)
        {
                std::cout << "Looping" <<std::endl;
                // Throttle Publication
                gazebo::common::Time::MSleep(500);
                gazebo::msgs::Light lightMsg;
                lightMsg.set_name("spot");
                trigger = !trigger;
                if(trigger){
                lightMsg.set_range(50);
              }else{
                lightMsg.set_range(0);
              }
                pub->Publish(lightMsg);

                //ros::spin();
        }

        // Make sure to shut everything down.
        gazebo::client::shutdown();
}


































//////////////////////////////////////////////////////////////////////////////////////
/*#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/light.pb.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class WorldPluginTutorial : public WorldPlugin
  {
    transport::NodePtr node;
    transport::PublisherPtr LightPub;
    physics::WorldPtr world;

    public: WorldPluginTutorial() : WorldPlugin()
            {
              std::cout << "Hello World" << std::endl;
            }

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
            {
              std::cout << "On Load !!!" << std::endl;
              this->node = transport::NodePtr(new transport::Node());
              world = _parent;

              // Initialize the node with the world name
              node->Init(world->GetName());

              std::cout << "Publish to: " << "~/light/modify" << std::endl;

              // Create a publisher on the ~/gazebo/default/light/modify topic
              LightPub = node->Advertise<msgs::Light>("~/light/modify");

               msgs::Light lightMsg;
               lightMsg.set_name("point");
               lightMsg.set_range(10);
               LightPub->Publish(lightMsg);
               //lightMsg.range (10);//= 10;

            }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}

int main (int argc, char** argv)
{
  //ROS Node Initializing
  ros::init(argc,argv,"Light_GAZEBO_INVOKER_NODE");
  ros::NodeHandle nh;

  gazebo::WorldPluginTutorial world_plugin;

  return 0;
}
*////////////////////////////////////////////////////


// #include <iostream>
// #include <sstream>
//
// #include "gazebo/transport/Node.hh"
// #include "gazebo/gui/GuiEvents.hh"
// #include "gazebo/common/MouseEvent.hh"
// #include "gazebo/rendering/UserCamera.hh"
// #include "gazebo/rendering/Light.hh"
// #include "gazebo/rendering/Scene.hh"
//
// #include "gazebo/gui/LightMaker.hh"
//
// using namespace gazebo;
// using namespace gui;
//
// unsigned int LightMaker::counter = 0;
//
// /////////////////////////////////////////////////
// LightMaker::LightMaker() : EntityMaker()
// {
//   this->lightPub = this->node->Advertise<msgs::Light>("~/light");
//
//   this->state = 0;
//
//   msgs::Set(this->msg.mutable_diffuse(), common::Color(0.5, 0.5, 0.5, 1));
//   msgs::Set(this->msg.mutable_specular(), common::Color(0.1, 0.1, 0.1, 1));
//
//   this->msg.set_attenuation_constant(0.5);
//   this->msg.set_attenuation_linear(0.01);
//   this->msg.set_attenuation_quadratic(0.001);
//   this->msg.set_range(20);
// }
//
// /////////////////////////////////////////////////
// void LightMaker::Start(const rendering::UserCameraPtr _camera)
// {
//   this->camera = _camera;
//
//   this->light = new rendering::Light(this->camera->GetScene());
//   this->light->Load();
//
//   this->light->SetLightType(this->lightTypename);
//   this->light->SetPosition(math::Vector3(0, 0, 1));
//   if (this->lightTypename == "directional")
//     this->light->SetDirection(math::Vector3(.1, .1, -0.9));
//
//   std::ostringstream stream;
//   stream << "user_" << this->lightTypename << "_light_" << counter++;
//   this->msg.set_name(stream.str());
//   this->state = 1;
// }
//
// /////////////////////////////////////////////////
// void LightMaker::Stop()
// {
//   delete this->light;
//   this->light = NULL;
//
//   this->state = 0;
//   gui::Events::moveMode(true);
// }
//
// /////////////////////////////////////////////////
// bool LightMaker::IsActive() const
// {
//   return this->state > 0;
// }
//
// /////////////////////////////////////////////////
// void LightMaker::OnMousePush(const common::MouseEvent &/*_event*/)
// {
// }
//
// /////////////////////////////////////////////////
// void LightMaker::CreateTheEntity()
// {
//   msgs::Set(this->msg.mutable_pose()->mutable_position(),
//             this->light->GetPosition());
//   msgs::Set(this->msg.mutable_pose()->mutable_orientation(),
//             math::Quaternion());
//   this->lightPub->Publish(this->msg);
//   this->camera.reset();
// }
//
// /////////////////////////////////////////////////
// void LightMaker::OnMouseRelease(const common::MouseEvent &_event)
// {
//   if (_event.button == common::MouseEvent::LEFT && !_event.dragging)
//   {
//     this->CreateTheEntity();
//     this->Stop();
//   }
// }
//
// /////////////////////////////////////////////////
// // \TODO: This was copied from ModelMaker. Figure out a better way to
// // prevent code duplication.
// void LightMaker::OnMouseMove(const common::MouseEvent &_event)
// {
//   math::Vector3 pos = this->light->GetPosition();
//
//   math::Vector3 origin1, dir1, p1;
//   math::Vector3 origin2, dir2, p2;
//
//   // Cast two rays from the camera into the world
//   this->camera->GetCameraToViewportRay(_event.pos.x, _event.pos.y,
//                                        origin1, dir1);
//
//   // Compute the distance from the camera to plane of translation
//   math::Plane plane(math::Vector3(0, 0, 1), 0);
//
//   double dist1 = plane.Distance(origin1, dir1);
//
//   // Compute two points on the plane. The first point is the current
//   // mouse position, the second is the previous mouse position
//   p1 = origin1 + dir1 * dist1;
//   pos = p1;
//
//   if (!_event.shift)
//   {
//     if (ceil(pos.x) - pos.x <= .4)
//       pos.x = ceil(pos.x);
//     else if (pos.x - floor(pos.x) <= .4)
//       pos.x = floor(pos.x);
//
//     if (ceil(pos.y) - pos.y <= .4)
//       pos.y = ceil(pos.y);
//     else if (pos.y - floor(pos.y) <= .4)
//       pos.y = floor(pos.y);
//   }
//   pos.z = this->light->GetPosition().z;
//
//   this->light->SetPosition(pos);
// }
