/******************************
 *  Map Semantic Translater
 *  provides Semantic meaning translation to ROS Goal Message
 *  as a services
 *  Author : Theppasith Nisitsukcharoen
 *  29-Sept-2016
 *******************************/
 #include <ros/ros.h>
 #include <ros/package.h>
 #include <std_srvs/Empty.h>
 #include <nav_msgs/Odometry.h>
 #include <geometry_msgs/Pose2D.h>
 #include <tf/transform_listener.h>
 #include <actionlib/client/simple_action_client.h>
 #include <move_base_msgs/MoveBaseAction.h>

 int main(int argc, char **argv)
 {

   ros::init(argc, argv, "talker");

   ros::NodeHandle n;

   ros::Publisher chatter_pub = n.advertise<move_base_msgs::MoveBaseGoal>
                                ("/kamtoa/goal", 1000);

   move_base_msgs::MoveBaseGoal msg;

   ros::Rate loop_rate(0.5);

   /**
    * A count of how many messages we have sent. This is used to create
    * a unique string for each message.
    */
   int count = 0;
   while (ros::ok())
   {
     msg.target_pose.header.frame_id = "/map";
     msg.target_pose.pose.position.x    = -5;
     msg.target_pose.pose.position.y    = -6;
     tf::Quaternion q = tf::createQuaternionFromRPY( 0 ,0, 0);
     msg.target_pose.pose.orientation.x = q.x();
     msg.target_pose.pose.orientation.y = q.y();
     msg.target_pose.pose.orientation.z = q.z();
     msg.target_pose.pose.orientation.w = q.w();

     std::cout << count  << std::endl;
     chatter_pub.publish(msg);

     ros::spinOnce();

     loop_rate.sleep();
     ++count;
   }

   return 0;
 }
