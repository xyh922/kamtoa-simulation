/******************************
 *  [Header]
 *  Point Transporter and Navigator
 *  Receive Custom Goal Message from Upper layer
 *  and traslate to "goalMessage"
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

// MoveBase Actionlib typedef
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PointTransporter
{
    public:
    PointTransporter();

    private:
    ros::NodeHandle   nh_;            // Nodehandle
    ros::Subscriber   upperLevelGoal; // Subscriber to UpperLevel
    std::string       upperLevelTopic;// Upper level topic name
    std::string       navGoalAction;  // move_base goal topic
    move_base_msgs::MoveBaseGoal  marked_current_pos;
    move_base_msgs::MoveBaseGoal  marked_goal;

    // On Received Goal from Upper Level
    void onReceiveGoalCallback(
      const move_base_msgs::MoveBaseGoalConstPtr  &receivedGoal);

    // Feedback while traversing
    void goalFeedbackCallback(
      const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

    // Feedback while goal is active
    void goalDoneCallback(
      const actionlib::SimpleClientGoalState &state,
      const move_base_msgs::MoveBaseResultConstPtr &result);

    // Feedback on Goal is finished.
    void goalActiveCallback();

};



// Class Constructor
PointTransporter::PointTransporter(){
    // Default values
    upperLevelTopic   = "/kamtoa/goal";
    navGoalAction     = "move_base";

    // Get parameters from parameter server
    nh_.param("upper_level_topic",upperLevelTopic,  upperLevelTopic   );
    nh_.param("nav_goal_action"  ,navGoalAction  ,  navGoalAction   );

}

void PointTransporter::onReceiveGoalCallback(
    const move_base_msgs::MoveBaseGoalConstPtr  &receivedGoal)
{
    // On received goal from upper layer

}

void PointTransporter::goalFeedbackCallback(
  const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
    // Goal is active and send feedback
}

void PointTransporter::goalDoneCallback(
  const actionlib::SimpleClientGoalState &state,
  const move_base_msgs::MoveBaseResultConstPtr &result)
{
    // Goal is done (error , success)

}

void PointTransporter::goalActiveCallback(){
    // Goal is Activing
}
