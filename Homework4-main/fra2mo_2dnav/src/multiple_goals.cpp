#include <ros/ros.h>
#include "../include/tf_nav.h"
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// the order of the goal is decided here
std::vector<std::string> traj = {"goal6","goal3", "goal4", "goal2", "goal1", "goal5"}; 
//std::vector<std::string> traj = {"goal3", "goal4", "goal2", "goal1"}; 
const float toRadians = M_PI/180.0;
 
int main(int argc, char** argv){
  ros::init(argc, argv, "multiple_navigation_goals");
 
  tf::TransformListener listener;
  tf::StampedTransform transform;
  ros::Rate r(1);

  move_base_msgs::MoveBaseGoal goal;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  for(int i = 0; i < traj.size(); i++){
    try{
      listener.waitForTransform( "map", traj[i], ros::Time(0), ros::Duration(10.0));
      listener.lookupTransform( "map", traj[i], ros::Time(0), transform);
      
      // print current position and orientation
      ROS_INFO("Current fra2mo position:\t\t %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
      ROS_INFO("Current fra2mo orientation (quaterinion) [x,y,z,w]:\t %f, %f, %f, %f", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
    }
    catch( tf::TransformException &ex ) {
      ROS_ERROR("%s", ex.what());
      r.sleep();
      continue;
    }

    // upload the goal
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = transform.getOrigin().x();
    goal.target_pose.pose.position.y = transform.getOrigin().y();
    goal.target_pose.pose.orientation.x = transform.getRotation().x();
    goal.target_pose.pose.orientation.y = transform.getRotation().y();
    goal.target_pose.pose.orientation.z = transform.getRotation().z();
    goal.target_pose.pose.orientation.w = transform.getRotation().w();

    // send the goal and waiting for the result
    ROS_INFO("Sending %i goal", i+1);
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Hooray, the base moved\n");
      ros::Duration(2.0).sleep();
    }
    else{
      ROS_INFO("The base failed to move for some reason");
      i = 5;
    }
  }
   return 0;
 }