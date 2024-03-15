#include <ros/ros.h>
#include "../include/tf_nav.h"
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
const float toRadians = M_PI/180.0;

std::vector<std::vector<double>> goals(4);
std::vector<double> fra2mo_pose(7,0.0), aruco_pose(7,0.0);
bool aruco_pose_available = false, find_des_pose = false, task_ended = false;

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "aruco_search");
  ros::NodeHandle nh;
  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;
  tf::StampedTransform base_footprint_tf, aruco_pose_tf;

  // Rate
  ros::Rate loop_rate(10);

  // Subscribers to node
  ros::Subscriber aruco_pose_sub = nh.subscribe("/aruco_single/pose", 1, arucoPoseCallback);
  ros::Publisher aruco_pose_pub = nh.advertise<geometry_msgs::PoseStamped>( "/aruco/pose", 1);

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  // define base variable for client-server comunication
  move_base_msgs::MoveBaseGoal goal;
  Eigen::Vector3d des_pose;

  // define the trajectory to make the robot go near by the obstacle 9
  tf2::Quaternion traj_orient, oriz_orient;
  traj_orient.setRPY( 0, 0, 180.0*toRadians);
  Eigen::Vector3d P_1 = {-6, 8.0, 1};
  Eigen::Vector3d P_2 = {-11.0, 4.0, 1};
  Eigen::Vector3d P_3 = {-13.5, 8.0, 1};
  std::vector<Eigen::Vector3d> traj = {P_1, P_2, P_3}; 

  // execute the trajectory 
  for (int i=0; i<3; i++){

    // upload the goal
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = traj[i][0];
    goal.target_pose.pose.position.y = traj[i][1];

    goal.target_pose.pose.orientation.x = traj_orient[0];
    goal.target_pose.pose.orientation.y = traj_orient[1];
    goal.target_pose.pose.orientation.z = traj_orient[2];
    goal.target_pose.pose.orientation.w = traj_orient[3];

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
    }
  }

  // variable for the research
  int j=1;
  Eigen::Vector3d p_map_to_object;
  Eigen::Matrix3d rot_map_to_object; 

  while(ros::ok()){
    
    if(aruco_pose_available){

      //// compute the position of the marker in the map frame ////

      // position and orientation of the object in the camera frame
      Eigen::Vector3d p_cam_to_object  = {aruco_pose[0], aruco_pose[1], aruco_pose[2]};
      Eigen::Quaterniond quaternion_cam(aruco_pose[6], aruco_pose[3], aruco_pose[4], aruco_pose[5]);
      Eigen::Matrix3d rot_cam_to_object = quaternion_cam.toRotationMatrix();

      // position and orientation of the camera in the footprint frame
      Eigen::Vector3d p_base_to_cam = {0.0975, 0.0, 0.065};
      Eigen::Matrix3d rot_base_to_cam =
        (Eigen::AngleAxisd(-90.0 * toRadians, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(-90.0 * toRadians, Eigen::Vector3d::UnitX())).toRotationMatrix();


      // position and orientation of the footprint in the map frame
      try{
        listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform( "map", "base_footprint", ros::Time(0), base_footprint_tf);
      }
      catch( tf::TransformException &ex ) {
        ROS_ERROR("%s", ex.what());
        loop_rate.sleep();
        continue;
      }
      Eigen::Vector3d p_map_to_base = {base_footprint_tf.getOrigin().x(), base_footprint_tf.getOrigin().y(), base_footprint_tf.getOrigin().z()};
      Eigen::Quaterniond quaternion_base(base_footprint_tf.getRotation().w(), base_footprint_tf.getRotation().x(), base_footprint_tf.getRotation().y(), base_footprint_tf.getRotation().z());
      Eigen::Matrix3d rot_map_to_base = quaternion_base.toRotationMatrix();

      // final computation
      Eigen::Vector3d p_base_to_object = p_base_to_cam + rot_base_to_cam*p_cam_to_object;
      p_map_to_object = p_map_to_base + rot_map_to_base*p_base_to_object;
      rot_map_to_object = rot_map_to_base*rot_base_to_cam*rot_cam_to_object;

      Eigen::Vector3d offset = {1.0, 0.0, 0.0};
      des_pose = p_map_to_object + offset;
      find_des_pose = true;
       

      //--- DEBUG ---//
        // std::cout << "p_cam_to_object: " << p_cam_to_object << std::endl;
        // std::cout << "rot_cam_to_object:\n" << rot_cam_to_object << std::endl;

        // std::cout << "p_base_to_object: " << p_base_to_object << std::endl;
        // std::cout << "rot_base_to_object:\n" << rot_base_to_object << std::endl;

        // std::cout << "p_map_to_object:\n" << p_map_to_object << std::endl;
        // std::cout << "rot_map_to_object:\n" << rot_map_to_object << std::endl;

        // std::cout << "des_pose:\n" << des_pose << std::endl;
      //-------------//

    } 

    if(find_des_pose && !task_ended){
      
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = des_pose[0];
      goal.target_pose.pose.position.y = des_pose[1];

      goal.target_pose.pose.orientation.x = traj_orient[0];
      goal.target_pose.pose.orientation.y = traj_orient[1];
      goal.target_pose.pose.orientation.z = traj_orient[2];
      goal.target_pose.pose.orientation.w = traj_orient[3];

      ROS_INFO("Sending final pose");
      ac.sendGoal(goal);
    
      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base reached the desired position");
        ros::Duration(2.0).sleep();
      }
      else{
        ROS_INFO("The base failed to move for some reason");
      }
      
      task_ended = true;
    }
    
    // Homework 4.c tf for the aruco pose 
    aruco_pose_tf.stamp_ = ros::Time::now();
    aruco_pose_tf.frame_id_ = "map";
    aruco_pose_tf.child_frame_id_ = "aruco_pose";
    Eigen::Quaterniond quat_map_to_object(rot_map_to_object);
    tf::Quaternion quat_map_to_object_tf(quat_map_to_object.x(), quat_map_to_object.y(), quat_map_to_object.z(), quat_map_to_object.w());
    
    aruco_pose_tf.setOrigin({p_map_to_object[0], p_map_to_object[1], p_map_to_object[2]});
    aruco_pose_tf.setRotation(quat_map_to_object_tf);
    
    broadcaster.sendTransform(aruco_pose_tf);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}