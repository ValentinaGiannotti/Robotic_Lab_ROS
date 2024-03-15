#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // Your control logic goes here
       ROS_INFO("\nReceived joint positions:");
    for (size_t i = 0; i < msg->position.size(); i++) {
        ROS_INFO("%f", msg->position[i]);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    // Create a subscriber to the joint state topic
    ros::Subscriber joint_state_sub = nh.subscribe("/arm/joint_states", 10, jointStateCallback);
 
  ros::Publisher joint0_pub = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J0_controller/command", 1);
  ros::Publisher joint1_pub = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J1_controller/command", 1);
  ros::Publisher joint2_pub = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J2_controller/command", 1);
  ros::Publisher joint3_pub = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J3_controller/command", 1);



  while (ros::ok())
  {

    std_msgs::Float64 joint0_command;
    joint0_command.data =1.6;
    joint0_pub.publish(joint0_command);

    std_msgs::Float64 joint1_command;
    joint1_command.data = 0.5;
    joint1_pub.publish(joint1_command);

    std_msgs::Float64 joint2_command;
    joint2_command.data = -1.2;
    joint2_pub.publish(joint2_command);

    std_msgs::Float64 joint3_command;
    joint3_command.data = -1;
    joint3_pub.publish(joint3_command);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

 

