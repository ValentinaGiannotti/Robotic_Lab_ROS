#include <ros/ros.h>
#include "std_msgs/Float64.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "iiwa_controller_publisher_node");
    ros::NodeHandle nh;
   // Crea publisher per inviare comandi ai controller
    ros::Publisher joint1CommandPub = nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J1_controller/command", 100);
    ros::Publisher joint2CommandPub = nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J2_controller/command", 100);
    ros::Publisher joint3CommandPub = nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J3_controller/command", 100); 
    ros::Publisher joint4CommandPub = nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J4_controller/command", 100);
    ros::Publisher joint5CommandPub = nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J5_controller/command", 100);
    ros::Publisher joint6CommandPub = nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J6_controller/command", 100);
    ros::Publisher joint7CommandPub = nh.advertise<std_msgs::Float64>("/iiwa/PositionJointInterface_J7_controller/command", 100);


    // Rate
    ros::Rate loop_rate(500);

    while (ros::ok())
    {
        std_msgs::Float64 position_command_j1;
        position_command_j1.data = 0.0 ;
        // Pubblica il comando sul topic del controller del giunto
        joint1CommandPub.publish(position_command_j1);
         
        // Invio messaggi distinti anche se il contenuto è il medesimo per avere generalità
        std_msgs::Float64 position_command_j2;
        position_command_j2.data = 1.57;
        joint2CommandPub.publish(position_command_j2);

        std_msgs::Float64 position_command_j3;
        position_command_j3.data = -1.57;
        joint3CommandPub.publish(position_command_j3);

        std_msgs::Float64 position_command_j4;
        position_command_j4.data = -1.57;
        joint4CommandPub.publish(position_command_j4);       

        std_msgs::Float64 position_command_j5;
        position_command_j5.data = 1.57;
        // Pubblica il comando sul topic del controller del giunto
        joint5CommandPub.publish(position_command_j5);
         
        // Invio messaggi distinti anche se il contenuto è il medesimo per avere generalità
        std_msgs::Float64 position_command_j6;
        position_command_j6.data = -1.57;
        joint6CommandPub.publish(position_command_j6);

        std_msgs::Float64 position_command_j7;
        position_command_j7.data = 1.57;
        joint7CommandPub.publish(position_command_j7);


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}