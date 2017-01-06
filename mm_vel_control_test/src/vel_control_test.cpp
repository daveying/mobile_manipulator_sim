#include "ros/ros.h"
#include "std_msgs/Float64.h"



int main(int argc, char** argv)
{
    ros::init(argc, argv, "vel_control_test");

    ros::NodeHandle n;
    ros::Publisher pan_joint_pub = n.advertise<std_msgs::Float64>("/arm_shoulder_pan_joint_vel_controller/command", 1000);
    ros::Publisher lift_joint_pub = n.advertise<std_msgs::Float64>("/arm_shoulder_lift_joint_vel_controller/command", 1000);
    ros::Publisher elbow_joint_pub = n.advertise<std_msgs::Float64>("/arm_elbow_joint_vel_controller/command", 1000);
    ros::Publisher wrist1_joint_pub = n.advertise<std_msgs::Float64>("/arm_wrist_1_joint_vel_controller/command", 1000);
    ros::Publisher wrist2_joint_pub = n.advertise<std_msgs::Float64>("/arm_wrist_2_joint_vel_controller/command", 1000);
    ros::Publisher wrist3_joint_pub = n.advertise<std_msgs::Float64>("/arm_wrist_3_joint_vel_controller/command", 1000);
    ros::Rate loop_rate(100);
    int count = -0;
    while(ros::ok())
    {
        std_msgs::Float64 msg;
        msg.data = 0.02 * count;
        pan_joint_pub.publish(msg);
        lift_joint_pub.publish(msg);
        elbow_joint_pub.publish(msg);
        wrist1_joint_pub.publish(msg);
        wrist2_joint_pub.publish(msg);
        wrist3_joint_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        count--;
    }    
    
    
    return 0;
}
