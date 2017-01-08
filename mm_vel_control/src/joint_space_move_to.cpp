#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <Eigen/Dense>

ros::Subscriber sub_joint_states;

ros::Publisher pub_pan;
ros::Publisher pub_lift;
ros::Publisher pub_elbow;
ros::Publisher pub_wrist1;
ros::Publisher pub_wrist2;
ros::Publisher pub_wrist3;

bool joint_data_come = false;
Eigen::VectorXd joint_values(6);
Eigen::MatrixXd goal_values(6, 3);
Eigen::VectorXd goal_value(6);

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);


int main(int argc, char **argv)
{
    goal_values << 0, 1.48, 1.48, 
                   0, -0.9, -0.9, 
                   0, 0, 1.57, 
                   0, -0.57, -0.57, 
                   0, 1.57, 1.57, 
                   0, 0, 0;
    for(int i = 0; i < 6; ++i)
        goal_value(i) = goal_values(i, 0);
    std::cout << argv[1][0] << std::endl;
    if (argv[1][0] == '1')
    {
        ROS_INFO("1");
        for(int i = 0; i < 6; ++i)
            goal_value(i) = goal_values(i, 1);
    }
    if (argv[1][0] == '2')
    {
        ROS_INFO("2");
        for(int i = 0; i < 6; ++i)
            goal_value(i) = goal_values(i, 2);
    }
    
    ros::init(argc, argv, "joint_space_move_to");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    
    sub_joint_states = n.subscribe("/joint_states", 1000, jointStateCallback);
    
    pub_pan = n.advertise<std_msgs::Float64>("/arm_shoulder_pan_joint_vel_controller/command", 1000);
    pub_lift = n.advertise<std_msgs::Float64>("/arm_shoulder_lift_joint_vel_controller/command", 1000);
    pub_elbow = n.advertise<std_msgs::Float64>("/arm_elbow_joint_vel_controller/command", 1000);
    pub_wrist1 = n.advertise<std_msgs::Float64>("/arm_wrist_1_joint_vel_controller/command", 1000);
    pub_wrist2 = n.advertise<std_msgs::Float64>("/arm_wrist_2_joint_vel_controller/command", 1000);
    pub_wrist3 = n.advertise<std_msgs::Float64>("/arm_wrist_3_joint_vel_controller/command", 1000);
    
    Eigen::VectorXd vel(6);
    double Kp = 3;
    int count = 0;
    while(ros::ok())
    {
        if(joint_data_come)
        {
            joint_data_come = false;
            vel = Kp * (goal_value - joint_values);
            std_msgs::Float64 temp_msg;
            temp_msg.data = vel(0);
            pub_elbow.publish(temp_msg);
            temp_msg.data = vel(1);
            pub_lift.publish(temp_msg);
            temp_msg.data = vel(2);
            pub_pan.publish(temp_msg);
            temp_msg.data = vel(3);
            pub_wrist1.publish(temp_msg);
            temp_msg.data = vel(4);
            pub_wrist2.publish(temp_msg);
            temp_msg.data = vel(5);
            pub_wrist3.publish(temp_msg);
        }
        if(vel.norm() < 0.001)
        {
            count++;
            ROS_INFO("%d", count);
        }
        if(count > 200)
            break;
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/*callback functions*/
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	if(msg->position.size() == 6)
	{
		//ROS_INFO("position: %lf, %lf, %lf, %lf, %lf, %lf", msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5]);
		joint_values(0) = msg->position[0];
		joint_values(1) = msg->position[1];
		joint_values(2) = msg->position[2];
		joint_values(3) = msg->position[3];
		joint_values(4) = msg->position[4];
		joint_values(5) = msg->position[5];
		joint_data_come = true;
	}
}

