#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/client/simple_action_client.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <vector>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vel_control_test");
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client("/arm_controller/follow_joint_trajectory", true);
    client.waitForServer();
    control_msgs::FollowJointTrajectoryGoal goal;
    
    
    
    //specify the trajectory
    trajectory_msgs::JointTrajectory traj;
    traj.header.seq = 0;
    //traj.header.stamp = ros::Time::now();
    traj.header.frame_id = "/odom";
    traj.joint_names.push_back("arm_elbow_joint");
    traj.joint_names.push_back("arm_shoulder_lift_joint");
    traj.joint_names.push_back("arm_shoulder_pan_joint");
    traj.joint_names.push_back("arm_wrist_1_joint");
    traj.joint_names.push_back("arm_wrist_2_joint");
    traj.joint_names.push_back("arm_wrist_3_joint");
    double signn = 1;
    //1st point
    trajectory_msgs::JointTrajectoryPoint temp_point;
    temp_point.velocities.push_back(0);
    temp_point.velocities.push_back(0);
    temp_point.velocities.push_back(0);
    temp_point.velocities.push_back(0);
    temp_point.velocities.push_back(0);
    temp_point.velocities.push_back(0);
    temp_point.time_from_start = ros::Duration(0);
    traj.points.push_back(temp_point);
    //2nd point
    temp_point.velocities[4] = signn * 0.1;
    temp_point.time_from_start = ros::Duration(0.2);
    traj.points.push_back(temp_point);
    //3rd point
    temp_point.velocities[4] = signn * 0.2;
    temp_point.time_from_start = ros::Duration(0.4);
    traj.points.push_back(temp_point);
    //4th point
    temp_point.velocities[4] = signn * 0.3;
    temp_point.time_from_start = ros::Duration(0.6);
    traj.points.push_back(temp_point);
    //5th point
    temp_point.velocities[4] = signn * 0.4;
    temp_point.time_from_start = ros::Duration(0.8);
    traj.points.push_back(temp_point);
    //6th point
    temp_point.velocities[4] = signn * 0.5;
    temp_point.time_from_start = ros::Duration(1.0);
    traj.points.push_back(temp_point);
    
    //7th point
    temp_point.velocities[4] = signn * 0.4;
    temp_point.time_from_start = ros::Duration(1.2);
    traj.points.push_back(temp_point);
    //8th point
    temp_point.velocities[4] = signn * 0.3;
    temp_point.time_from_start = ros::Duration(1.4);
    traj.points.push_back(temp_point);
    //9th point
    temp_point.velocities[4] = signn * 0.2;
    temp_point.time_from_start = ros::Duration(1.6);
    traj.points.push_back(temp_point);
    //10th point
    temp_point.velocities[4] = signn * 0.1;
    temp_point.time_from_start = ros::Duration(1.8);
    traj.points.push_back(temp_point);
    //11th point
    temp_point.velocities[4] = 0.0;
    temp_point.time_from_start = ros::Duration(2.0);
    traj.points.push_back(temp_point);
    
    
    goal.trajectory = traj;
    goal.goal_time_tolerance = ros::Duration(0);
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(10));
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        printf("Yay! The goal is executed with no error");
    printf("Cureent State: %s\n", client.getState().toString().c_str());
    return 0;
}
