#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <keyboard/Key.h>
#define PI 3.1415926

std::vector<double> mouse_pose;
std::vector<double> mouse_init_pose;
std::vector<double> mouse_diff_pose;
bool mouse_data_come = false;
bool mouse_pose_first = true;

ros::Subscriber sub_mouse_pose;
ros::Subscriber sub_key_orientation;

double roll = 0;
double pitch = 0;
double yaw = 0;

void mousePoseCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void keyOrientationCallback(const keyboard::Key::ConstPtr& msg);

int main(int argc, char** argv)
{
	mouse_pose.resize(3);
	mouse_init_pose.resize(3);
	mouse_diff_pose.resize(3);
	ROS_INFO("%lf", mouse_diff_pose[2]);
	ros::init(argc, argv, "goal_tf_broadcaster");
	ros::NodeHandle n;
	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	tf::Transform transform;
	tf::Transform init_transform;
	tf::StampedTransform stransform;
	sub_mouse_pose = n.subscribe("/mouse_speeds", 1000, mousePoseCallback);
	sub_key_orientation = n.subscribe("/keyboard/keydown", 100, keyOrientationCallback);
	bool s = false;
	while(!s && ros::ok())
	{
		try
		{
			listener.lookupTransform("/odom", "/arm_tool0", ros::Time(0), stransform);
			s = true;
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1).sleep();
			continue; 
		}
	}
	
	tf::Vector3 init_position = stransform.getOrigin();
	ROS_INFO("%lf,%lf,%lf",init_position[0],init_position[1],init_position[2]);
	ros::Rate rate(10);
	tf::Transform rot = stransform.inverse();
	rot.setOrigin(tf::Vector3(0,0,0));
	while(ros::ok())
	{
		//publish init transform
		init_transform.setOrigin(init_position);
		init_transform.setRotation(stransform.getRotation());
		br.sendTransform(tf::StampedTransform(init_transform, ros::Time::now(), "/odom", "init_pose"));
		//end publish init transform
		transform.setOrigin(rot * (tf::Vector3(mouse_diff_pose[0],mouse_diff_pose[1],mouse_diff_pose[2]))*0.5);//0.5 is the motion scalar
		tf::Quaternion orien;
		orien.setRPY(roll, pitch, yaw);
		transform.setRotation(orien);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "init_pose", "goal"));
		rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;
}


void mousePoseCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	if(mouse_pose_first)
	{
		mouse_pose_first = false;
		mouse_init_pose[0] = msg->twist.linear.x;
		mouse_init_pose[1] = msg->twist.linear.y;
		mouse_init_pose[2] = 0;
	}
	else
	{
		mouse_pose[0] = msg->twist.linear.x;
		mouse_pose[1] = msg->twist.linear.y;
		mouse_pose[2] = 0;
		mouse_diff_pose[0] = mouse_pose[0] - mouse_init_pose[0];
		mouse_diff_pose[1] = mouse_pose[1] - mouse_init_pose[1];
		//mouse_diff_pose[2] = 0;
		mouse_data_come = true;
	}
}
void keyOrientationCallback(const keyboard::Key::ConstPtr& msg)
{
	//273->arrow up  		274->arrow down  =>roll
	//275->arrow right  	276->arrow left  =>pitch
	//280->page up  		281->page down   =>yaw
	//119->w				115->s           =>z
	//32->space								 =>reset angles to zero
	ROS_INFO("key code: %d", msg->code);
	if(msg->code == 273)
	{	
		roll += 0.1*PI;
		ROS_INFO("arrow up down, roll +0.1pi");
		ROS_INFO("[roll pitch yaw]: [%lf %lf %lf]", roll, pitch, yaw);
	}
	if(msg->code == 274)
	{	
		roll -= 0.1*PI;
		ROS_INFO("arrow down down, roll -0.1pi");
		ROS_INFO("[roll pitch yaw]: [%lf %lf %lf]", roll, pitch, yaw);
	}
	if(msg->code == 275)
	{	
		pitch += 0.1*PI;
		ROS_INFO("arrow right down, pitch +0.1pi");
		ROS_INFO("[roll pitch yaw]: [%lf %lf %lf]", roll, pitch, yaw);
	}
	if(msg->code == 276)
	{	
		pitch -= 0.1*PI;
		ROS_INFO("arrow left down, pitch -0.1pi");
		ROS_INFO("[roll pitch yaw]: [%lf %lf %lf]", roll, pitch, yaw);
	}
	if(msg->code == 280)
	{	
		yaw += 0.1*PI;
		ROS_INFO("page up down, yaw +0.1pi");
		ROS_INFO("[roll pitch yaw]: [%lf %lf %lf]", roll, pitch, yaw);
	}
	if(msg->code == 281)
	{	
		yaw -= 0.1*PI;
		ROS_INFO("page down down, yaw -0.1pi");
		ROS_INFO("[roll pitch yaw]: [%lf %lf %lf]", roll, pitch, yaw);
	}
	if(msg->code == 115)
	{	
		mouse_diff_pose[2] -= 0.05;
		ROS_INFO("s down, z -0.05m");
		ROS_INFO("relative pose: [x y z]: [%lf %lf %lf]", mouse_diff_pose[0], mouse_diff_pose[1], mouse_diff_pose[2]);
	}
	if(msg->code == 119)
	{	
		mouse_diff_pose[2] += 0.05;
		ROS_INFO("w down, z +0.05m");
		ROS_INFO("relative pose: [x y z]: [%lf %lf %lf]", mouse_diff_pose[0], mouse_diff_pose[1], mouse_diff_pose[2]);
	}
	if(msg->code == 32)
	{
		ROS_INFO("reset angles and z position!");
		roll = pitch = yaw = 0;
		mouse_diff_pose[2] = 0;	
		ROS_INFO("[roll pitch yaw]: [%lf %lf %lf]", roll, pitch, yaw);
		ROS_INFO("relative pose: [x y z]: [%lf %lf %lf]", mouse_diff_pose[0], mouse_diff_pose[1], mouse_diff_pose[2]);
	}
}





