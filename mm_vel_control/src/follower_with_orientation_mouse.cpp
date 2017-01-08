#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>

std::vector<double> joint_speeds;
std::vector<double> joint_values;
bool joint_data_come = false;


ros::Subscriber sub_joint_states;
ros::Publisher pub_joint_speed;

ros::Publisher pub_pan;
ros::Publisher pub_lift;
ros::Publisher pub_elbow;
ros::Publisher pub_wrist1;
ros::Publisher pub_wrist2;
ros::Publisher pub_wrist3;

ros::Publisher pub_linear_v;
ros::Publisher pub_angular_v;


/*callback functions*/
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
/*other functions*/
void updateDiff(tf::TransformListener &tf_listener, Eigen::Vector3d &diff_vector, tf::Vector3 &axis, double &shortest_angle);
void movingAvarage(Eigen::MatrixXd &joint_ctrl_velocities, std::vector<double> &result);
void integralDiff(Eigen::Vector3d &diff_vector, double shortest_angle, Eigen::Vector3d &integral_vector, double &integral_angle);
double calcNorm(Eigen::MatrixXd vector);
double calcNorm(tf::Vector3 vector);

int main(int argc, char** argv)
{
	joint_speeds.resize(6);
	joint_values.resize(6);


	ros::init(argc, argv, "ur5_jacobian_follower");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	ros::AsyncSpinner spinner(1);
	spinner.start();
	sub_joint_states = n.subscribe("/joint_states", 1000, jointStateCallback);
	
	pub_pan = n.advertise<std_msgs::Float64>("/arm_shoulder_pan_joint_vel_controller/command", 1000);
    pub_lift = n.advertise<std_msgs::Float64>("/arm_shoulder_lift_joint_vel_controller/command", 1000);
    pub_elbow = n.advertise<std_msgs::Float64>("/arm_elbow_joint_vel_controller/command", 1000);
    pub_wrist1 = n.advertise<std_msgs::Float64>("/arm_wrist_1_joint_vel_controller/command", 1000);
    pub_wrist2 = n.advertise<std_msgs::Float64>("/arm_wrist_2_joint_vel_controller/command", 1000);
    pub_wrist3 = n.advertise<std_msgs::Float64>("/arm_wrist_3_joint_vel_controller/command", 1000);
    
	pub_joint_speed = n.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 1000);

	pub_linear_v = n.advertise<std_msgs::Float64>("/linear_v", 1000);
	pub_angular_v = n.advertise<std_msgs::Float64>("/angular_v", 1000);
	std_msgs::Float64 linear_v_msg;
	std_msgs::Float64 angular_v_msg;

	/*for jacobian calculation*/
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame(base frame): %s", kinematic_model->getModelFrame().c_str());
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

	/*for velocities calculation*/
	Eigen::Vector3d reference_point_position(0.0,0.0,0.23);
	Eigen::MatrixXd jacobian;	//Jacobian of current configuration

	Eigen::MatrixXd joint_ctrl_velocities(6,1);		//joint velocities required
	Eigen::MatrixXd tcp_velocities_world(6,1);		//tcp velocities calculated by PID controller
	Eigen::Vector3d diff_vector(0.0, 0.0, 0.0);	 	//position Kp
	Eigen::Vector3d integral_vector(0.0, 0.0, 0.23); //position Ki

	tf::Vector3 axis(0, 0, 0); 	//orientation Kp
	double shortest_angle = 0; 	//orientation Kp
	double integral_angle = 0;  //orientation Ki
	tf::Vector3 t_w_wxyz;

	/*for command send*/
	trajectory_msgs::JointTrajectory trj;
	trajectory_msgs::JointTrajectoryPoint trjp;
	trjp.velocities.resize(6);
	trj.points.push_back(trjp);

	//TODO, Paramters of PID controller
	double Kp_pose = 4, Ki_pose = 0.001/5, Kp_orien = 2.8, Ki_orien = 0.000;

	tf::TransformListener tf_listener;
	
	updateDiff(tf_listener, diff_vector, axis, shortest_angle);
	integralDiff(diff_vector, shortest_angle, integral_vector, integral_angle);

	while(ros::ok())
	{
		if(joint_data_come)
		{
			joint_data_come = false;
			//calculate jacobian
			kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
			kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
			ROS_INFO_STREAM("Jacobian: \n" << jacobian);
			//end calculate jacobian

			//calculate TCP velocities
			tcp_velocities_world(0,0) = Kp_pose*diff_vector(0) + Ki_pose * integral_vector(0);
			tcp_velocities_world(1,0) = Kp_pose*diff_vector(1) + Ki_pose * integral_vector(1);
			tcp_velocities_world(2,0) = Kp_pose*diff_vector(2) + Ki_pose * integral_vector(2);
			double linear_v = calcNorm(tcp_velocities_world);
			if(linear_v > 0.5)
			{
				tcp_velocities_world(0,0) = tcp_velocities_world(0,0) * 0.5 / linear_v;
				tcp_velocities_world(1,0) = tcp_velocities_world(1,0) * 0.5 / linear_v;
				tcp_velocities_world(2,0) = tcp_velocities_world(2,0) * 0.5 / linear_v;
			}
			linear_v_msg.data = linear_v;
			pub_linear_v.publish(linear_v_msg);

			double angular_v = shortest_angle*Kp_orien + integral_angle * Ki_orien;
			if(angular_v > 0.1)
			{
				angular_v = 0.1;
			}
			angular_v_msg.data = angular_v;
			pub_angular_v.publish(angular_v_msg);

			t_w_wxyz = axis * angular_v;

			tcp_velocities_world(3,0) = t_w_wxyz[0];
			tcp_velocities_world(4,0) = t_w_wxyz[1];
			tcp_velocities_world(5,0) = t_w_wxyz[2];
			//end calculate TCP velocities

			//calculate joint velocities
			joint_ctrl_velocities = jacobian.inverse() * tcp_velocities_world;
			//moving averages
			movingAvarage(joint_ctrl_velocities, trj.points[0].velocities);
			//end moving averages
		}

		updateDiff(tf_listener, diff_vector, axis, shortest_angle);
		integralDiff(diff_vector, shortest_angle, integral_vector, integral_angle);

		trj.header.stamp = ros::Time::now();
		pub_joint_speed.publish(trj);
		std_msgs::Float64 temp_msg;
        temp_msg.data = trj.points[0].velocities[2];
        pub_elbow.publish(temp_msg);
        temp_msg.data = trj.points[0].velocities[1];
        pub_lift.publish(temp_msg);
        temp_msg.data = trj.points[0].velocities[0];
        pub_pan.publish(temp_msg);
        temp_msg.data = trj.points[0].velocities[3];
        pub_wrist1.publish(temp_msg);
        temp_msg.data = trj.points[0].velocities[4];
        pub_wrist2.publish(temp_msg);
        temp_msg.data = trj.points[0].velocities[5];
        pub_wrist3.publish(temp_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
}


double calcNorm(Eigen::MatrixXd vector)
{
	return sqrt(vector(0,0)*vector(0,0) + vector(1,0)*vector(1,0) + vector(2,0)*vector(2,0));
}

double calcNorm(tf::Vector3 vector)
{
	return sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
}

void integralDiff(Eigen::Vector3d &diff_vector, double shortest_angle, Eigen::Vector3d &integral_vector, double &integral_angle)
{
	//TODO, change the integral time
	int count = 150; //100Hz, 50 is represents 0.5s
	static std::vector<Eigen::Vector3d> vector_c(count, Eigen::Vector3d(0, 0, 0));
	static int index = 0;
	vector_c[index] = diff_vector;
	index++;
	if(index == count)index = 0;
	for(int i = 0; i != vector_c.size(); ++i)
		integral_vector += vector_c[i];

	static std::vector<double> angle_c(count, 0);
	static int index2 = 0;
	angle_c[index2] = shortest_angle;
	index2++;
	if(index2 == count)index2 = 0;
	for(int i = 0; i != angle_c.size(); ++i)
		integral_angle += angle_c[i];
}

void movingAvarage(Eigen::MatrixXd &joint_ctrl_velocities, std::vector<double> &result)
{
	static Eigen::MatrixXd joint_speed_c(6,10);
	static int index = 0;
	for(int ii = 0; ii<6; ii++)
	{
		joint_speed_c(ii, index) = joint_ctrl_velocities(ii, 0);
	}
	index++;
	if(index==10)index=0;
	double v1=0, v2=0, v3=0, v4 = 0, v5=0, v0=0;
	for(int iii=0; iii<10; iii++)
	{
		v0+=joint_speed_c(0,iii);
		v1+=joint_speed_c(1,iii);
		v2+=joint_speed_c(2,iii);
		v3+=joint_speed_c(3,iii);
		v4+=joint_speed_c(4,iii);
		v5+=joint_speed_c(5,iii);
	}
	v0/=10;
	v1/=10;
	v2/=10;
	v3/=10;
	v4/=10;
	v5/=10;
	result[0] = v0;
	result[1] = v1;
	result[2] = v2;
	result[3] = v3;
	result[4] = v4;
	result[5] = v5;
}

void updateDiff(tf::TransformListener &tf_listener, Eigen::Vector3d &diff_vector, tf::Vector3 &axis, double &shortest_angle)
{	

	tf::StampedTransform g_w_transform;//goal respect to world
	tf::StampedTransform t_w_transform;//tool respect to world
	tf::Transform g_t_transform;//goal respect to tool
	tf::Quaternion g_t_rotation;

	bool s = false;
	while(!s && ros::ok())
	{
		try
		{
			tf_listener.lookupTransform("odom", "arm_tool0", ros::Time(0), t_w_transform);
			tf_listener.lookupTransform("odom", "goal", ros::Time(0), g_w_transform);
			g_t_transform = t_w_transform.inverse() * g_w_transform;
			g_t_rotation = g_t_transform.getRotation();

			//third return value
			shortest_angle = g_t_rotation.getAngle();
			ROS_INFO("angle: %lf", shortest_angle);
			if(shortest_angle < 0.001)shortest_angle = 0;

			tf::StampedTransform t_w_rotation = t_w_transform;
			t_w_rotation.setOrigin(tf::Vector3(0,0,0));

			//second return value
			axis = t_w_rotation * g_t_rotation.getAxis();

			//first return value
			diff_vector(0) = g_w_transform.getOrigin().x() - t_w_transform.getOrigin().x();
			diff_vector(1) = g_w_transform.getOrigin().y() - t_w_transform.getOrigin().y();
			diff_vector(2) = g_w_transform.getOrigin().z() - t_w_transform.getOrigin().z();
			ROS_INFO("diff vector:%lf, %lf, %lf\n" , diff_vector(0), diff_vector(1), diff_vector(2));
			s = true;
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1).sleep();
			continue;
		}
	}
	
}


/*callback functions*/
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	if(msg->velocity.size() == 6)
	{
		//ROS_INFO("velocities: %lf, %lf, %lf, %lf, %lf, %lf", msg->velocity[0], msg->velocity[1], msg->velocity[2], msg->velocity[3], msg->velocity[4], msg->velocity[5]);
		joint_speeds[0] = msg->velocity[2];
		joint_speeds[1] = msg->velocity[1];
		joint_speeds[2] = msg->velocity[0];
		joint_speeds[3] = msg->velocity[3];
		joint_speeds[4] = msg->velocity[4];
		joint_speeds[5] = msg->velocity[5];
		joint_values[0] = msg->position[2];
		joint_values[1] = msg->position[1];
		joint_values[2] = msg->position[0];
		joint_values[3] = msg->position[3];
		joint_values[4] = msg->position[4];
		joint_values[5] = msg->position[5];
		joint_data_come = true;
	}
}

