#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ContactsState.h"
#include "sensor_msgs/LaserScan.h"
//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

#include<algorithm>
#include<string>
#include<limits>

//ros::Publisher motor commands;
ros::Publisher motor_command_publisher;
ros::Subscriber bumper_information_subscriber;
ros::Subscriber laser_scan_sub;

int bumper_contact = 0;
float min_dist = std::numeric_limits<float>::max() ;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested.
// This function should publish the requested linear x and angluar velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities.

bool relay_robot_velocities(ball_chaser::DriveToTarget::Request& req,ball_chaser::DriveToTarget::Response& res)
{
	ROS_INFO("DriveToTarget received - x:%1.2f, z:%1.2f", (float)req.linear_x,(float)req.angular_z);

	float linear_x = (float)req.linear_x;
	float angular_z = (float)req.angular_z;	

	//Create a motor_command object of type geometry_msgs::Twist
	geometry_msgs::Twist motor_command;
	//Set wheel velocities, forward [0.5, 0.0]

	if(bumper_contact && min_dist<0.2)
		ROS_INFO("Move the ball, the robot has collided with it (hopefully)! Distance between robot and ball is %1.2f", min_dist);
	
	
	motor_command.linear.x = linear_x;
	motor_command.angular.z = angular_z;
	
	//Publish angles to drive the robot
	motor_command_publisher.publish(motor_command);


	res.msg_feedback = "Linear velocity is "+ std::to_string(linear_x) + ", Angular velocity is " + std::to_string(angular_z);
	
}


void stop_robot(gazebo_msgs::ContactsState contact)
{
	if(contact.states.size()>0)
		bumper_contact = 1;
}

void assign_scan(sensor_msgs::LaserScan laser)
{
	min_dist = *(std::min_element(std::begin(laser.ranges), std::end(laser.ranges)));

}

int main(int argc, char** argv)
{
	//Initialize a ROS node
	ros::init(argc, argv, "drive_bot");

	//Create a ROS NodeHandle object
	ros::NodeHandle n;

	//Set the loop rate at 10Hz
	ros::Rate loop_rate(10);

	//Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic
	//with a publishing queue size of 10
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	bumper_information_subscriber = n.subscribe("/bumper_value",1000,stop_robot);
	laser_scan_sub = n.subscribe("/scan",1000,assign_scan);

	//TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
	ros::ServiceServer vel_commands = n.advertiseService("ball_chaser/command_robot",relay_robot_velocities);
	//TODO: Handle ROS communication events

	ros::spin();

	return 0;

}
