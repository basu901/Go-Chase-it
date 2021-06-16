#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include<sensor_msgs/Image.h>
#include<math.h>

#define _USE_MATH_DEFINES

//Define a global client that can request services
ros::ServiceClient client;

//This function calls the command_robot service to drive the robot in the specified direction

void drive_robot(float lin_x, float ang_z)
{
	//TODO: Request a service and pass the velocities to it to drive the robot
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	if(!client.call(srv))
		ROS_ERROR("Failed to call service command_robot");


}

//This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

	int white_pixel = 255;

	//TODO: Loop through each pixel in the image and check if theres a bright spot because of the ball
	//Identify if the pixel is to the left or right or center and accordingly move the bot.
	//Request a stop when there's no white ball in the camera

	bool found_ball= false;
	int pos = 0, col_pos = 0;
	float vel = 0.0 , angle = 0.0;

	int i,j;
	
	for(i = 0;i <img.height*img.width;i++)
	{
		if(img.data[i]==white_pixel)
		{
			found_ball=true;
			ROS_INFO("Found the ball!");
			pos = i;
			break;
		}

		//ROS_INFO("%d",img.data[i]);
	}

		
	if(found_ball)
	{
		ROS_INFO("Found the ball");
		int col_pos = pos%img.width;
		vel = 0.0;
		angle = ((col_pos - 0.5*img.width)/(0.5*img.width))*M_PI_2;
		ROS_INFO("Set the angle at %1.3f",angle);
	}

	
	drive_robot(vel,angle);

}

int main(int argc, char** argv)
{

	//Initialize the process_image node and create a handle to it
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	//Define a client service capable of requesting services from command_robot
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw",10,process_image_callback);

	//Handle ROS communication events
	ros::spin();

	return 0;

}
