//general utilities
#include <cmath>
#include <math.h>
#include <fstream>
#include <string>
#include <time.h>
#include <stdlib.h>
// #include <flann/flann.h>
// #include <flann/io/hdf5.h>
#include <termios.h>
#include <iostream>
#include <stdio.h>


// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include <controller_manager_msgs/SwitchController.h>


// utilities
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/SVD>
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>


#include "commands.h"
#include "qbmove_communications.h"
#include "iostream"
#include <math.h>
#include <fstream>
#include <signal.h>

#define T0ramp	0.0	
#define T1ramp	0.5
#define T2ramp	1.5
#define TFramp  2.0

#define T0wave 	0.0
#define T1wave	0.2
#define T2wave	1.0
#define TFwave	1.5

#define	OMEGA		(2*M_PI*4)
#define AMPLITUDE	5000

#define M 		(1/(TFramp - T0ramp))
#define A 		(M/(T1ramp - T0ramp))
#define B 		(M/(TFramp - T2ramp))
#define Q 		(M*(T1ramp - T0ramp))

comm_settings comm_settings_t ;
short int values[2] = {0,0};

short int imu_values[6] = {0,0,0,0,0,0};
double imu_norm = 0.0;

int activated_;

#define HAND_CLOSED 17000




void audio_disney(std_msgs::Bool m)
{	
	if(m.data)
	{	
		system("mplayer  /home/emanuele/catkin_ws/src/disney_demo/src/BayMax.mp3 >/dev/null 2>&1");
		system("mplayer  /home/emanuele/catkin_ws/src/disney_demo/src/BayMax.mp3 >/dev/null 2>&1");
		system("mplayer  /home/emanuele/catkin_ws/src/disney_demo/src/BayMax.mp3 >/dev/null 2>&1");
	}
}


int main(int argc, char** argv)
{

	/*** ros parameters ***/
	ros::init(argc,argv,"audio_node");
    ros::NodeHandle nh;

	
	ros::Subscriber sub;
	sub = nh.subscribe("/audio_disney", 1, audio_disney);
	

	ros::spin();

	return 0;
}

