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


comm_settings comm_settings_t ;
short int values[2] = {0,0};

short int imu_values[6] = {0,0,0,0,0,0};
double imu_norm = 0.0;

int activated_;

#define HAND_CLOSED 15000

using namespace std;


void read_hand_activated(std_msgs::Int16 m)
{
	activated_ = m.data;
}


int main(int argc, char** argv)
{

	// ros parameters
	ros::init(argc,argv,"read_hand");
    ros::NodeHandle nh;

    ros::Publisher pub, pub_audio;
    pub = nh.advertise<std_msgs::Bool>("/go", 100);
	
	std_msgs::Bool bool_msg, bool_audio_msg ;						
	
	activated_ = false;

	ros::Subscriber sub;
	sub = nh.subscribe("/read_hand", 1, read_hand_activated);

	pub_audio = nh.advertise<std_msgs::Bool>("/audio_disney", 100);


	// hand parameters
	short int last_accel[3] = {0,0,0};
	double last_norm = 0;
	int cont = 0;
	
    openRS485 ( &comm_settings_t , "/dev/ttyUSB0" ) ;       // QB Board Opening

    // sleep(250000);
    sleep(1);

	commActivate(&comm_settings_t, 0, 1);
    usleep(250000);
    sleep(1);
			
	int hand_movement = 0;
	
	commBumpDetectionActivate(&comm_settings_t, 0, 0);
    usleep(250000);
    sleep(1);

    commGetIMUMeasurements(&comm_settings_t, 0, imu_values);
	imu_norm = sqrt(imu_values[0]*imu_values[0] + imu_values[1]*imu_values[1] + imu_values[2]*imu_values[2]);
	last_accel[0] = imu_values[0];
	last_accel[1] = imu_values[1];
	last_accel[2] = imu_values[2];
	last_norm = imu_norm;


	values[0] = 0;
	values[1] = 0;   // motor 2		
	commSetInputs(&comm_settings_t, 0, values);
	usleep(250000);
	
	while(ros::ok())
	{

		switch (activated_)
		{
		
			case 1:
				ROS_INFO_STREAM("Closing Hand - Fast Mode");
				values[0] = HAND_CLOSED;
				values[1] = 0;   // motor 2		
				commSetInputs(&comm_settings_t, 0, values);
	    		usleep(250000);
	    		break;

	   
	    	case 2:
				ROS_INFO_STREAM("Opening Hand - Slow Mode");
				// Open SoftHand, while moving fingers up and down
				
				bool_audio_msg.data = true;
	   			pub_audio.publish(bool_audio_msg);
	   			ros::spinOnce();

				hand_movement = 1;
				values[0] = HAND_CLOSED;
				while (hand_movement) 
				{
					if (values[0] < 1000)
					{   
						hand_movement = 0;
					}
					else 
					{
						values[0] -= 800; // motor 1
					}
					values[1] = 0;   	  // motor 2
					commSetInputs(&comm_settings_t, 0, values);
					usleep(200000);
				}
				break;


			case 3:
				ROS_INFO_STREAM("Closing Hand - Bit Mode");
				values[0] = 5000;
				values[1] = 0;   // motor 2		
				commSetInputs(&comm_settings_t, 0, values);
	    		usleep(250000);
	    		break;




			case 9:
				ROS_INFO_STREAM("Waiting Bump");
				cont = 0;
				while(!cont) 
				{
					commGetIMUMeasurements(&comm_settings_t, 0, imu_values);
					// Compute bump reading IMU and move SoftHand accordingly
					imu_norm = sqrt(imu_values[0]*imu_values[0] + imu_values[1]*imu_values[1] + imu_values[2]*imu_values[2]);
					// cout << "Norm: " << imu_norm << endl;
					
					//!cont is useful to avoid difference from last bump could be seen as a new bump
					// if (!cont && fabs(imu_norm - last_norm) > 250 && fabs(imu_norm - last_norm) < 500
					// 		&& abs(imu_values[0]-last_accel[0]) > 100 && abs(imu_values[1]-last_accel[1]) < 300 && abs(imu_values[2]-last_accel[2]) < 300)
					// if (!cont && abs(imu_values[0]-last_accel[0]) > 300 && abs(imu_values[1]-last_accel[1]) < 300 && abs(imu_values[2]-last_accel[2]) < 300)
					std::cout << "gyro[0]: " << abs(imu_values[3]) << "gyro[1]: " << abs(imu_values[3]) <<  "gyro[2]: " << abs(imu_values[3]) << std::endl;
					if (abs(imu_values[3]) > 650 || abs(imu_values[4]) > 650 || abs(imu_values[5]) > 650 )
					{
						cout << "Bump detected" << endl;
						
						cout << "diff 0: " << abs(imu_values[0] - last_accel[0]) << endl;
						cout << "diff 1: " << abs(imu_values[1] - last_accel[1]) << endl;
						cout << "diff 2: " << abs(imu_values[2] - last_accel[2]) << endl;
						cout << "diff norm: " << fabs(imu_norm - last_norm) << endl << endl;
						cout << "gyro: " << imu_values[3] << '\t' << imu_values[4] << '\t' << imu_values[5] << endl;

						bool_msg.data = true;
			   			pub.publish(bool_msg);
			   			ros::spinOnce();
						cont = 1;
					}

					last_accel[0] = imu_values[0];
					last_accel[1] = imu_values[1];
					last_accel[2] = imu_values[2];
					last_norm = imu_norm;
					
					// cout << endl;
		    		// usleep(100000);
				}
				break;
		}// end switch
		
		imu_values[3] = 0;
		imu_values[4] = 0;
		imu_values[5] = 0;
		// std::cout << "\nwhile read\n";
		activated_ = 0;
		ros::spinOnce();
	}// end while	


	return 0;
}