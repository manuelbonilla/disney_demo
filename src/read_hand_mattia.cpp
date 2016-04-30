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

    ros::Publisher pub;
    pub = nh.advertise<std_msgs::Bool>("/go", 100);
	
	std_msgs::Bool bool_msg;						
	
	activated_ = false;

	ros::Subscriber sub;
	sub = nh.subscribe("/read_hand", 1, read_hand_activated);




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


	while(ros::ok())
	{
		if(activated_ == 2)
		{
			values[0] = HAND_CLOSED;
			values[1] = 0;   // motor 2		
			commSetInputs(&comm_settings_t, 0, values);
	    	usleep(250000);
		}

		if(activated_ == 1) 
		{
			/********************** Movement: 3rd stage: closure + rotation **********************/
			ROS_INFO_STREAM("Waiting for Movement 3");

			values[0] = HAND_CLOSED;
			values[1] = 0;   // motor 2		
			commSetInputs(&comm_settings_t, 0, values);
	    	usleep(250000);
			
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
				if (!cont && abs(imu_values[0]-last_accel[0]) > 300 && abs(imu_values[1]-last_accel[1]) < 300 && abs(imu_values[2]-last_accel[2]) < 300)
				{
						cout << "Bump detected" << endl;
						
						cout << "diff 0: " << abs(imu_values[0] - last_accel[0]) << endl;
						cout << "diff 1: " << abs(imu_values[1] - last_accel[1]) << endl;
						cout << "diff 2: " << abs(imu_values[2] - last_accel[2]) << endl;
						cout << "diff norm: " << fabs(imu_norm - last_norm) << endl << endl;
						cout << "gyro: " << imu_values[3] << '\t' << imu_values[4] << '\t' << imu_values[5] << endl;
						cont = 1;
				}
				else 
				{
					//cout << "No bump" << endl;
					cont = 0;
				}
					
				/*	cout << "imu: " << imu_values[0] << " diff: " << abs(imu_values[0] - last_accel[0]) << endl;
					cout << "imu: " << imu_values[1] << " diff: " << abs(imu_values[1] - last_accel[1]) << endl;
					cout << "imu: " << imu_values[2] << " diff: " << abs(imu_values[2] - last_accel[2]) << endl;
					cout << "diff norm: " << fabs(imu_norm - last_norm) << endl << endl;
				*/
				
				last_accel[0] = imu_values[0];
				last_accel[1] = imu_values[1];
				last_accel[2] = imu_values[2];
				last_norm = imu_norm;
				
				cout << endl;
	    		usleep(100000);
			}
			
			if(cont)
			{
		   		bool_msg.data = true;
			   	pub.publish(bool_msg);
			   	ros::spinOnce();
			}

			/************************* Movement: 4th stage: hand opening *************************/		
			if (cont)
			{
				
				// Movement implemented into code
				hand_movement = 1;
				values[0] = HAND_CLOSED;
				
				//Movement: 4th sleeptage: open hand
				// Open SoftHand, while moving fingers up and down
				while (hand_movement) 
				{
					
					if (values[0] < 1000)
					{   // The hand has completely opened
						
						// Re-close hand and finishes primitive
						//values[0] = HAND_CLOSED;
						
						//Sleep(1000);  // Wait for hand closure
						
						hand_movement = 0;
					}
					else 
					{
						
						// Set hand reference position
						values[0] -= 800;                // motor 1
						
					}
				
					values[1] = 0;   // motor 2
					
					commSetInputs(&comm_settings_t, 0, values);
					//cout << values[0] << " " << values[1] << endl;
					usleep(200000);
				}

			}

			cont = 0;
			activated_ = 3;
		}// end if activated

		ros::spinOnce();
	}// end while	


	return 0;
}