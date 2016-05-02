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

#define HAND_CLOSED 19000

using namespace std;


float computeDelta(float t);
float computeWave(float t);
int computeTrajectory1(float t);
int computeTrajectory2(float t);

void read_hand_activated(std_msgs::Int16 m)
{
	activated_ = m.data;
}


int main(int argc, char** argv)
{

	/*** ros parameters ***/
	ros::init(argc,argv,"read_hand_double");
    ros::NodeHandle nh;

    ros::Publisher pub, pub_audio;
    pub = nh.advertise<std_msgs::Bool>("/go", 100);
    pub_audio = nh.advertise<std_msgs::Bool>("/audio_disney", 100);
	
	ros::Subscriber sub;
	sub = nh.subscribe("/read_hand", 1, read_hand_activated);
	
	std_msgs::Bool bool_msg, bool_audio_msg;						
	activated_ = false;

	std::vector<double> p1,p2;
	p1.resize(400);
	p2.resize(400);
    nh.param<std::vector<double>>("p1", p1, std::vector<double>{0,0,0,0,0,0,0});
    nh.param<std::vector<double>>("p2", p2, std::vector<double>{0,0,0,0,0,0,0});

    double spin_rate = 100;
	ros::Rate rate(spin_rate);




	/*** hand parameters ***/
	short int last_accel[3] = {0,0,0};
	double last_norm = 0;
	int cont = 0;
	

    openRS485 ( &comm_settings_t , "/dev/ttyUSB0" ) ;       // QB Board Opening
    sleep(1);

	commActivate(&comm_settings_t, 0, 1);
    usleep(250000);
    sleep(1);
			

	commBumpDetectionActivate(&comm_settings_t, 0, 0);
    usleep(250000);
    sleep(1);


    commGetIMUMeasurements(&comm_settings_t, 0, imu_values);
	imu_norm = sqrt(imu_values[0]*imu_values[0] + imu_values[1]*imu_values[1] + imu_values[2]*imu_values[2]);
	last_accel[0] = imu_values[0];
	last_accel[1] = imu_values[1];
	last_accel[2] = imu_values[2];
	imu_values[3] = 0;
	imu_values[4] = 0;
	imu_values[5] = 0;
	last_norm = imu_norm;

	// opening hand
	values[0] = 0;   // motor 1
	values[1] = 0;   // motor 2		
	commSetInputs(&comm_settings_t, 0, values);
	usleep(250000);
	
	bool open_hand = false;
	float temp = 0;
			

	while(ros::ok())
	{

		switch (activated_)
		{
		
			case 1:
				ROS_INFO_STREAM("Closing Hand - Fast Mode");
				values[0] = HAND_CLOSED;
				values[1] = HAND_CLOSED;   // motor 2		
				commSetInputs(&comm_settings_t, 0, values);
	    		usleep(250000);
	    		break;

	   
	    	case 2:
				ROS_INFO_STREAM("Opening Hand - Slow Mode");
				// Open SoftHand, while moving fingers up and down
				// for (int i=0; i < (int) p1.size(); i++ )
				// {
				// 	values[0] = p1[i]*HAND_CLOSED;   // motor 1
				// 	values[1] = p2[i]*HAND_CLOSED;   // motor 2
				// 	commSetInputs(&comm_settings_t, 0, values);
				// 	rate.sleep();
				// }
				
				open_hand = false;
				temp = 0;
				
				bool_audio_msg.data = true;
	   			pub_audio.publish(bool_audio_msg);
	   			ros::spinOnce();

				while(!open_hand)
				{
					
					values[0] = computeTrajectory2(temp);
					values[1] = computeTrajectory1(temp);
					commSetInputs(&comm_settings_t, 0, values);

					// cout << "\nvalue[0]: " << values[0] << std::endl;
					// cout << "values[1]: "  << values[1] << std::endl;

					rate.sleep();
					temp += 0.01;
					if (temp >= 3)
						open_hand = true;
				}

				break;


			case 3:
				ROS_INFO_STREAM("Closing Hand - Bit Mode");
				values[0] = 8000;
				values[1] = 8000;   // motor 2		
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
					// imu_norm = sqrt(imu_values[0]*imu_values[0] + imu_values[1]*imu_values[1] + imu_values[2]*imu_values[2]);
					// cout << "Norm: " << imu_norm << endl;
					// cout << "gyro: " << imu_values[3] << '\t' << imu_values[4] << '\t' << imu_values[5] << endl;
					
					//!cont is useful to avoid difference from last bump could be seen as a new bump
					// if (!cont && fabs(imu_norm - last_norm) > 250 && fabs(imu_norm - last_norm) < 500
					// 		&& abs(imu_values[0]-last_accel[0]) > 100 && abs(imu_values[1]-last_accel[1]) < 300 && abs(imu_values[2]-last_accel[2]) < 300)
					// if (!cont && abs(imu_values[0]-last_accel[0]) > 300 && abs(imu_values[1]-last_accel[1]) < 300 && abs(imu_values[2]-last_accel[2]) < 300)
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
	
				}
				break;
		}// end switch
		
		imu_values[3] = 0;
		imu_values[4] = 0;
		imu_values[5] = 0;
		activated_ = 0;
		ros::spinOnce();
	}// end while	


	return 0;
}



float computeDelta(float t)
{
	float delta;
	if ( t<T0ramp )
	{
		delta = 0;
	}
	else if (t<T1ramp )
	{
		delta = A * (t - T0ramp)* (t - T0ramp);
 	}
 	else if (t < T2ramp)
 	{
 		delta = M * (t - T1ramp) + Q;
 	}
 	else if (t < TFramp)
 	{		
 		delta = 1 - B * (t - TFramp)*(t - TFramp);
 	}
 	else //(t > TFramp)
 		delta = 1;

 	return delta;

}

float computeWave(float t)
{
	float env;
	if ( t<T0wave )
	{
		env = 0;
	}
	else if (t<T1wave )
	{
		env = 0.5 -  0.5* cos(M_PI*(t - T0wave) / (T1wave - T0wave));      
 	}
 	else if (t < T2wave)
 	{
 		env = 1;
 	}
 	else if (t < TFwave)
 	{		
		env = 0.5 +  0.5* cos(M_PI*(t - TFwave) / (T2wave - TFwave));      
 	}
 	else 
 		env = 0;

 	return ( sin(OMEGA * t) * env );

}

int computeTrajectory1 (float t)
{
	return (int) (computeWave(t)*AMPLITUDE + HAND_CLOSED - computeDelta(t)*HAND_CLOSED);
}


int computeTrajectory2 (float t)
{
	return (int) (-computeWave(t)*AMPLITUDE + HAND_CLOSED - computeDelta(t)*HAND_CLOSED);
}

