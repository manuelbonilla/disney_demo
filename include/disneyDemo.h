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


// KUKA controllers
#include "lwr_controllers/PoseRPY.h"



// Hand Headers
// #include "qbmove_communications.h"

struct homogeneous_vito
{
    // rotation matrix
    Eigen::Matrix3d R;
    // traslation vector 
    Eigen::Vector3d v;
    // homogeneous matrix
    Eigen::Matrix4d T;
};



#define CLOSE_STEP  200     // min hand step close for smoothing close


class disneyDemo
{
public:
	disneyDemo();
	~disneyDemo();

	void managerKuka();


private:
    //Node handle
    ros::NodeHandle nh_, n_;

    //Message Pub
    void publisher(std::vector<double> v);
    ros::Publisher pub_;
    lwr_controllers::PoseRPY msg_;

    // Message Sub
    ros::Subscriber sub_;
    bool checkMovement_value_;
    void checkMovement(std_msgs::Bool msg);
    bool my_break_;
    

    std::vector<double> v_record_;


    void initVariables();
    void getTransform(homogeneous_vito& x, std::string link_from, std::string link_to);


    void getInitPosition(std::vector<double> v);  // IMPORTAN have to define EE link 
    Eigen::Vector3d Rot2Angle(Eigen::Matrix3d R_in);
    Eigen::Matrix3d rotX(float x);
    Eigen::Matrix3d rotY(float y);
    Eigen::Matrix3d rotZ(float z);
    homogeneous_vito H_vito_EE_;  
    homogeneous_vito H_;

};

















