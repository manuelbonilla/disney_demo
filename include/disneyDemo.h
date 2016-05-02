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
#include <std_msgs/Float64.h>



// KUKA controllers
#include "lwr_controllers/PoseRPY.h"



// Hand Headers
// #include "qbmove_communications.h"

#define CLOSE_HAND_FAST 1
#define OPEN_HAND_SLOW  2
#define CLOSE_HAND_BIT  3
#define WAITING_BUMP    9 




#define TH_ERROR_1  0.08    // th 1 for final goal 
#define TH_ERROR_2  0.05    // th 2 for final goal 
#define CLOSE_STEP  200     // min hand step close for smoothing close


class disneyDemo
{
public:
	disneyDemo();
	~disneyDemo();

    void firstMovement();
    void initDemo();
    void managerKukaSingle();
    void managerKukaComplete();

private:
    //Node handle
    ros::NodeHandle nh_, n_;
    std::vector<double> home1_, home2_;
    std::vector<double> back1_, back2_;           
    

    //Message Pub
    void publisher(std::vector<double> v1, std::vector<double> v2);
    ros::Publisher pub1_, pub2_, pub_start_robot_, pub_read_hand_, pub_stiffness_, pub_first_time_;


    // Message Sub
    ros::Subscriber sub_check_, sub_go_;
    bool check_, go_, prima_volta_;
    void check(std_msgs::Bool bool_msg);
    void go(std_msgs::Bool bool_msg);


    std::string control_topic_left_;
    
    void pubHand(int x);
    void waiting();

    // NON UTILIZZATE
    void getInitPosition(std::vector<double> v);  // IMPORTAN have to define EE link 
    Eigen::Vector3d Rot2Angle(Eigen::Matrix3d R_in);
    Eigen::Matrix3d rotX(float x);
    Eigen::Matrix3d rotY(float y);
    Eigen::Matrix3d rotZ(float z);


};

















