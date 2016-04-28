//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
// #include <flann/flann.h>
// #include <flann/io/hdf5.h>
#include <vector>
#include <algorithm>


// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

// Hy headers
#include "disneyDemo.h"




int main(int argc, char** argv)
{
    ros::init(argc,argv,"manager_node");
    ros::NodeHandle nh_main;



    disneyDemo k;
    k.initDemo();
    


    std::cout <<"\033[34m\033[1mStart DisneyDemo\033[0m" << std::endl;
    // k.firstMovement();

  	bool exit_flag = false;

    while (!exit_flag)
    {   
        k.managerKuka();

        if(!ros::ok())
         exit_flag = false;
    }



    std::cout << "\r\n\n\n\033[32m\033[1mEXIT! \033[0m" << std::endl;
    
    return 0;
}
