#include "disneyDemo.h"


/* Constructor */
disneyDemo::disneyDemo()
{
	initVariables();
}


/* Constructor */
disneyDemo::~disneyDemo()
{
    // nothing to be done
}



/**********************************************************************************************/
/*																					  		  */
/*																					  		  */
/*					                  		 FUNCTIONS                                        */
/*																					    	  */
/*																					  		  */
/**********************************************************************************************/


// =============================================================================================
//																				   initVariables
// =============================================================================================
void disneyDemo::initVariables()
{
	// pub_ = nh_.advertise<lwr_controllers::PoseRPY>("left_arm/one_task_inverse_dynamics/command", 3); 	
	pub_ = nh_.advertise<lwr_controllers::PoseRPY>("left_arm/one_task_inverse_kinematics/command", 3); 	

	// sub_ = nh_.subscribe("left_arm/one_task_inverse_dinamics/check", 1, &disneyDemo::checkMovement, this);

	checkMovement_value_ = false;
}



// =============================================================================================
//																				   initVariables
// =============================================================================================
void disneyDemo::managerKuka()
{
	bool flag 	   	= false;
	bool exit_user 	= false;
	float diff=0;
	
	getTransform(H_vito_EE_, "vito_anchor", "left_arm_base_link");
	getTransform(H_, "left_arm_7_link","left_hand_palm_link");

	std::vector<double> v;
	v.resize(7);
	v[0] = 0;
	v[1] = 0;
	v[2] = 0;
	v[3] = -1.5;
	v[4] = -1.5;
	v[5] = 0.5;
	v[6] = 0;


	publisher(v);
}



// =============================================================================================
//																				   publisher
// =============================================================================================
void disneyDemo::publisher(std::vector<double> v)
{
	Eigen::Vector3d r;
	Eigen::Vector3d p; 
	Eigen::Vector3d pp;
	
	// r << 0,0,90*(M_PI*180);
	// p << 0.10, 0.10, 0.10;

	r << v[0], v[1], v[2];
	p << v[3], v[4], v[5];

	p = H_vito_EE_.R*p + H_vito_EE_.v;
	// pp <<  -0.002,0.00695,-0.0905;
	pp = H_vito_EE_.T.topLeftCorner(3,3)*rotZ(r(2))*rotY(r(1))*rotX(r(0)) * pp;
	p = p+pp;

	r = Rot2Angle(H_vito_EE_.T.topLeftCorner(3,3)*rotZ(r(2))*rotY(r(1))*rotX(r(0)));

	std::cout << "final pos: " << p[0] << " " << p[1]<< " " << p[2]<<std::endl; 

	lwr_controllers::PoseRPY local_msg;
	local_msg.id = 0;
    local_msg.position.x = p(0);
    local_msg.position.y = p(1);
    local_msg.position.z = p(2);
    local_msg.orientation.roll  = r(0);
    local_msg.orientation.pitch = r(1);
    local_msg.orientation.yaw   = r(2);

	pub_.publish(local_msg);
	ros::spinOnce();

	// while(!checkMovement_value_)
	// {
		// std::cout << "STO ASPETTANDO" << std::endl;
		ros::spinOnce();
	// }

	checkMovement_value_ = true;
	// std::cout << "SONO Fuori" << std::endl;
}



// =============================================================================================
//																				   checkMovement
// =============================================================================================
void disneyDemo::checkMovement(std_msgs::Bool msg)
{
	checkMovement_value_ = msg.data;
}



// =============================================================================================
//																				  	getTransform
// =============================================================================================
void disneyDemo::getTransform(homogeneous_vito& x, std::string link_from, std::string link_to)
{
	Eigen::Quaterniond q;

	tf::TransformListener listener;
	tf::StampedTransform t;
	
	
	try
	{
		listener.waitForTransform(link_from, link_to, ros::Time(0), ros::Duration(1)); //ros::Duration(2.5)
		listener.lookupTransform(link_to,link_from,  ros::Time(0), t);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	tf::quaternionTFToEigen(t.getRotation(), q);
	tf::vectorTFToEigen(t.getOrigin(), x.v);

	x.R = q.toRotationMatrix();
	
	x.T = Eigen::Matrix4d::Identity();
	x.T.topLeftCorner(3,3)  = x.R;
	x.T.col(3) = Eigen::Vector4d(x.v(0),x.v(1),x.v(2), 1);
}




// =============================================================================================
//																				  	   Utilities
// =============================================================================================
Eigen::Vector3d disneyDemo::Rot2Angle(Eigen::Matrix3d R_in)
{
  Eigen::Vector3d Angles_out;

  Angles_out(0) = atan2(R_in(2,1),R_in(2,2)); // ROLL
  Angles_out(1) = atan2(-R_in(2,0), sqrt(R_in(2,1)*R_in(2,1) + R_in(2,2)*R_in(2,2))); //PITCH
  Angles_out(2) = atan2(R_in(1,0),R_in(0,0)); //YAW

  return Angles_out;    
}


Eigen::Matrix3d disneyDemo::rotX(float x)
{
  Eigen::Matrix3d Rx;
  x = x *(M_PI/180);
  Rx << 1 , 0, 0,
        0, cos(x), -sin(x),
        0, sin(x), cos(x);
  return Rx;
}

Eigen::Matrix3d disneyDemo::rotY(float y)
{
  Eigen::Matrix3d Ry;
  y = y *(M_PI/180);
  Ry << cos(y) , 0, sin(y),
      0,       1,   0,
      -sin(y), 0, cos(y);
  return Ry;
}

Eigen::Matrix3d disneyDemo::rotZ(float z)
{
    Eigen::Matrix3d Rz;
    z = z *(M_PI/180);
    Rz << cos(z), -sin(z), 0,
      sin(z), cos(z),  0,
      0,        0,     1;
    return Rz;
}