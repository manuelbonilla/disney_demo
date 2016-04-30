#include "disneyDemo.h"


/* Constructor */
disneyDemo::disneyDemo()
{
	nh_.param<std::string>("/left_arm/controller", control_topic_left_, "disney_demo_controller_mt");
    pub_start_robot_ = nh_.advertise<std_msgs::Bool>("/left_arm/"+ control_topic_left_ + "/start_controller", 100);
	pub1_ = nh_.advertise<geometry_msgs::Pose>("/left_arm/"+ control_topic_left_ +"/command1", 3); 	
	pub2_ = nh_.advertise<geometry_msgs::Pose>("/left_arm/"+ control_topic_left_ +"/command2", 3); 	

   pub_read_hand_ = nh_.advertise<std_msgs::Int16>("/read_hand", 3);
   
   sub_check_ = nh_.subscribe("left_arm/"+control_topic_left_ +"/check", 1, &disneyDemo::check, this);
   sub_go_    = nh_.subscribe("go",1,&disneyDemo::go, this);
   
	
   check_ = false;
	go_    = false;

	home1_.resize(7);
  back1_.resize(7);
  nh_.param<std::vector<double>>("home1", home1_, std::vector<double>{0,0,0,0,0,0,0});
  nh_.param<std::vector<double>>("back1", back1_, std::vector<double>{0,0,0,0,0,0,0});

 	home2_.resize(7);
  back2_.resize(7);
  nh_.param<std::vector<double>>("home2", home2_, std::vector<double>{0,0,0,0,0,0,0});
  nh_.param<std::vector<double>>("back2", back2_, std::vector<double>{0,0,0,0,0,0,0});
}


/* Destructor */
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
//																				   		initDemo
// =============================================================================================
void disneyDemo::initDemo()
{
	ros::spinOnce();

	//controller switch
	ros::ServiceClient client;
	ROS_INFO_STREAM("Initializing Controllers");
	controller_manager_msgs::SwitchController switch_srv;
	switch_srv.request.start_controllers.push_back(control_topic_left_);
	switch_srv.request.stop_controllers.push_back("joint_trajectory_controller");
	switch_srv.request.strictness = 2;

	client = nh_.serviceClient<controller_manager_msgs::SwitchController>("/left_arm/controller_manager/switch_controller");
	if (!client.call(switch_srv))
    {
      	ROS_ERROR_STREAM("Not possible to switch left_arm control to: " << control_topic_left_.c_str());
    }
  	else
    {
      	if (switch_srv.response.ok == true)
        {
        	ROS_INFO_STREAM("left_arm control switched to: " << control_topic_left_.c_str());
        }
      	else
        {
          	ROS_ERROR_STREAM("Not possible to switch left_arm control to: " << control_topic_left_.c_str());
        }
    }
    ros::spinOnce();
    sleep(1);


    // set stiffness
    ros::Publisher pub_stiffness;
    pub_stiffness = nh_.advertise<std_msgs::Float64>("/left_arm/stiffness_scale", 3);
    std_msgs::Float64 f;
    f.data = 0.1;
    pub_stiffness.publish(f);

    // start controller
	std_msgs::Bool bool_msg;						
    bool_msg.data = true;

    pub_start_robot_.publish(bool_msg);
    ROS_INFO_STREAM("Starting Robot");
    ros::spinOnce();
    sleep(1);
}

// =============================================================================================
//																				   firstMovement
// =============================================================================================
void disneyDemo::firstMovement()
{
	std::vector<double> first1_, first2_; 
	
	first1_.resize(7);
    first2_.resize(7);
    nh_.param<std::vector<double>>("first1", first1_, std::vector<double>{0,0,0,0,0,0,0});
    nh_.param<std::vector<double>>("first2", first2_, std::vector<double>{0,0,0,0,0,0,0});

    publisher(first1_,first2_);
	std::cout << "\033[32m\033[1mFIRST MOVEMENT! \033[0m";
}



// =============================================================================================
//																				  singleMovement
// =============================================================================================
void disneyDemo::managerKukaSingle()
{
    std_msgs::Int16 m;

    std::cout<<"\033[32m\033[1mBUMB! \033[0m\n";

    nh_.param<std::vector<double>>("home1", home1_);
    nh_.param<std::vector<double>>("back1", back1_);

    nh_.param<std::vector<double>>("home2", home2_);
    nh_.param<std::vector<double>>("back2", back2_);


	if(go_)
	{	
      m.data = 2; 
      pub_read_hand_.publish(m);
      ros::spinOnce();

		publisher(back1_, back2_);

      m.data = 3; 
      pub_read_hand_.publish(m);
      ros::spinOnce();

		publisher(home1_, home2_);
	}

    // sent to hand new command
    m.data = 1; 
    pub_read_hand_.publish(m);
    go_ = false;
	ros::spinOnce();
}



// =============================================================================================
//                                                                              completeMovement
// =============================================================================================
void disneyDemo::managerKukaComplete()
{
    std::vector<double> start, one, two, home2a;

    start.resize(7);
    one.resize(7);
    two.resize(7);
    home2a.resize(7);

    nh_.param<std::vector<double>>("start", start, std::vector<double>{0,0,0,0,0,0,0});
    nh_.param<std::vector<double>>("one", one, std::vector<double>{0,0,0,0,0,0,0});
    nh_.param<std::vector<double>>("two", two, std::vector<double>{0,0,0,0,0,0,0});
    nh_.param<std::vector<double>>("home1", home1_, std::vector<double>{0,0,0,0,0,0,0});
    nh_.param<std::vector<double>>("home2a", home2a, std::vector<double>{0,0,0,0,0,0,0});
    nh_.param<std::vector<double>>("back1", back1_, std::vector<double>{0,0,0,0,0,0,0});
    nh_.param<std::vector<double>>("home2", home2_, std::vector<double>{0,0,0,0,0,0,0});
    nh_.param<std::vector<double>>("back2", back2_, std::vector<double>{0,0,0,0,0,0,0});


    publisher(start, home2_);
    std::cout<<"\033[32m\033[1mSTART \033[0m\n";
    pubHand(WAITING_BUMP);
    waiting();    


    publisher(one, home2_);
    pubHand(CLOSE_HAND_BIT);
    publisher(start, home2_);
    std::cout<<"\033[32m\033[1mONE \033[0m\n";
    pubHand(WAITING_BUMP);
    waiting();
    
    
    publisher(two, home2_);
    pubHand(CLOSE_HAND_FAST);
    publisher(home1_, home2_);
    std::cout<<"\033[32m\033[1mTWO \033[0m\n";
    pubHand(WAITING_BUMP);
    waiting();

    pubHand(OPEN_HAND_SLOW);    
    publisher(back1_ , back2_);
    ros::spinOnce();
}


// =============================================================================================
//                                                                                       pubHand
// =============================================================================================
void disneyDemo::pubHand(int x)
{   
    std_msgs::Int16 m;
    m.data = x; 
    pub_read_hand_.publish(m);
    ros::spinOnce();
}

// =============================================================================================
//                                                                                       pubHand
// =============================================================================================
void disneyDemo::waiting()
{   
    while(!go_)
    {       
        // std::cout<<"\033[32m\033[1mBUMP! \033[0m\n";
        ros::spinOnce();
    }
    go_ = false;
}




// =============================================================================================
//																				       publisher
// =============================================================================================
void disneyDemo::publisher(std::vector<double> v1, std::vector<double> v2)
{

	tf::Quaternion q1, q2;
	q1.setRPY(v1[0]*(M_PI/180),v1[1]*(M_PI/180),v1[2]*(M_PI/180));
	q2.setRPY(v2[0]*(M_PI/180),v2[1]*(M_PI/180),v2[2]*(M_PI/180));

	geometry_msgs::Pose local_msg1, local_msg2;
   local_msg1.position.x = v1[3];
   local_msg1.position.y = v1[4];
   local_msg1.position.z = v1[5];
   local_msg1.orientation.x  = q1.x();
   local_msg1.orientation.y =  q1.y();
   local_msg1.orientation.z  = q1.z();
   local_msg1.orientation.w  = q1.w();

   local_msg2.position.x = v2[3];
   local_msg2.position.y = v2[4];
   local_msg2.position.z = v2[5];
   local_msg2.orientation.x  = q2.x();
   local_msg2.orientation.y =  q2.y();
   local_msg2.orientation.z  = q2.z();
   local_msg2.orientation.w  = q2.w();


	pub1_.publish(local_msg1);
	pub2_.publish(local_msg2);
	ros::spinOnce();


	while(!check_ )
	{
    	// std::cout<<"\033[31m\033[1mMovement! \033[0m\n";
		ros::spinOnce();
	}

	check_  = false;
}



// =============================================================================================
//																				   checkMovement
// =============================================================================================
void disneyDemo::check(std_msgs::Bool bool_msg)
{	
	check_ = bool_msg.data;
}

// =============================================================================================
//																				   	  goMovement
// =============================================================================================
void disneyDemo::go(std_msgs::Bool m)
{	
	go_ = m.data;
}













// =============================================================================================
//							PER IL MOMENTO NON LE USO							  	   Utilities
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