#ifndef TELEOPERATION_CONTROLLER_MT_EFFORT_H
#define TELEOPERATION_CONTROLLER_MT_EFFORT_H


#include <lwr_controllers/KinematicChainControllerBase.h>
#include <lwr_controllers/PoseRPY.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>

#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>
#include <std_msgs/Int16.h>



namespace disney_demo
{
	class TeleoperationControllerMTEffort: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:
		TeleoperationControllerMTEffort();
		~TeleoperationControllerMTEffort();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const geometry_msgs::Pose::ConstPtr &msg);
		void command2(const geometry_msgs::Pose::ConstPtr &msg);
		void startControllerCallBack(const std_msgs::Bool::ConstPtr& msg);

	private:
		ros::Subscriber sub_command_, sub_command_2;
		ros::Subscriber sub_gains_, sub_first_time;

		KDL::Frame x_;		//current pose
		KDL::Frame x_des_, x_des_2;	//desired pose

		KDL::Twist x_err_;

		KDL::JntArray q_cmd_; // computed set points

		KDL::Jacobian J_;	//Jacobian

		Eigen::MatrixXd J_pinv_;
		Eigen::Matrix<double,3,3> skew_;
		double alpha1, alpha2, vel_sat;

		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_curr_, quat_des_;

		KDL::Vector v_temp_;
		
		int cmd_flag_;

		bool second_task;

		int cont;
		
		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;

		void firstTime (std_msgs::Int16 m); 

		ros::Publisher pub_error, pub_error2, pub_check, pub_vel;
		ros::Subscriber sub_start_controller;
		tf::TransformBroadcaster tf_desired_hand_pose, elbow_reference;
	};

}

#endif
