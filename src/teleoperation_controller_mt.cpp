
#include <disney_demo_controller_mt.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>
#include <utils/kuka_lwr_utilities.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>

#include <math.h>

#include <std_msgs/Float64MultiArray.h>
#include <tf_conversions/tf_kdl.h>

namespace disney_demo
{
TeleoperationControllerMT::TeleoperationControllerMT() {}
TeleoperationControllerMT::~TeleoperationControllerMT() {}

bool TeleoperationControllerMT::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
{
    if ( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
    {
        ROS_ERROR("Couldn't initilize OneTaskInverseKinematics controller.");
        return false;
    }

    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, joint_limits_.min, joint_limits_.max, *fk_pos_solver_, *ik_vel_solver_));

    q_cmd_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());

    // get joint positions
    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_states_.q(i) = joint_msr_states_.q(i);
    }

    // computing forward kinematics
    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_des_);

    //Desired posture is the current one
    x_des_ = x_;

    cmd_flag_ = 0;

    sub_command_ = nh_.subscribe("command1", 1, &TeleoperationControllerMT::command, this);
    sub_command_2 = nh_.subscribe("command2", 1, &TeleoperationControllerMT::command2, this);

    pub_error = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
    pub_error2 = nh_.advertise<std_msgs::Float64MultiArray>("error2", 1000);
    //sub_command_2 = nh_.subscribe("command2", 1, &TeleoperationControllerMT::command2, this);
    pub_check = nh_.advertise<std_msgs::Bool>("check", 1000);


    sub_start_controller = nh_.subscribe("start_controller", 1, &TeleoperationControllerMT::startControllerCallBack, this);
    nh_.param<double>("alpha1", alpha1, 1);
    nh_.param<double>("alpha2", alpha2, 1);

    ROS_INFO_STREAM("alpha2: " << alpha2);

    // for (unsigned i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
    // {
    //     ROS_INFO_STREAM("Segment name: " << kdl_chain_.getSegment(i).getName() << " Number " << i);
    // }

    second_task = false;
    return true;
}

void TeleoperationControllerMT::starting(const ros::Time& time)
{

}

void TeleoperationControllerMT::update(const ros::Time& time, const ros::Duration& period)
{
   nh_.param<double>("alpha1", alpha1, 1.0);
    nh_.param<double>("alpha2", alpha2, 1.0);
    // ROS_INFO_STREAM("Gains alpha1: " << alpha1 << "\t alpha2: " << alpha2);


    std_msgs::Float64MultiArray error_msg;
    std_msgs::Float64MultiArray error_msg2;
    KDL::Twist x_err_2;
    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    }

    if (cmd_flag_)
    {
        // computing Jacobian
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

        // computing J_pinv_
        pseudo_inverse(J_.data, J_pinv_);

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

        // end-effector position error

        x_err_.vel = x_des_.p - x_.p;
        x_.M.GetQuaternion(quat_curr_.v(0), quat_curr_.v(1), quat_curr_.v(2), quat_curr_.a);

        x_des_.M.GetQuaternion(quat_des_.v(0), quat_des_.v(1), quat_des_.v(2), quat_des_.a);

        skew_symmetric(quat_des_.v, skew_);

        for (int i = 0; i < skew_.rows(); i++)
        {
            v_temp_(i) = 0.0;
            for (int k = 0; k < skew_.cols(); k++)
                v_temp_(i) += skew_(i, k) * (quat_curr_.v(k));
        }

        // end-effector orientation error
        x_err_.rot = quat_curr_.a * quat_des_.v - quat_des_.a * quat_curr_.v - v_temp_;

        // computing q_dot


        /***************************************************/
        Eigen::MatrixXd epsilon_hat = skew_symmetric(quat_des_.v);
        Eigen::MatrixXd orientation_error = Eigen::MatrixXd::Zero(3, 1);
        double eta_d = quat_des_.a;
        double eta = quat_curr_.a;
        Eigen::MatrixXd actual_quaternion = Eigen::MatrixXd::Zero(3, 1);
        Eigen::MatrixXd desired_quaternion = Eigen::MatrixXd::Zero(3, 1);

        actual_quaternion << quat_curr_.v(0), quat_curr_.v(1), quat_curr_.v(2);
        desired_quaternion << quat_des_.v(0), quat_des_.v(1), quat_des_.v(2);

        orientation_error = eta_d * actual_quaternion - eta * desired_quaternion + epsilon_hat * actual_quaternion;

        KDL::Twist xerr;
        xerr.vel = x_des_.p - x_.p;
        xerr.rot = 0.5 * (  x_.M.UnitX() * x_des_.M.UnitX() +
                            x_.M.UnitY() * x_des_.M.UnitY() +
                            x_.M.UnitZ() * x_des_.M.UnitZ());


        Eigen::MatrixXd qp_des_attractive = Eigen::MatrixXd::Zero(7, 1);
        Eigen::MatrixXd x_err_temp_eigen = Eigen::MatrixXd::Zero(6, 1);
        x_err_temp_eigen << x_err_.vel.data[0],  x_err_.vel.data[1],  x_err_.vel.data[2],
                         orientation_error(0), orientation_error(1), orientation_error(2);
        qp_des_attractive = alpha1 * J_pinv_ * x_err_temp_eigen;


        // computing q_dot
        // for (int i = 0; i < J_pinv_.rows(); i++)
        // {
        //     joint_des_states_.qdot(i) = qp_des_attractive(i); //removed scaling factor of .7

        // }

        // ROS_INFO_STREAM("Error orientation quat: " << orientation_error.transpose());
        KDL::Twist x_err_temp;
        x_err_temp =  diff(x_, x_des_);
        // ROS_INFO_STREAM("Error orientation diff: "  << x_err_temp.rot.data[0] << " "
        //                 << x_err_temp.rot.data[1] << " "
        //                 << x_err_temp.rot.data[2]);
        // ROS_INFO_STREAM("Error orientation vectorts: "  << xerr.rot.data[0] << " "
        //                 << xerr.rot.data[1] << " "
        //                 << xerr.rot.data[2]);

        /****************************************/

        x_err_.rot = x_err_temp.rot;
        for (int i = 0; i < J_pinv_.rows(); i++)
        {
            joint_des_states_.qdot(i) = 0.0;
            for (int k = 0; k < J_pinv_.cols(); k++)
                joint_des_states_.qdot(i) += alpha1 * J_pinv_(i, k) * x_err_(k); //removed scaling factor of .7
        }



        KDL::Frame x_2;
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_2, 6);


        KDL::Jacobian J_2;
        J_2.resize(kdl_chain_.getNrOfJoints());
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_2, 6);

        // ROS_INFO_STREAM("Jac:" << std::endl << J_2.data);

        if (second_task)
        {
            Eigen::Matrix<double, 7, 7> P;
            P =  Eigen::Matrix<double, 7, 7>::Identity() - J_pinv_ * J_.data;

            Eigen::Matrix<double, 7, 1> q_null;
            Eigen::MatrixXd J_pinv_2;

            Eigen::Matrix<double, 3, 7> J_2_short = Eigen::Matrix<double, 3, 7>::Zero();
            J_2_short = J_2.data.block<3, 7>(0, 0);
            pseudo_inverse(J_2_short, J_pinv_2);
            Eigen::Matrix<double, 7, 3> NullSpace = Eigen::Matrix<double, 7, 3>::Zero();
            NullSpace = P * J_pinv_2;


            x_err_2.vel = x_des_2.p - x_2.p;

            Eigen::MatrixXd x_err_2_eigen = Eigen::MatrixXd::Zero(3, 1);
            x_err_2_eigen << x_err_2.vel(0), x_err_2.vel(1), x_err_2.vel(2);

            q_null = alpha2 * NullSpace * x_err_2_eigen; //removed scaling factor of .7

            // Eigen::Matrix<double, 7, 7> P;
            // P =  Eigen::Matrix<double, 7, 7>::Identity() - J_pinv_ * J_.data;

            // Eigen::Matrix<double, 7, 1> q_null;
            // Eigen::MatrixXd J_pinv_2;

            // Eigen::Matrix<double, 6, 7> J_2_short = Eigen::Matrix<double, 6, 7>::Zero();
            // J_2_short = J_2.data.block<6, 7>(0, 0);
            // pseudo_inverse(J_2_short, J_pinv_2);
            // Eigen::Matrix<double, 7, 6> NullSpace = Eigen::Matrix<double, 7, 6>::Zero();
            // NullSpace = P * J_pinv_2;


            // // x_err_2.vel = x_des_2.p - x_2.p;

            // x_err_2 = diff(x_2, x_des_2);

            // Eigen::MatrixXd x_err_2_eigen = Eigen::MatrixXd::Zero(6, 1);
            // x_err_2_eigen << x_err_2.vel(0), x_err_2.vel(1), x_err_2.vel(2), x_err_2.rot(0), x_err_2.rot(1), x_err_2.rot(2);

            // q_null = alpha2 * NullSpace * x_err_2_eigen; //removed scaling factor of .7


            for (int i = 0; i < J_pinv_2.rows(); i++)
            {

                joint_des_states_.qdot(i) += alpha2 * q_null[i]; //removed scaling factor of .7

            }



        }

        saturateJointVelocities(joint_des_states_.qdot);

        // integrating q_dot -> getting q (Euler method)
        for (unsigned int i = 0; i < joint_handles_.size(); i++)
            joint_des_states_.q(i) += period.toSec() * joint_des_states_.qdot(i);

        saturateJointPositions(joint_des_states_.q);
        // joint limits saturation
        // for (int i = 0;  i < joint_handles_.size(); i++)
        // {
        //     if (joint_des_states_.q(i) < joint_limits_.min(i))
        //         joint_des_states_.q(i) = joint_limits_.min(i);
        //     if (joint_des_states_.q(i) > joint_limits_.max(i))
        //         joint_des_states_.q(i) = joint_limits_.max(i);
        // }
        if (Equal(x_, x_des_, 0.1))
        {
            std_msgs::Bool bool_msg;                        
            bool_msg.data = true;
            pub_check.publish(bool_msg);
            ROS_INFO("On target 1");
            cmd_flag_ = 0;
        }


        // if (Equal(x_2, x_des_2, 0.03))
        // {
        //     ROS_INFO("On target 2");
        //     // cmd_flag_ = 0;
        // }
    }

    // set controls for joints
    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        joint_handles_[i].setCommand(joint_des_states_.q(i));
    }

    for (unsigned int i = 0; i < 6; ++i) {
        error_msg.data.push_back(x_err_(i));
        error_msg2.data.push_back(x_err_2(i));
    }



   
    pub_error.publish(error_msg);
    pub_error2.publish(error_msg2);


}

void TeleoperationControllerMT::command(const geometry_msgs::Pose::ConstPtr &msg)
{
    KDL::Frame frame_des_;


    frame_des_ = KDL::Frame(
                     KDL::Rotation::Quaternion(msg->orientation.x,
                             msg->orientation.y,
                             msg->orientation.z,
                             msg->orientation.w),
                     KDL::Vector(msg->position.x,
                                 msg->position.y,
                                 msg->position.z));

    x_des_ = frame_des_;

    tf::Transform CollisionTransform;
    tf::transformKDLToTF( frame_des_, CollisionTransform);
    tf_desired_hand_pose.sendTransform( tf::StampedTransform( CollisionTransform, ros::Time::now(), "vito_anchor", "reference") );
    cmd_flag_ = 1;
}

void TeleoperationControllerMT::command2(const geometry_msgs::Pose::ConstPtr &msg)
{
    KDL::Frame frame_des_;


    frame_des_ = KDL::Frame(
                     KDL::Rotation::Quaternion(msg->orientation.x,
                             msg->orientation.y,
                             msg->orientation.z,
                             msg->orientation.w),
                     KDL::Vector(msg->position.x,
                                 msg->position.y,
                                 msg->position.z));

    x_des_2 = frame_des_;
    second_task =  true;


    tf::Transform CollisionTransform2;
    tf::transformKDLToTF( frame_des_, CollisionTransform2);
    elbow_reference.sendTransform( tf::StampedTransform( CollisionTransform2, ros::Time::now(), "vito_anchor", "elbow_reference") );
}


void TeleoperationControllerMT::startControllerCallBack(const std_msgs::Bool::ConstPtr& msg)
{
    cmd_flag_ = msg->data;
    return;
}


}

PLUGINLIB_EXPORT_CLASS(disney_demo::TeleoperationControllerMT, controller_interface::ControllerBase)


