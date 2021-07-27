#ifndef __LIPMCONTROL_H__
#define __LIPMCONTROL_H__
#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <lipm_msgs/TrajectoryPoints.h>
#include <lipm_msgs/MotionControlAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <whole_body_ik_msgs/HumanoidAction.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include "lipm_control/Queue.h"
#include <whole_body_ik/nao_wbc.h>
#include <geometry_msgs/PointStamped.h>
#include <lipm_control/LIPMControl.h>

using namespace Eigen;
using namespace std;

class control
{
private:
    /// ROS nodehanlder
    ros::NodeHandle nh;
    double freq;
    int trajectorySize, i;
    bool desiredTrajectoryAvailable;
    Quaterniond q;
    lipm_msgs::TrajectoryPoints CoMTrajectory, VRPTrajectory, DCMTrajectory, LLegTrajectory, RLegTrajectory;
    lipm_msgs::MotionControlResult result_;
    lipm_msgs::MotionControlFeedback feedback_;
    bool init;
    Eigen::Affine3d Twb;
    Eigen::Quaterniond qwb;
    Eigen::Vector3d pwb, CoM, vCoM, vCoM_, aCoM, CoM_ref, vCoM_ref, aCoM_ref, vwb, omegawb, ZMP, lf_pos_ref, rf_pos_ref;
    Queue<sensor_msgs::JointStateConstPtr> joint_data;
	Queue<nav_msgs::OdometryConstPtr> odom_data, com_data;
	Queue<geometry_msgs::PointStampedConstPtr> zmp_data;
    LIPMControl *lc;  
    nav_msgs::Odometry odom_msg, com_msg;
    sensor_msgs::JointState joint_state_msg;
    VectorXd jointNominalConfig, jointVelocityTarget, qd;
    std::string com_topic, odom_topic, joint_state_topic, action_server_topic, zmp_topic;
    nao_wbc* nao_whole_body_control;
    bool firstCoMVel, eop;
    pin_wrapper *desired_pin;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    actionlib::SimpleActionServer<lipm_msgs::MotionControlAction> *as_; 
    actionlib::SimpleActionClient<whole_body_ik_msgs::HumanoidAction> *ac_;
    ros::Subscriber odom_sub, zmp_sub, com_sub, joint_state_sub;
    ~control();
    control(ros::NodeHandle nh_);
    void desiredTrajectoryCb(const lipm_msgs::MotionControlGoalConstPtr &goal);
    void popFeedback();
    void run();
    void odomCb(const nav_msgs::OdometryConstPtr &msg);
    void CoMCb(const nav_msgs::OdometryConstPtr &msg);
    void jointStateCb(const sensor_msgs::JointStateConstPtr &msg);
    void ZMPCb(const geometry_msgs::PointStampedConstPtr &msg);
    void joints(const sensor_msgs::JointStateConstPtr &msg);
    void odom(const nav_msgs::OdometryConstPtr &msg);
    void com(const nav_msgs::OdometryConstPtr &msg);
    void zmp(const geometry_msgs::PointStampedConstPtr &msg);

};
#endif
