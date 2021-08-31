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
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <lipm_control/LIPMControl.h>
#include <lipm_control/postureStabilizer.h>
#include <lipm_control/ZMPDistributor.h>
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
    lipm_msgs::TrajectoryPoints CoMTrajectory, VRPTrajectory, DCMTrajectory, LLegTrajectory, RLegTrajectory;
    lipm_msgs::MotionControlResult result_;
    lipm_msgs::MotionControlFeedback feedback_;
    bool init;
    Eigen::Affine3d Twb, Twl, Twr;
    Eigen::Quaterniond qwb, qwl, qwr, lf_orient_ref, rf_orient_ref, h_orient_ref, rh_orient_ref, lh_orient_ref;
    Eigen::Vector3d pwb, pwl, pwr, CoM, vCoM, vCoM_, aCoM, CoM_ref, vCoM_ref, aCoM_ref, vwb, omegawb, ZMP, lf_pos_ref, rf_pos_ref, LLeg_GRF, RLeg_GRF, LLeg_GRT, RLeg_GRT, ZMP_ref;
    Eigen::VectorXd q;
    Queue<sensor_msgs::JointStateConstPtr> joint_data;
    Queue<nav_msgs::OdometryConstPtr> odom_data, com_data, LLegodom_data, RLegodom_data;
    Queue<geometry_msgs::PointStampedConstPtr> zmp_data;
    Queue<geometry_msgs::WrenchStampedConstPtr> wrenchLLeg_data, wrenchRLeg_data;
    Queue<std_msgs::StringConstPtr> gait_phase_data;
    LIPMControl *lc;
    ZMPDistributor *zd;
    postureStabilizer *ps;
    nav_msgs::Odometry odom_msg, com_msg;
    sensor_msgs::JointState joint_state_msg;
    geometry_msgs::WrenchStamped wrenchLLeg_msg, wrenchRLeg_msg;
    VectorXd jointNominalConfig, jointNominalVelocity, qd, dqd;
    std::string com_topic, odom_topic, joint_state_topic, action_server_topic, zmp_topic, wrenchRLeg_topic, wrenchLLeg_topic, LLegodom_topic, RLegodom_topic, gait_phase_topic;
    nao_wbc *nao_whole_body_control;
    bool firstCoMVel, eop;
    double Gain;
    double QGain;
    whole_body_ik_msgs::HumanoidGoal humanoidGoal_;
    double dof_weight;
    double dof_gain;
    bool CoM_Ad_enabled;
    bool right_support, double_support, right_contact, left_contact;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    actionlib::SimpleActionServer<lipm_msgs::MotionControlAction> *as_;
    actionlib::SimpleActionClient<whole_body_ik_msgs::HumanoidAction> *ac_;
    ros::Subscriber odom_sub, zmp_sub, com_sub, joint_state_sub, wrenchLLeg_sub, wrenchRLeg_sub, LLegodom_sub,  RLegodom_sub, gait_phase_sub;
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
    void wrenchLLeg(const geometry_msgs::WrenchStampedConstPtr &msg);
    void wrenchRLeg(const geometry_msgs::WrenchStampedConstPtr &msg);
    void wrenchRLegCb(const geometry_msgs::WrenchStampedConstPtr &msg);
    void wrenchLLegCb(const geometry_msgs::WrenchStampedConstPtr &msg);
    void LLegodomCb(const nav_msgs::OdometryConstPtr &msg);
    void LLegodom(const nav_msgs::OdometryConstPtr &msg);
    void RLegodomCb(const nav_msgs::OdometryConstPtr &msg);
    void RLegodom(const nav_msgs::OdometryConstPtr &msg);
    void gaitPhase(const std_msgs::StringConstPtr &msg);
    void gaitPhaseCb(const std_msgs::StringConstPtr &msg);

};
#endif
