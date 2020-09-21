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
using namespace Eigen;


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
    bool odom_inc;
    Eigen::Affine3d Twb;
    Eigen::Quaterniond qwb;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    actionlib::SimpleActionServer<lipm_msgs::MotionControlAction> *as_; 
    actionlib::SimpleActionClient<whole_body_ik_msgs::HumanoidAction> *ac_;
    ros::Subscriber odom_sub;
    ~control();
    control(ros::NodeHandle nh_);
    void desiredTrajectoryCb(const lipm_msgs::MotionControlGoalConstPtr &goal);
    void popFeedback();
    void run();
    void odomCb(const nav_msgs::OdometryConstPtr &msg);
};
#endif
