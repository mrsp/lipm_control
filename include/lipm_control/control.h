#ifndef __LIPMCONTROL_H__
#define __LIPMCONTROL_H__
#include <ros/ros.h>
#include <iostream>
#include <lipm_msgs/TrajectoryPoints.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <eigen3/Eigen/Dense>

using namespace message_filters;
using namespace Eigen;

typedef message_filters::sync_policies::ApproximateTime<lipm_msgs::TrajectoryPoints, lipm_msgs::TrajectoryPoints, lipm_msgs::TrajectoryPoints, 
lipm_msgs::TrajectoryPoints, lipm_msgs::TrajectoryPoints> syncPolicy;


class control
{
private:
    /// ROS nodehanlder
    ros::NodeHandle nh;
    double freq;
    int trajectorySize, i;
    bool desiredTrajectoryAvailable;
    message_filters::Synchronizer<syncPolicy> *ts_sync;
    Quaterniond q;
    lipm_msgs::TrajectoryPoints CoMTrajectory, VRPTrajectory, DCMTrajectory, LLegTrajectory, RLegTrajectory;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~control();
    control(ros::NodeHandle nh_);
    void desiredTrajectoryCb(const lipm_msgs::TrajectoryPointsConstPtr &comd_msg,const  lipm_msgs::TrajectoryPointsConstPtr &vrpd_msg,const lipm_msgs::TrajectoryPointsConstPtr &dcmd_msg, 
        const lipm_msgs::TrajectoryPointsConstPtr &LLeg_msg,const  lipm_msgs::TrajectoryPointsConstPtr &RLeg_msg);
    void popFeedback();
    void run();
};
#endif
