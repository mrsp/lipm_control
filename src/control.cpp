#include "lipm_control/control.h"

control::control(ros::NodeHandle nh_)
{
    nh = nh_;
    
    /*
    CoM_sub = nh.subscribe("lipm_msgs/CoM",10, &control::desiredCoMCb, this, ros::TransportHints().tcpNoDelay());
    DCM_sub = nh.subscribe("lipm_msgs/DCM",10, &control::desiredDCMCb, this, ros::TransportHints().tcpNoDelay());
    VRP_sub = nh.subscribe("lipm_msgs/VRP",10, &control::desiredVRPCb, this, ros::TransportHints().tcpNoDelay());
    LLeg_sub = nh.subscribe("lipm_msgs/LLeg",10, &control::desiredfootLCb, this, ros::TransportHints().tcpNoDelay());
    RLeg_sub = nh.subscribe("lipm_msgs/RLeg",10, &control::desiredfootRCb, this, ros::TransportHints().tcpNoDelay());
    */

    ros::NodeHandle n_p("~");
    double g, comZ, dt;
    n_p.param<double>("gravity", g, 9.80665);
    n_p.param<double>("comZ", comZ, 0.26);
    n_p.param<double>("dt", dt, 0.01);
    n_p.param<double>("control_frequency", freq, 100);

    i = 0;
    trajectorySize = 0;
    desiredTrajectoryAvailable = false;
}

void control::desiredTrajectoryCb(const lipm_msgs::TrajectoryPointsConstPtr &comd_msg,const  lipm_msgs::TrajectoryPointsConstPtr &vrpd_msg,const lipm_msgs::TrajectoryPointsConstPtr &dcmd_msg, 
        const lipm_msgs::TrajectoryPointsConstPtr &LLeg_msg,const  lipm_msgs::TrajectoryPointsConstPtr &RLeg_msg)
{
    CoMTrajectory = *comd_msg;
    VRPTrajectory = *vrpd_msg;
    DCMTrajectory = *dcmd_msg;
    LLegTrajectory = *LLeg_msg;
    RLegTrajectory = *RLeg_msg;
    trajectorySize = CoMTrajectory.positions.size();
    desiredTrajectoryAvailable= true;
}


void control::run()
{
    message_filters::Subscriber<lipm_msgs::TrajectoryPoints> CoMd_sub(nh,"lipm_motion/CoM",10);
    message_filters::Subscriber<lipm_msgs::TrajectoryPoints> DCMd_sub(nh,"lipm_motion/DCM",10);
    message_filters::Subscriber<lipm_msgs::TrajectoryPoints> VRPd_sub(nh,"lipm_motion/VRP",10);
    message_filters::Subscriber<lipm_msgs::TrajectoryPoints> LLegd_sub(nh,"lipm_motion/LLeg",10);
    message_filters::Subscriber<lipm_msgs::TrajectoryPoints> RLegd_sub(nh,"lipm_motion/RLeg",10);
    ts_sync = new message_filters::Synchronizer<syncPolicy>(syncPolicy(10), CoMd_sub, DCMd_sub, VRPd_sub, LLegd_sub, RLegd_sub);
    ts_sync->registerCallback(boost::bind(&control::desiredTrajectoryCb,this,_1,_2,_3,_4,_5));
    
    static ros::Rate rate(freq);
    
    while(ros::ok())
    {
        if(desiredTrajectoryAvailable && i < trajectorySize)
        {
            //if(CoMbuffer.size()>0 && ZMPBuffer.size()>0 && torsoBuffer.size()>0)
            if(1)
            {
                Vector3d tmp = Vector3d(CoMTrajectory.velocities[i].x, CoMTrajectory.velocities[i].y, CoMTrajectory.velocities[i].z);
                //wbc.setCoMLinearTask(tmp);
                tmp = Vector3d(0,0,0);
                //wbc.setTorsoAngularTask(tmp);
                tmp = Vector3d(LLegTrajectory.velocities[i].x, LLegTrajectory.velocities[i].y, LLegTrajectory.velocities[i].z);
                //wbc.setLLegLinearTask(tmp);
                tmp = Vector3d(0,0,0);
               // wbc.setLLegAngularTask(tmp);
                tmp = Vector3d(RLegTrajectory.velocities[i].x, RLegTrajectory.velocities[i].y, RLegTrajectory.velocities[i].z);
                //wbc.setRLegLinearTask(tmp);
                tmp = Vector3d(0,0,0);
                //.setRLegAngularTask(tmp);
                //wbc.ik();
                i++;
            }
        }
        else
        {
            i = 0;
            desiredTrajectoryAvailable = false;
        }
        popFeedback();
        rate.sleep();
    }
}

void control::popFeedback()
{
    
}

control::~control()
{

}