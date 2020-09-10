#include "lipm_control/control.h"

control::control(ros::NodeHandle nh_)
{
    nh = nh_;
    ros::NodeHandle n_p("~");
    double g, comZ, dt;
    n_p.param<double>("gravity", g, 9.80665);
    n_p.param<double>("comZ", comZ, 0.26);
    n_p.param<double>("dt", dt, 0.01);
    n_p.param<double>("control_frequency", freq, 50);

    i = 0;
    trajectorySize = 0;
    desiredTrajectoryAvailable = false;
    as_ = new actionlib::SimpleActionServer<lipm_msgs::MotionControlAction>(nh, "lipm_control/plan", boost::bind(&control::desiredTrajectoryCb, this, _1), false);
    as_->start();
}
void control::desiredTrajectoryCb(const lipm_msgs::MotionControlGoalConstPtr &goal)
{
    CoMTrajectory =  goal->CoM;
    VRPTrajectory =  goal->VRP;
    DCMTrajectory =  goal->DCM;
    LLegTrajectory = goal->LLeg;
    RLegTrajectory = goal->RLeg;
    trajectorySize = CoMTrajectory.positions.size();
    desiredTrajectoryAvailable= true;
    feedback_.percent_completed = 100;
    as_->publishFeedback(feedback_);
    result_.status = 1;
    as_->setSucceeded(result_);
    std::cout<<"Message Received"<<std::endl;
}


void control::run()
{
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
                tmp = Vector3d(LLegTrajectory.positions[i].x, LLegTrajectory.positions[i].y, LLegTrajectory.positions[i].z);
                //wbc.setLLegLinearTask(tmp);
                tmp = Vector3d(0,0,0);
               // wbc.setLLegAngularTask(tmp);
                tmp = Vector3d(RLegTrajectory.positions[i].x, RLegTrajectory.positions[i].y, RLegTrajectory.positions[i].z);
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
        ros::spinOnce();
    }
}

void control::popFeedback()
{
    
}

control::~control()
{

}