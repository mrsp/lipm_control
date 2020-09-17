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
    ac_ = new actionlib::SimpleActionClient<whole_body_ik_msgs::HumanoidAction>("/talos/whole_body_control", true);
    ac_->waitForServer();
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
                whole_body_ik_msgs::HumanoidGoal humanoidGoal_;
                //humanoidGoal_.CoM.x = CoMTrajectory.velocities[i].x;
                //humanoidGoal_.CoM.y = CoMTrajectory.velocities[i].y;
                //humanoidGoal_.CoM.z = CoMTrajectory.velocities[i].z;
                humanoidGoal_.CoM.linear_task.des.x = 0;
                humanoidGoal_.CoM.linear_task.des.y = 0;
                humanoidGoal_.CoM.linear_task.des.z = 0;
                humanoidGoal_.CoM.linear_task.weight = 10.0;
                humanoidGoal_.CoM.linear_task.gain = 0.5;
                humanoidGoal_.dt = 1.0/freq;

                humanoidGoal_.LLeg.linear_task.des.x = 0;
                humanoidGoal_.LLeg.linear_task.des.y = 0;
                humanoidGoal_.LLeg.linear_task.des.z = 0;
                humanoidGoal_.LLeg.linear_task.weight = 1000.0;
                humanoidGoal_.LLeg.linear_task.gain = 0.5;

                humanoidGoal_.LLeg.angular_task.des.x = 0;
                humanoidGoal_.LLeg.angular_task.des.y = 0;
                humanoidGoal_.LLeg.angular_task.des.z = 0;
                humanoidGoal_.LLeg.angular_task.des.w = 1;
                humanoidGoal_.LLeg.angular_task.weight = 1000.0;
                humanoidGoal_.LLeg.angular_task.gain = 0.5;

                humanoidGoal_.RLeg.linear_task.des.x = 0;
                humanoidGoal_.RLeg.linear_task.des.y = 0;
                humanoidGoal_.RLeg.linear_task.des.z = 0;
                humanoidGoal_.RLeg.linear_task.weight = 1000.0;
                humanoidGoal_.RLeg.linear_task.gain = 0.5;

                humanoidGoal_.RLeg.angular_task.des.x = 0;
                humanoidGoal_.RLeg.angular_task.des.y = 0;
                humanoidGoal_.RLeg.angular_task.des.
                z = 0;
                humanoidGoal_.LLeg.angular_task.des.w = 1;
                humanoidGoal_.RLeg.angular_task.weight = 1000.0;
                humanoidGoal_.RLeg.angular_task.gain = 0.5;
                
                ac_->sendGoal(humanoidGoal_);
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