#include "lipm_control/control.h"

control::control(ros::NodeHandle nh_)
{
    nh = nh_;
    ros::NodeHandle n_p("~");
    double g, comZ, dt;
    n_p.param<double>("gravity", g, 9.80665);
    n_p.param<double>("comZ", comZ, 0.879533781657);
    n_p.param<double>("dt", dt, 0.02);
    n_p.param<double>("control_frequency", freq, 50);
    Twb = Eigen::Affine3d::Identity();
    i = 0;
    trajectorySize = 0;
    odom_sub = nh.subscribe("/floating_base_pose_simulated", 1, &control::odomCb, this);
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

void control::odomCb(const nav_msgs::OdometryConstPtr &msg)
{
    odom_inc = true;
    //Twb.translation() = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    //qwb = Eigen::Quaterniond(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);

    Twb.translation() = Vector3d(-0.0371361213341,0.000128050994496,0.990920581655);
    qwb =  Eigen::Quaterniond(1,0,0,0);
    Twb.linear() = qwb.toRotationMatrix();
}


void control::run()
{
    static ros::Rate rate(freq);
    
    while(ros::ok())
    {
        if(desiredTrajectoryAvailable && i < trajectorySize)
        {
            //if(CoMbuffer.size()>0 && ZMPBuffer.size()>0 && torsoBuffer.size()>0)
            if(odom_inc)
            {
                odom_inc = false;
                whole_body_ik_msgs::HumanoidGoal humanoidGoal_;
                humanoidGoal_.dt = 1.0/freq;
                
                Eigen::Vector3d temp = Eigen::Vector3d(CoMTrajectory.positions[i].x, CoMTrajectory.positions[i].y, CoMTrajectory.positions[i].z);
                temp = Twb.inverse() * temp;
                humanoidGoal_.CoM.linear_task.des.x = temp(0);
                humanoidGoal_.CoM.linear_task.des.y = temp(1);
                humanoidGoal_.CoM.linear_task.des.z = temp(2);
                // // humanoidGoal_.CoM.linear_task.des.x = 0.0259240577632;
                // // humanoidGoal_.CoM.linear_task.des.y = -0.000263675138099;
                // // humanoidGoal_.CoM.linear_task.des.z = -0.11631700406;
                std::cout<<"Goal CoM "<<temp<<std::endl;
                humanoidGoal_.CoM.linear_task.weight = 10;
                humanoidGoal_.CoM.linear_task.gain = 0.85;

                temp = Eigen::Vector3d(LLegTrajectory.positions[i].x, LLegTrajectory.positions[i].y, LLegTrajectory.positions[i].z);
                temp = Twb.inverse() * temp;
                humanoidGoal_.LLeg.linear_task.des.x = temp(0);
                humanoidGoal_.LLeg.linear_task.des.y = temp(1);
                humanoidGoal_.LLeg.linear_task.des.z = temp(2);
                // humanoidGoal_.LLeg.linear_task.des.x = 0.00842687758184;
                // humanoidGoal_.LLeg.linear_task.des.y =  0.0852573000938;
                std::cout<<"Goal LLeg "<<temp<<std::endl;

                humanoidGoal_.LLeg.linear_task.des.z = -0.991225844005;
                humanoidGoal_.LLeg.linear_task.weight = 1000.0;
                humanoidGoal_.LLeg.linear_task.gain = 0.85;

                humanoidGoal_.LLeg.angular_task.des.x = 0;
                humanoidGoal_.LLeg.angular_task.des.y = 0;
                humanoidGoal_.LLeg.angular_task.des.z = 0;
                humanoidGoal_.LLeg.angular_task.des.w = 1;
                humanoidGoal_.LLeg.angular_task.weight = 1000.0;
                humanoidGoal_.LLeg.angular_task.gain = 0.85;

                temp = Eigen::Vector3d(RLegTrajectory.positions[i].x, RLegTrajectory.positions[i].y, RLegTrajectory.positions[i].z);
                temp = Twb.inverse() * temp;
                humanoidGoal_.RLeg.linear_task.des.x = temp(0);
                humanoidGoal_.RLeg.linear_task.des.y = temp(1);
                humanoidGoal_.RLeg.linear_task.des.z = temp(2);
                std::cout<<"Goal RLeg "<<temp<<std::endl;

                // humanoidGoal_.RLeg.linear_task.des.x = 0.00842687758184;
                // humanoidGoal_.RLeg.linear_task.des.y = -0.0852573000938;
                humanoidGoal_.RLeg.linear_task.des.z = -0.991225844005;
                humanoidGoal_.RLeg.linear_task.weight = 1000.0;
                humanoidGoal_.RLeg.linear_task.gain = 0.85;

                humanoidGoal_.RLeg.angular_task.des.x = 0;
                humanoidGoal_.RLeg.angular_task.des.y = 0;
                humanoidGoal_.RLeg.angular_task.des.z = 0;
                humanoidGoal_.RLeg.angular_task.des.w = 1;
                humanoidGoal_.RLeg.angular_task.weight = 1000.0;
                humanoidGoal_.RLeg.angular_task.gain = 0.85;
                


                humanoidGoal_.Torso.angular_task.des.x = 0;
                humanoidGoal_.Torso.angular_task.des.y = 0;
                humanoidGoal_.Torso.angular_task.des.z = 0;
                humanoidGoal_.Torso.angular_task.des.w = 1;
                humanoidGoal_.Torso.angular_task.weight = 10;
                humanoidGoal_.Torso.angular_task.gain = 0.85;
                

                humanoidGoal_.Head.linear_task.des.x =  0.0201189;
                humanoidGoal_.Head.linear_task.des.y =  0;
                humanoidGoal_.Head.linear_task.des.z =  0.393193;
                humanoidGoal_.Head.linear_task.weight = 1;
                humanoidGoal_.Head.linear_task.gain = 0.85;    

                humanoidGoal_.Head.angular_task.des.x = 0;
                humanoidGoal_.Head.angular_task.des.y = 0;
                humanoidGoal_.Head.angular_task.des.z = 0;
                humanoidGoal_.Head.angular_task.des.w = 1;
                humanoidGoal_.Head.angular_task.weight = 1;
                humanoidGoal_.Head.angular_task.gain = 0.85;

                humanoidGoal_.RHand.linear_task.des.x =  0.201773;
                humanoidGoal_.RHand.linear_task.des.y =  -0.359338;
                humanoidGoal_.RHand.linear_task.des.z =  0.0106318;
                humanoidGoal_.RHand.linear_task.weight = 1;
                humanoidGoal_.RHand.linear_task.gain = 0.85;


                humanoidGoal_.RHand.angular_task.des.x = -0.0684384;
                humanoidGoal_.RHand.angular_task.des.y = -0.6058;
                humanoidGoal_.RHand.angular_task.des.z = 0.198244;
                humanoidGoal_.RHand.angular_task.des.w = 0.767477;
                humanoidGoal_.RHand.angular_task.weight = 1;
                humanoidGoal_.RHand.angular_task.gain = 0.5;

                humanoidGoal_.LHand.linear_task.des.x =  0.201774;
                humanoidGoal_.LHand.linear_task.des.y =  0.359338;
                humanoidGoal_.LHand.linear_task.des.z =  0.0106231;
                humanoidGoal_.LHand.linear_task.weight = 1;
                humanoidGoal_.LHand.linear_task.gain = 0.85;
   
                humanoidGoal_.LHand.angular_task.des.x = 0.0685307;
                humanoidGoal_.LHand.angular_task.des.y =  -0.60584;
                humanoidGoal_.LHand.angular_task.des.z =  -0.19819;
                humanoidGoal_.LHand.angular_task.des.w = 0.767451;
                humanoidGoal_.LHand.angular_task.weight = 1;
                humanoidGoal_.LHand.angular_task.gain = 0.85;

                ac_->sendGoal(humanoidGoal_);
                ac_->waitForResult();
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