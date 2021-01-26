#include "lipm_control/control.h"

control::control(ros::NodeHandle nh_)
{
    nh = nh_;
    ros::NodeHandle n_p("~");
    double g, comZ, dt;
    n_p.param<double>("gravity", g, 9.80665);
    n_p.param<double>("comZ", comZ, 0.879533781657);
    n_p.param<double>("control_frequency", freq, 200);
    Twb = Eigen::Affine3d::Identity();
    i = 0;
    trajectorySize = 0;
    odom_sub = nh.subscribe("/floating_base_pose_simulated", 1000, &control::odomCb, this);
    desiredTrajectoryAvailable = false;
    as_ = new actionlib::SimpleActionServer<lipm_msgs::MotionControlAction>(nh, "lipm_control/plan", boost::bind(&control::desiredTrajectoryCb, this, _1), false);
    as_->start();
    ac_ = new actionlib::SimpleActionClient<whole_body_ik_msgs::HumanoidAction>("/talos/whole_body_control", true);
    ac_->waitForServer();
    init = false;
}
void control::desiredTrajectoryCb(const lipm_msgs::MotionControlGoalConstPtr &goal)
{
    CoMTrajectory = goal->CoM;
    VRPTrajectory = goal->VRP;
    DCMTrajectory = goal->DCM;
    LLegTrajectory = goal->LLeg;
    RLegTrajectory = goal->RLeg;
    trajectorySize = CoMTrajectory.positions.size();
    desiredTrajectoryAvailable = true;
    feedback_.percent_completed = 100;
    as_->publishFeedback(feedback_);
    result_.status = 1;
    as_->setSucceeded(result_);
    std::cout << "Message Received" << std::endl;
}

void control::odomCb(const nav_msgs::OdometryConstPtr &msg)
{
    odom_inc = true;
    Twb.translation() = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    qwb = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    //Twb.translation() = Vector3d(0.0130706968762,0.000128050994496,1.08588963803);
    //qwb =  Eigen::Quaterniond(1,0,0,0);
    Twb.linear() = qwb.toRotationMatrix();
}

void control::run()
{
    static ros::Rate rate(freq);

    while (ros::ok())
    {
         if (i==50)
         {
            
                odom_inc = false;
                whole_body_ik_msgs::HumanoidGoal humanoidGoal_;
                humanoidGoal_.dt = 1.5;

                humanoidGoal_.Joints.resize(30);


                //Head
                humanoidGoal_.Joints[0].dof_task.des = 0.0;
                humanoidGoal_.Joints[0].dof_task.weight = 0.1;
                humanoidGoal_.Joints[0].dof_task.gain = 1.0;
                humanoidGoal_.Joints[0].dof_task.name = "head_2_joint";

                humanoidGoal_.Joints[1].dof_task.des = 0.0;
                humanoidGoal_.Joints[1].dof_task.weight = 0.1;
                humanoidGoal_.Joints[1].dof_task.gain = 1.0;
                humanoidGoal_.Joints[1].dof_task.name = "head_1_joint";

                //Torso
                humanoidGoal_.Joints[2].dof_task.des = 9.630276567307305e-07;
                humanoidGoal_.Joints[2].dof_task.weight = 0.1;
                humanoidGoal_.Joints[2].dof_task.gain = 1.0;
                humanoidGoal_.Joints[2].dof_task.name = "torso_1_joint";

                humanoidGoal_.Joints[3].dof_task.des = 0.00025052828388183457;
                humanoidGoal_.Joints[3].dof_task.weight = 0.1;
                humanoidGoal_.Joints[3].dof_task.gain = 1.0;
                humanoidGoal_.Joints[3].dof_task.name = "torso_2_joint";

                //Left Arm
                humanoidGoal_.Joints[4].dof_task.des = 0.299975229388715;
                humanoidGoal_.Joints[4].dof_task.weight = 0.1;
                humanoidGoal_.Joints[4].dof_task.gain = 1.0;
                humanoidGoal_.Joints[4].dof_task.name = "arm_left_1_joint";

                humanoidGoal_.Joints[5].dof_task.des = 0.39996282399174987;
                humanoidGoal_.Joints[5].dof_task.weight = 0.1;
                humanoidGoal_.Joints[5].dof_task.gain = 1.0;
                humanoidGoal_.Joints[5].dof_task.name = "arm_left_2_joint";

                humanoidGoal_.Joints[6].dof_task.des = -0.5000333212143211;
                humanoidGoal_.Joints[6].dof_task.weight = 0.1;
                humanoidGoal_.Joints[6].dof_task.gain = 1.0;
                humanoidGoal_.Joints[6].dof_task.name = "arm_left_3_joint";

                humanoidGoal_.Joints[7].dof_task.des = -1.5000561487005104;
                humanoidGoal_.Joints[7].dof_task.weight = 0.1;
                humanoidGoal_.Joints[7].dof_task.gain = 1.0;
                humanoidGoal_.Joints[7].dof_task.name = "arm_left_4_joint";

                humanoidGoal_.Joints[8].dof_task.des = -0.004710635566556931;
                humanoidGoal_.Joints[8].dof_task.weight = 0.1;
                humanoidGoal_.Joints[8].dof_task.gain = 1.0;
                humanoidGoal_.Joints[8].dof_task.name = "arm_left_5_joint";

                humanoidGoal_.Joints[9].dof_task.des = 0.0008074396976889275;
                humanoidGoal_.Joints[9].dof_task.weight = 0.1;
                humanoidGoal_.Joints[9].dof_task.gain = 1.0;
                humanoidGoal_.Joints[9].dof_task.name = "arm_left_6_joint";


                humanoidGoal_.Joints[10].dof_task.des = 0.00033309628302102823;
                humanoidGoal_.Joints[10].dof_task.weight = 0.1;
                humanoidGoal_.Joints[10].dof_task.gain = 1.0;
                humanoidGoal_.Joints[10].dof_task.name = "arm_left_7_joint";



                //Right Arm
                humanoidGoal_.Joints[11].dof_task.des = -0.2999902314497742;
                humanoidGoal_.Joints[11].dof_task.weight = 0.1;
                humanoidGoal_.Joints[11].dof_task.gain = 1.0;
                humanoidGoal_.Joints[11].dof_task.name = "arm_right_1_joint";

                humanoidGoal_.Joints[12].dof_task.des = -0.39996282399174987;
                humanoidGoal_.Joints[12].dof_task.weight = 0.1;
                humanoidGoal_.Joints[12].dof_task.gain = 1.0;
                humanoidGoal_.Joints[12].dof_task.name = "arm_right_2_joint";

                humanoidGoal_.Joints[13].dof_task.des = 0.5000333212143211;
                humanoidGoal_.Joints[13].dof_task.weight = 0.1;
                humanoidGoal_.Joints[13].dof_task.gain = 1.0;
                humanoidGoal_.Joints[13].dof_task.name = "arm_right_3_joint";

                humanoidGoal_.Joints[14].dof_task.des = -1.5000561487005104;
                humanoidGoal_.Joints[14].dof_task.weight = 0.1;
                humanoidGoal_.Joints[14].dof_task.gain = 1.0;
                humanoidGoal_.Joints[14].dof_task.name = "arm_right_4_joint";

                humanoidGoal_.Joints[15].dof_task.des = 0.004710635566556931;
                humanoidGoal_.Joints[15].dof_task.weight = 0.1;
                humanoidGoal_.Joints[15].dof_task.gain = 1.0;
                humanoidGoal_.Joints[15].dof_task.name = "arm_right_5_joint";

                humanoidGoal_.Joints[16].dof_task.des = -0.0008074396976889275;
                humanoidGoal_.Joints[16].dof_task.weight = 0.1;
                humanoidGoal_.Joints[16].dof_task.gain = 1.0;
                humanoidGoal_.Joints[16].dof_task.name = "arm_right_6_joint";

                humanoidGoal_.Joints[17].dof_task.des = 0.00033309628302102823;
                humanoidGoal_.Joints[17].dof_task.weight = 0.1;
                humanoidGoal_.Joints[17].dof_task.gain = 1.0;
                humanoidGoal_.Joints[17].dof_task.name = "arm_right_7_joint";

                //Left Leg
                humanoidGoal_.Joints[18].dof_task.des = -8.124331024372822e-05;
                humanoidGoal_.Joints[18].dof_task.weight = 0.1;
                humanoidGoal_.Joints[18].dof_task.gain = 1.0;
                humanoidGoal_.Joints[18].dof_task.name = "leg_left_1_joint";

                humanoidGoal_.Joints[19].dof_task.des = -0.000831040487059731;
                humanoidGoal_.Joints[19].dof_task.weight = 0.1;
                humanoidGoal_.Joints[19].dof_task.gain = 1.0;
                humanoidGoal_.Joints[19].dof_task.name = "leg_left_2_joint";

                humanoidGoal_.Joints[20].dof_task.des = -0.5194120606620993;
                humanoidGoal_.Joints[20].dof_task.weight = 0.1;
                humanoidGoal_.Joints[20].dof_task.gain = 1.0;
                humanoidGoal_.Joints[20].dof_task.name = "leg_left_3_joint";

                humanoidGoal_.Joints[21].dof_task.des = 1.0210774019643543;
                humanoidGoal_.Joints[21].dof_task.weight = 0.1;
                humanoidGoal_.Joints[21].dof_task.gain = 1.0;
                humanoidGoal_.Joints[21].dof_task.name = "leg_left_4_joint";

                humanoidGoal_.Joints[22].dof_task.des = -0.50147038258197;
                humanoidGoal_.Joints[22].dof_task.weight = 0.1;
                humanoidGoal_.Joints[22].dof_task.gain = 1.0;
                humanoidGoal_.Joints[22].dof_task.name = "leg_left_5_joint";

                humanoidGoal_.Joints[23].dof_task.des = 0.00083149167042329;
                humanoidGoal_.Joints[23].dof_task.weight = 0.1;
                humanoidGoal_.Joints[23].dof_task.gain = 1.0;
                humanoidGoal_.Joints[23].dof_task.name = "leg_left_6_joint";

                //Right Leg
                humanoidGoal_.Joints[24].dof_task.des = 8.124331024372822e-05;
                humanoidGoal_.Joints[24].dof_task.weight = 0.1;
                humanoidGoal_.Joints[24].dof_task.gain = 1.0;
                humanoidGoal_.Joints[24].dof_task.name = "leg_right_1_joint";

                humanoidGoal_.Joints[25].dof_task.des =  0.000831040487059731;
                humanoidGoal_.Joints[25].dof_task.weight = 0.1;
                humanoidGoal_.Joints[25].dof_task.gain = 1.0;
                humanoidGoal_.Joints[25].dof_task.name = "leg_right_2_joint";

                humanoidGoal_.Joints[26].dof_task.des = -0.5194120606620993;
                humanoidGoal_.Joints[26].dof_task.weight = 0.1;
                humanoidGoal_.Joints[26].dof_task.gain = 1.0;
                humanoidGoal_.Joints[26].dof_task.name = "leg_right_3_joint";

                humanoidGoal_.Joints[27].dof_task.des = 1.0210774019643543;
                humanoidGoal_.Joints[27].dof_task.weight = 0.1;
                humanoidGoal_.Joints[27].dof_task.gain = 1.0;
                humanoidGoal_.Joints[27].dof_task.name = "leg_right_4_joint";

                humanoidGoal_.Joints[28].dof_task.des = -0.50147038258197;
                humanoidGoal_.Joints[28].dof_task.weight = 0.1;
                humanoidGoal_.Joints[28].dof_task.gain = 1.0;
                humanoidGoal_.Joints[28].dof_task.name = "leg_right_5_joint";

                humanoidGoal_.Joints[29].dof_task.des = -0.00083149167042329;
                humanoidGoal_.Joints[29].dof_task.weight = 0.1;
                humanoidGoal_.Joints[29].dof_task.gain = 1.0;
                humanoidGoal_.Joints[29].dof_task.name = "leg_right_6_joint";




                //Eigen::Vector3d temp = Eigen::Vector3d(0.0259259176517, 0.0, 0.879533781657);
                // temp = Twb.inverse() * temp;
                // humanoidGoal_.CoM.linear_task.des.x = temp(0);
                // humanoidGoal_.CoM.linear_task.des.y = temp(1);
                // humanoidGoal_.CoM.linear_task.des.z = temp(2);
                // humanoidGoal_.CoM.linear_task.weight = 0.1;
                // humanoidGoal_.CoM.linear_task.gain = 0.5;

                // temp = Eigen::Vector3d(0, 0.0851, 0);
                // temp = Twb.inverse() * temp;
                
                //Transform to Local Base
                // humanoidGoal_.LLeg.linear_task.des.x = 0.010;
                // humanoidGoal_.LLeg.linear_task.des.y = 0.0844;
                // humanoidGoal_.LLeg.linear_task.des.z = -0.784;
                // humanoidGoal_.LLeg.linear_task.weight = 10.0;
                // humanoidGoal_.LLeg.linear_task.gain = 0.75;

                // humanoidGoal_.LLeg.angular_task.des.x = 0;
                // humanoidGoal_.LLeg.angular_task.des.y = 0;
                // humanoidGoal_.LLeg.angular_task.des.z = 0;
                // humanoidGoal_.LLeg.angular_task.des.w = 1;
                // humanoidGoal_.LLeg.angular_task.weight = 10.0;
                // humanoidGoal_.LLeg.angular_task.gain = 0.75;

                //temp = Eigen::Vector3d(0, -0.0851, 0);
                
                //Transform to Local Base
                //temp = Twb.inverse() * temp;
                // humanoidGoal_.RLeg.linear_task.des.x = 0.010;
                // humanoidGoal_.RLeg.linear_task.des.y = -0.0844;
                // humanoidGoal_.RLeg.linear_task.des.z = -0.784;
                // humanoidGoal_.RLeg.linear_task.weight = 10.0;
                // humanoidGoal_.RLeg.linear_task.gain = 0.75;

                // humanoidGoal_.RLeg.angular_task.des.x = 0;
                // humanoidGoal_.RLeg.angular_task.des.y = 0;
                // humanoidGoal_.RLeg.angular_task.des.z = 0;
                // humanoidGoal_.RLeg.angular_task.des.w = 1;
                // humanoidGoal_.RLeg.angular_task.weight = 10.0;
                // humanoidGoal_.RLeg.angular_task.gain = 0.75;


                // humanoidGoal_.Torso.linear_task.des.x = 0;
                // humanoidGoal_.Torso.linear_task.des.y = 0;
                // humanoidGoal_.Torso.linear_task.des.z =  0;
                // humanoidGoal_.Torso.linear_task.weight = 1.0e-2;
                // humanoidGoal_.Torso.linear_task.gain = 0.5;
                
                // humanoidGoal_.Torso.angular_task.des.x = 0;
                // humanoidGoal_.Torso.angular_task.des.y = 0;
                // humanoidGoal_.Torso.angular_task.des.z = 0;
                // humanoidGoal_.Torso.angular_task.des.w = 1;
                // humanoidGoal_.Torso.angular_task.weight = 1.0e-1;
                // humanoidGoal_.Torso.angular_task.gain = 0.5;

                // humanoidGoal_.Head.linear_task.des.x = 0.0201189;
                // humanoidGoal_.Head.linear_task.des.y = 0;
                // humanoidGoal_.Head.linear_task.des.z = 0.393193;
                // humanoidGoal_.Head.linear_task.weight = 1.0e-2;
                // humanoidGoal_.Head.linear_task.gain = 0.5;

                // humanoidGoal_.Head.angular_task.des.x = 0;
                // humanoidGoal_.Head.angular_task.des.y = 0;
                // humanoidGoal_.Head.angular_task.des.z = 0.423;
                // humanoidGoal_.Head.angular_task.des.w = 0.906;
                // humanoidGoal_.Head.angular_task.weight = 1.0e-2;
                // humanoidGoal_.Head.angular_task.gain = 0.5;

                // humanoidGoal_.RHand.linear_task.des.x = 0.202;
                // humanoidGoal_.RHand.linear_task.des.y = -0.359;
                // humanoidGoal_.RHand.linear_task.des.z = 0.011;
                // humanoidGoal_.RHand.linear_task.weight = 1.0e-1;
                // humanoidGoal_.RHand.linear_task.gain = 0.5;

                // humanoidGoal_.RHand.angular_task.des.x = -0.069;
                // humanoidGoal_.RHand.angular_task.des.y = -0.607;
                // humanoidGoal_.RHand.angular_task.des.z = 0.198;
                // humanoidGoal_.RHand.angular_task.des.w = 0.766;
                // humanoidGoal_.RHand.angular_task.weight = 1.0e-1;
                // humanoidGoal_.RHand.angular_task.gain = 0.5;

                // humanoidGoal_.LHand.linear_task.des.x = 0.0202;
                // humanoidGoal_.LHand.linear_task.des.y = 0.359;
                // humanoidGoal_.LHand.linear_task.des.z = 0.011 ;
                // humanoidGoal_.LHand.linear_task.weight = 1.0e-1;
                // humanoidGoal_.LHand.linear_task.gain = 0.5;

                // humanoidGoal_.LHand.angular_task.des.x = 0.069;
                // humanoidGoal_.LHand.angular_task.des.y = -0.607;
                // humanoidGoal_.LHand.angular_task.des.z = -0.198;
                // humanoidGoal_.LHand.angular_task.des.w = 0.766;
                // humanoidGoal_.LHand.angular_task.weight = 1.0e-1;
                // humanoidGoal_.LHand.angular_task.gain = 0.5;

                ac_->sendGoal(humanoidGoal_);
                ac_->waitForResult();
           
        }
        i++;

        // else
        // {
        //     init = true;
        //     i = 0;
        // }
        popFeedback();
        rate.sleep();
        ros::spinOnce();
    }

    // if(desiredTrajectoryAvailable && i < trajectorySize)
    // {
    //     if(odom_inc)
    //     {
    //         odom_inc = false;
    //         whole_body_ik_msgs::HumanoidGoal humanoidGoal_;
    //         humanoidGoal_.dt = 1.0/freq;

    //         Eigen::Vector3d temp = Eigen::Vector3d(CoMTrajectory.positions[i].x, CoMTrajectory.positions[i].y, CoMTrajectory.positions[i].z);
    //         temp = Twb.inverse() * temp;
    //         humanoidGoal_.CoM.linear_task.des.x = temp(0);
    //         humanoidGoal_.CoM.linear_task.des.y = temp(1);
    //         humanoidGoal_.CoM.linear_task.des.z = temp(2);
    //         // // humanoidGoal_.CoM.linear_task.des.x = 0.0259240577632;
    //         // // humanoidGoal_.CoM.linear_task.des.y = -0.000263675138099;
    //         // // humanoidGoal_.CoM.linear_task.des.z = -0.11631700406;
    //         humanoidGoal_.CoM.linear_task.weight = 10;
    //         humanoidGoal_.CoM.linear_task.gain = 0.85;

    //         temp = Eigen::Vector3d(LLegTrajectory.positions[i].x, LLegTrajectory.positions[i].y, LLegTrajectory.positions[i].z);
    //         temp = Twb.inverse() * temp;
    //         //Transform to Local Base
    //         humanoidGoal_.LLeg.linear_task.des.x = temp(0);
    //         humanoidGoal_.LLeg.linear_task.des.y = temp(1);
    //         humanoidGoal_.LLeg.linear_task.des.z = temp(2);
    //         humanoidGoal_.LLeg.linear_task.weight = 1000.0;
    //         humanoidGoal_.LLeg.linear_task.gain = 0.85;

    //         humanoidGoal_.LLeg.angular_task.des.x = 0;
    //         humanoidGoal_.LLeg.angular_task.des.y = 0;
    //         humanoidGoal_.LLeg.angular_task.des.z = 0;
    //         humanoidGoal_.LLeg.angular_task.des.w = 1;
    //         humanoidGoal_.LLeg.angular_task.weight = 1000.0;
    //         humanoidGoal_.LLeg.angular_task.gain = 0.85;

    //         temp = Eigen::Vector3d(RLegTrajectory.positions[i].x, RLegTrajectory.positions[i].y, RLegTrajectory.positions[i].z);
    //         //Transform to Local Base
    //         temp = Twb.inverse() * temp;
    //         humanoidGoal_.RLeg.linear_task.des.x = temp(0);
    //         humanoidGoal_.RLeg.linear_task.des.y = temp(1);
    //         humanoidGoal_.RLeg.linear_task.des.z = temp(2);
    //         humanoidGoal_.RLeg.linear_task.weight = 1000.0;
    //         humanoidGoal_.RLeg.linear_task.gain = 0.85;

    //         humanoidGoal_.RLeg.angular_task.des.x = 0;
    //         humanoidGoal_.RLeg.angular_task.des.y = 0;
    //         humanoidGoal_.RLeg.angular_task.des.z = 0;
    //         humanoidGoal_.RLeg.angular_task.des.w = 1;
    //         humanoidGoal_.RLeg.angular_task.weight = 1000.0;
    //         humanoidGoal_.RLeg.angular_task.gain = 0.85;

    //         humanoidGoal_.Torso.angular_task.des.x = 0;
    //         humanoidGoal_.Torso.angular_task.des.y = 0;
    //         humanoidGoal_.Torso.angular_task.des.z = 0;
    //         humanoidGoal_.Torso.angular_task.des.w = 1;
    //         humanoidGoal_.Torso.angular_task.weight = 10;
    //         humanoidGoal_.Torso.angular_task.gain = 0.85;

    //         humanoidGoal_.Head.linear_task.des.x =  0.0201189;
    //         humanoidGoal_.Head.linear_task.des.y =  0;
    //         humanoidGoal_.Head.linear_task.des.z =  0.393193;
    //         humanoidGoal_.Head.linear_task.weight = 1;
    //         humanoidGoal_.Head.linear_task.gain = 0.85;

    //         humanoidGoal_.Head.angular_task.des.x = 0;
    //         humanoidGoal_.Head.angular_task.des.y = 0;
    //         humanoidGoal_.Head.angular_task.des.z = 0;
    //         humanoidGoal_.Head.angular_task.des.w = 1;
    //         humanoidGoal_.Head.angular_task.weight = 1;
    //         humanoidGoal_.Head.angular_task.gain = 0.85;

    //         humanoidGoal_.RHand.linear_task.des.x =  0.201773;
    //         humanoidGoal_.RHand.linear_task.des.y =  -0.359338;
    //         humanoidGoal_.RHand.linear_task.des.z =  0.0106318;
    //         humanoidGoal_.RHand.linear_task.weight = 1;
    //         humanoidGoal_.RHand.linear_task.gain = 0.85;

    //         humanoidGoal_.RHand.angular_task.des.x = -0.0684384;
    //         humanoidGoal_.RHand.angular_task.des.y = -0.6058;
    //         humanoidGoal_.RHand.angular_task.des.z = 0.198244;
    //         humanoidGoal_.RHand.angular_task.des.w = 0.767477;
    //         humanoidGoal_.RHand.angular_task.weight = 1;
    //         humanoidGoal_.RHand.angular_task.gain = 0.5;

    //         humanoidGoal_.LHand.linear_task.des.x =  0.201774;
    //         humanoidGoal_.LHand.linear_task.des.y =  0.359338;
    //         humanoidGoal_.LHand.linear_task.des.z =  0.0106231;
    //         humanoidGoal_.LHand.linear_task.weight = 1;
    //         humanoidGoal_.LHand.linear_task.gain = 0.85;

    //         humanoidGoal_.LHand.angular_task.des.x = 0.0685307;
    //         humanoidGoal_.LHand.angular_task.des.y =  -0.60584;
    //         humanoidGoal_.LHand.angular_task.des.z =  -0.19819;
    //         humanoidGoal_.LHand.angular_task.des.w = 0.767451;
    //         humanoidGoal_.LHand.angular_task.weight = 1;
    //         humanoidGoal_.LHand.angular_task.gain = 0.85;

    //         ac_->sendGoal(humanoidGoal_);
    //         ac_->waitForResult();
    //         i++;
    //     }
    // }
    // else
    // {
    //     i = 0;
    //     desiredTrajectoryAvailable = false;
    // }
    //    popFeedback();
    //    rate.sleep();
    //    ros::spinOnce();
    //}
}

void control::popFeedback()
{
}

control::~control()
{
}