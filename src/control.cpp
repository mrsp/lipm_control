#include "lipm_control/control.h"

control::control(ros::NodeHandle nh_)
{
    nh = nh_;
    ros::NodeHandle n_p("~");
    double g, comZ, dt;
    n_p.param<double>("gravity", g, 9.80665);
    n_p.param<double>("comZ", comZ, 0.879533781657);
    n_p.param<double>("control_frequency", freq, 100);
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
    Twb.linear() = qwb.toRotationMatrix();
}

void control::run()
{
    static ros::Rate rate(freq);

    while (ros::ok())
    {

        if (desiredTrajectoryAvailable && i < trajectorySize)
        {
            if (odom_inc)
            {
                std::cout << "Loop " << i << std::endl;
                odom_inc = false;
                whole_body_ik_msgs::HumanoidGoal humanoidGoal_;
                Eigen::Vector3d temp;
                Eigen::Quaterniond tempq;

                humanoidGoal_.dt = 1.0 / freq;

                temp = Eigen::Vector3d(CoMTrajectory.positions[i].x, CoMTrajectory.positions[i].y, CoMTrajectory.positions[i].z);
                humanoidGoal_.CoM.linear_task.desired_position.x = temp(0);
                humanoidGoal_.CoM.linear_task.desired_position.y = temp(1);
                humanoidGoal_.CoM.linear_task.desired_position.z = temp(2);
                temp = Eigen::Vector3d(CoMTrajectory.linear_velocities[i].x, CoMTrajectory.linear_velocities[i].y, CoMTrajectory.linear_velocities[i].z);
                humanoidGoal_.CoM.linear_task.desired_linear_velocity.x = temp(0);
                humanoidGoal_.CoM.linear_task.desired_linear_velocity.y = temp(1);
                humanoidGoal_.CoM.linear_task.desired_linear_velocity.z = temp(2);
                humanoidGoal_.CoM.linear_task.weight = 0.01;
                humanoidGoal_.CoM.linear_task.gain = 0.85;

                temp = Eigen::Vector3d(LLegTrajectory.positions[i].x, LLegTrajectory.positions[i].y, LLegTrajectory.positions[i].z);
                humanoidGoal_.LLeg.linear_task.desired_position.x = temp(0);
                humanoidGoal_.LLeg.linear_task.desired_position.y = temp(1);
                humanoidGoal_.LLeg.linear_task.desired_position.z = temp(2);
                humanoidGoal_.LLeg.linear_task.weight = 0.1;
                humanoidGoal_.LLeg.linear_task.gain = 0.85;
                temp = Eigen::Vector3d(LLegTrajectory.linear_velocities[i].x, LLegTrajectory.linear_velocities[i].y, LLegTrajectory.linear_velocities[i].z);
                humanoidGoal_.LLeg.linear_task.desired_linear_velocity.x = 0;
                humanoidGoal_.LLeg.linear_task.desired_linear_velocity.y = 0;
                humanoidGoal_.LLeg.linear_task.desired_linear_velocity.z = 0;

                //tempq = Eigen::Quaterniond(LLegTrajectory.orientations[i].w(),LLegTrajectory.orientations[i].x(), LLegTrajectory.orientations[i].y(), LLegTrajectory.orientations[i].z());
                // humanoidGoal_.LLeg.angular_task.desired_orientation.x = 0;
                // humanoidGoal_.LLeg.angular_task.desired_orientation.y = 0;
                // humanoidGoal_.LLeg.angular_task.desired_orientation.z = 0;
                // humanoidGoal_.LLeg.angular_task.desired_orientation.w = 1;
                // humanoidGoal_.LLeg.angular_task.weight = 1.0;
                // humanoidGoal_.LLeg.angular_task.gain = 0.85;
                // temp = Eigen::Vector3d(LLegTrajectory.angular_velocities[i].x, LLegTrajectory.angular_velocities[i].y, LLegTrajectory.angular_velocities[i].z);
                // humanoidGoal_.LLeg.angular_task.desired_angular_velocity.x = 0;
                // humanoidGoal_.LLeg.angular_task.desired_angular_velocity.y = 0;
                // humanoidGoal_.LLeg.angular_task.desired_angular_velocity.z = 0;

                //RLeg Desired Trajectories
                temp = Eigen::Vector3d(RLegTrajectory.positions[i].x, RLegTrajectory.positions[i].y, RLegTrajectory.positions[i].z);
                humanoidGoal_.RLeg.linear_task.desired_position.x = temp(0);
                humanoidGoal_.RLeg.linear_task.desired_position.y = temp(1);
                humanoidGoal_.RLeg.linear_task.desired_position.z = temp(2);
                humanoidGoal_.RLeg.linear_task.weight = 0.1;
                humanoidGoal_.RLeg.linear_task.gain = 0.85;
                temp = Eigen::Vector3d(RLegTrajectory.linear_velocities[i].x, RLegTrajectory.linear_velocities[i].y, RLegTrajectory.linear_velocities[i].z);
                humanoidGoal_.RLeg.linear_task.desired_linear_velocity.x = 0;
                humanoidGoal_.RLeg.linear_task.desired_linear_velocity.y = 0;
                humanoidGoal_.RLeg.linear_task.desired_linear_velocity.z = 0;

                // humanoidGoal_.RLeg.angular_task.desired_angular_velocity.x = 0;
                // humanoidGoal_.RLeg.angular_task.desired_angular_velocity.y = 0;
                // humanoidGoal_.RLeg.angular_task.desired_angular_velocity.z = 0;
                // humanoidGoal_.RLeg.angular_task.desired_orientation.x = 0;
                // humanoidGoal_.RLeg.angular_task.desired_orientation.y = 0;
                // humanoidGoal_.RLeg.angular_task.desired_orientation.z = 0;
                // humanoidGoal_.RLeg.angular_task.desired_orientation.w = 1;
                // humanoidGoal_.RLeg.angular_task.weight = 10.0;
                // humanoidGoal_.RLeg.angular_task.gain = 0.85;

                // humanoidGoal_.Torso.angular_task.des.x = 0;
                // humanoidGoal_.Torso.angular_task.des.y = 0;
                // humanoidGoal_.Torso.angular_task.des.z = 0;
                // humanoidGoal_.Torso.angular_task.des.w = 1;
                // humanoidGoal_.Torso.angular_task.weight = 1;
                // humanoidGoal_.Torso.angular_task.gain = 1.0;
                double dof_weight = 0.001;
                double dof_gain = 0.85;
                humanoidGoal_.Joints.resize(30);

                //HEAD Joints
                humanoidGoal_.Joints[0].desired_angle = -8.981051937695383e-07;
                humanoidGoal_.Joints[0].weight = dof_weight;
                humanoidGoal_.Joints[0].gain = dof_gain;
                humanoidGoal_.Joints[0].name = "head_2_joint";

                humanoidGoal_.Joints[1].desired_angle = 0.00041801880549030557;
                humanoidGoal_.Joints[1].name = "head_1_joint";
                humanoidGoal_.Joints[1].weight = dof_weight;
                humanoidGoal_.Joints[1].gain = dof_gain;
                //TORSO Joints
                humanoidGoal_.Joints[2].desired_angle = 9.630276567307305e-07;
                humanoidGoal_.Joints[2].weight = dof_weight;
                humanoidGoal_.Joints[2].gain = dof_gain;
                humanoidGoal_.Joints[2].name = "torso_1_joint";

                humanoidGoal_.Joints[3].desired_angle = 0.00025052828388183457;
                humanoidGoal_.Joints[3].name = "torso_2_joint";
                humanoidGoal_.Joints[3].weight = dof_weight;
                humanoidGoal_.Joints[3].gain = dof_gain;
                //LEFT ARM Joints
                humanoidGoal_.Joints[4].desired_angle = 0.299975229388715;
                humanoidGoal_.Joints[4].name = "arm_left_1_joint";
                humanoidGoal_.Joints[3].weight = dof_weight;
                humanoidGoal_.Joints[3].gain = dof_gain;

                humanoidGoal_.Joints[5].desired_angle = 0.39996282399174987;
                humanoidGoal_.Joints[5].name = "arm_left_2_joint";
                humanoidGoal_.Joints[5].weight = dof_weight;
                humanoidGoal_.Joints[5].gain = dof_gain;

                humanoidGoal_.Joints[6].desired_angle = -0.5000333212143211;
                humanoidGoal_.Joints[6].name = "arm_left_3_joint";
                humanoidGoal_.Joints[6].weight = dof_weight;
                humanoidGoal_.Joints[6].gain = dof_gain;

                humanoidGoal_.Joints[7].desired_angle = -1.5000561487005104;
                humanoidGoal_.Joints[7].name = "arm_left_4_joint";
                humanoidGoal_.Joints[7].weight = dof_weight;
                humanoidGoal_.Joints[7].gain = dof_gain;

                humanoidGoal_.Joints[8].desired_angle = -0.004710635566556931;
                humanoidGoal_.Joints[8].name = "arm_left_5_joint";
                humanoidGoal_.Joints[8].weight = dof_weight;
                humanoidGoal_.Joints[8].gain = dof_gain;

                humanoidGoal_.Joints[9].desired_angle = 0.0008074396976889275;
                humanoidGoal_.Joints[9].name = "arm_left_6_joint";
                humanoidGoal_.Joints[9].weight = dof_weight;
                humanoidGoal_.Joints[9].gain = dof_gain;

                humanoidGoal_.Joints[10].desired_angle = 0.00033309628302102823;
                humanoidGoal_.Joints[10].name = "arm_left_7_joint";
                humanoidGoal_.Joints[10].weight = dof_weight;
                humanoidGoal_.Joints[10].gain = dof_gain;

                //RIGHT ARM
                humanoidGoal_.Joints[11].desired_angle = -0.299975229388715;
                humanoidGoal_.Joints[11].name = "arm_right_1_joint";
                humanoidGoal_.Joints[11].weight = dof_weight;
                humanoidGoal_.Joints[11].gain = dof_gain;

                humanoidGoal_.Joints[12].desired_angle = -0.39996282399174987;
                humanoidGoal_.Joints[12].name = "arm_right_2_joint";
                humanoidGoal_.Joints[12].weight = dof_weight;
                humanoidGoal_.Joints[12].gain = dof_gain;

                humanoidGoal_.Joints[13].desired_angle = 0.5000333212143211;
                humanoidGoal_.Joints[13].name = "arm_right_3_joint";
                humanoidGoal_.Joints[13].weight = dof_weight;
                humanoidGoal_.Joints[13].gain = dof_gain;

                humanoidGoal_.Joints[14].desired_angle = -1.5000561487005104;
                humanoidGoal_.Joints[14].name = "arm_right_4_joint";
                humanoidGoal_.Joints[14].weight = dof_weight;
                humanoidGoal_.Joints[14].gain = dof_gain;

                humanoidGoal_.Joints[15].desired_angle = 0.004710635566556931;
                humanoidGoal_.Joints[15].name = "arm_right_5_joint";
                humanoidGoal_.Joints[15].weight = dof_weight;
                humanoidGoal_.Joints[15].gain = dof_gain;

                humanoidGoal_.Joints[16].desired_angle = -0.0008074396976889275;
                humanoidGoal_.Joints[16].name = "arm_right_6_joint";
                humanoidGoal_.Joints[16].weight = dof_weight;
                humanoidGoal_.Joints[16].gain = dof_gain;

                humanoidGoal_.Joints[17].desired_angle = 0.00033309628302102823;
                humanoidGoal_.Joints[17].name = "arm_right_7_joint";
                humanoidGoal_.Joints[17].weight = dof_weight;
                humanoidGoal_.Joints[17].gain = dof_gain;

                //LEFT LEG
                humanoidGoal_.Joints[18].desired_angle = -8.124331024372822e-05;
                humanoidGoal_.Joints[18].name = "leg_left_1_joint";
                humanoidGoal_.Joints[18].weight = dof_weight;
                humanoidGoal_.Joints[18].gain = dof_gain;

                humanoidGoal_.Joints[19].desired_angle = -0.000831040487059731;
                humanoidGoal_.Joints[19].name = "leg_left_2_joint";
                humanoidGoal_.Joints[19].weight = dof_weight;
                humanoidGoal_.Joints[19].gain = dof_gain;

                humanoidGoal_.Joints[20].desired_angle = -0.5194120606620993;
                humanoidGoal_.Joints[20].name = "leg_left_3_joint";
                humanoidGoal_.Joints[20].weight = dof_weight;
                humanoidGoal_.Joints[20].gain = dof_gain;

                humanoidGoal_.Joints[21].desired_angle = 1.0210774019643543;
                humanoidGoal_.Joints[21].name = "leg_left_4_joint";
                humanoidGoal_.Joints[21].weight = dof_weight;
                humanoidGoal_.Joints[21].gain = dof_gain;

                humanoidGoal_.Joints[22].desired_angle = -0.50147038258197;
                humanoidGoal_.Joints[22].name = "leg_left_5_joint";
                humanoidGoal_.Joints[22].weight = dof_weight;
                humanoidGoal_.Joints[22].gain = dof_gain;

                humanoidGoal_.Joints[23].desired_angle = 0.00083149167042329;
                humanoidGoal_.Joints[23].name = "leg_left_6_joint";
                humanoidGoal_.Joints[23].weight = dof_weight;
                humanoidGoal_.Joints[23].gain = dof_gain;

                //RIGHT LEG
                humanoidGoal_.Joints[24].desired_angle = 8.124331024372822e-05;
                humanoidGoal_.Joints[24].name = "leg_right_1_joint";
                humanoidGoal_.Joints[24].weight = dof_weight;
                humanoidGoal_.Joints[24].gain = dof_gain;

                humanoidGoal_.Joints[25].desired_angle = 0.000831040487059731;
                humanoidGoal_.Joints[25].name = "leg_right_2_joint";
                humanoidGoal_.Joints[25].weight = dof_weight;
                humanoidGoal_.Joints[25].gain = dof_gain;

                humanoidGoal_.Joints[26].desired_angle = -0.5194120606620993;
                humanoidGoal_.Joints[26].name = "leg_right_3_joint";
                humanoidGoal_.Joints[26].weight = dof_weight;
                humanoidGoal_.Joints[26].gain = dof_gain;

                humanoidGoal_.Joints[27].desired_angle = 1.0210774019643543;
                humanoidGoal_.Joints[27].name = "leg_right_4_joint";
                humanoidGoal_.Joints[27].weight = dof_weight;
                humanoidGoal_.Joints[27].gain = dof_gain;

                humanoidGoal_.Joints[28].desired_angle = -0.50147038258197;
                humanoidGoal_.Joints[28].name = "leg_right_5_joint";
                humanoidGoal_.Joints[28].weight = dof_weight;
                humanoidGoal_.Joints[28].gain = dof_gain;

                humanoidGoal_.Joints[29].desired_angle = -0.00083149167042329;
                humanoidGoal_.Joints[29].name = "leg_right_6_joint";
                humanoidGoal_.Joints[29].weight = dof_weight;
                humanoidGoal_.Joints[29].gain = dof_gain;

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