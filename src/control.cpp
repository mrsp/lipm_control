#include "lipm_control/control.h"
#include <chrono>

using namespace std::chrono;

control::control(ros::NodeHandle nh_)
{
    nh = nh_;
    ros::NodeHandle n_p("~");
    double g, hc, mass;
    double Pcom_x, Pcom_y, Pzmp_x, Pzmp_y, Pdcm_x, Pdcm_y, Idcm_x, Idcm_y;
    n_p.param<double>("gravity", g, 9.80665);
    n_p.param<double>("hc", hc, 0.268);
    n_p.param<double>("mass", mass, 5.14);

    n_p.param<double>("control_frequency", freq, 100);
    n_p.param<std::string>("joint_state_topic", joint_state_topic, "/nao_raisim_ros/joint_states");
    n_p.param<std::string>("com_topic", com_topic, "/nao_raisim_ros/CoM");
    n_p.param<std::string>("odom_topic", odom_topic, "/nao_raisim_ros/odom");
    n_p.param<std::string>("zmp_topic", zmp_topic, "/nao_raisim_ros/ZMP");
    n_p.param<std::string>("wrenchLLeg_topic", wrenchLLeg_topic, "/nao_raisim_ros/LLeg/force_torque_states");
    n_p.param<std::string>("wrenchRLeg_topic", wrenchRLeg_topic, "/nao_raisim_ros/RLeg/force_torque_states");
    n_p.param<std::string>("LLegodom_topic", LLegodom_topic, "/nao_raisim_ros/LLeg/odom");
    n_p.param<std::string>("RLegodom_topic", RLegodom_topic, "/nao_raisim_ros/RLeg/odom");
    n_p.param<std::string>("gait_phase_topic", gait_phase_topic, "/nao_raisim_ros/gait_phase");

    //Load the CoM Admittance Control Parameters
    n_p.param<double>("CoM_admittance_gain_x", Pcom_x, 0.01);
    n_p.param<double>("CoM_admittance_gain_y", Pcom_y, 0.01);
    n_p.param<double>("ZMP_proportional_gain_x", Pzmp_x, 2);
    n_p.param<double>("ZMP_proportional_gain_y", Pzmp_y, 2);
    n_p.param<double>("DCM_proportional_gain_x", Pdcm_x, 10);
    n_p.param<double>("DCM_proportional_gain_y", Pdcm_y, 10);
    n_p.param<double>("DCM_Integral_gain_x", Idcm_x, 2.5);
    n_p.param<double>("DCM_Integral_gain_y", Idcm_y, 2.5);
    n_p.param<bool>("enabled", CoM_Ad_enabled, false);

    //Load the WBC Parameters
    double CoM_task_weight, LLeg_task_weight, RLeg_task_weight, LHand_task_weight, RHand_task_weight, Head_task_weight, Torso_task_weight, DOF_weight, DOF_gain;
    n_p.param<double>("Linear_task_gain", Gain, 0.35);
    n_p.param<double>("Angular_task_gain", QGain, 0.15);
    n_p.param<double>("CoM_task_weight", CoM_task_weight, 1e-1);
    n_p.param<double>("LLeg_task_weight", LLeg_task_weight, 1.0);
    n_p.param<double>("RLeg_task_weight", RLeg_task_weight, 1.0);
    n_p.param<double>("Torso_task_weight", Torso_task_weight, 1e-3);
    n_p.param<double>("Head_task_weight", Head_task_weight, 5e-4);
    n_p.param<double>("LHand_task_weight", LHand_task_weight, 5e-4);
    n_p.param<double>("RHand_task_weight", RHand_task_weight, 5e-4);
    n_p.param<double>("DOF_weight", DOF_weight, 5e-5);
    n_p.param<double>("DOF_gain", DOF_gain, 0.8);

    //Humanoid Robot Task Goal
    humanoidGoal_.dt = 1.0 / freq;
    humanoidGoal_.CoM.linear_task.weight = CoM_task_weight;
    humanoidGoal_.CoM.linear_task.gain = Gain;
    humanoidGoal_.LLeg.linear_task.weight = LLeg_task_weight;
    humanoidGoal_.LLeg.linear_task.gain = Gain;
    humanoidGoal_.LLeg.angular_task.weight = LLeg_task_weight;
    humanoidGoal_.LLeg.angular_task.gain = QGain;
    humanoidGoal_.RLeg.linear_task.weight = RLeg_task_weight;
    humanoidGoal_.RLeg.linear_task.gain = Gain;
    humanoidGoal_.RLeg.angular_task.weight = RLeg_task_weight;
    humanoidGoal_.RLeg.angular_task.gain = QGain;
    humanoidGoal_.Torso.angular_task.weight = Torso_task_weight;
    humanoidGoal_.Torso.angular_task.gain = QGain;
    dof_weight = DOF_weight;
    dof_gain = DOF_gain;

    //CoM Admittance Control Module
    lc = new LIPMControl(hc, 1.0 / freq, Pcom_x, Pcom_y, Pzmp_x, Pzmp_y, Pdcm_x, Pdcm_y, Idcm_x, Idcm_y);
    zd = new ZMPDistributor(mass, g);

    double Tc = 0;
    double Kc = 0;
    double Ta = 0.01;
    double Ka = 0.0;
    double Tn = 0;
    double Kn = 0;
    ps = new postureStabilizer(1.0/freq, Kc, Tc, Ka, Ta, Kn, Tn);
    //NAO WBC Module
    nao_whole_body_control = new nao_wbc(nh);

    //LIPM Control Parameters
    Twb = Eigen::Affine3d::Identity();
    pwb.setZero();
    vwb.setZero();
    omegawb.setZero();
    i = 0;
    firstCoMVel = true;
    trajectorySize = 0;
    odom_sub = nh.subscribe(odom_topic, 1000, &control::odomCb, this);
    zmp_sub = nh.subscribe(zmp_topic, 1000, &control::ZMPCb, this);
    com_sub = nh.subscribe(com_topic, 1000, &control::CoMCb, this);
    
    wrenchLLeg_sub = nh.subscribe(wrenchLLeg_topic, 1000, &control::wrenchLLegCb, this);
    wrenchRLeg_sub = nh.subscribe(wrenchRLeg_topic, 1000, &control::wrenchRLegCb, this);
    
    LLegodom_sub = nh.subscribe(LLegodom_topic, 1000, &control::LLegodomCb, this);
    RLegodom_sub = nh.subscribe(RLegodom_topic, 1000, &control::RLegodomCb, this);
    gait_phase_sub = nh.subscribe(gait_phase_topic, 1000, &control::gaitPhaseCb, this);

    joint_state_sub = nh.subscribe(joint_state_topic, 1000, &control::jointStateCb, this);
    desiredTrajectoryAvailable = false;
    as_ = new actionlib::SimpleActionServer<lipm_msgs::MotionControlAction>(nh, "lipm_control/plan", boost::bind(&control::desiredTrajectoryCb, this, _1), false);
    as_->start();

    jointNominalConfig = nao_whole_body_control->jointNominalConfig;
    jointNominalVelocity = nao_whole_body_control->jointNominalVelocity;
    qd = jointNominalConfig;
    dqd = jointNominalVelocity;
    eop = false;
    right_support = false;
    double_support = false;
    right_contact = false;
    left_contact = false;
    cout << "LIPM Control Module Initialized " << endl;
}
void control::desiredTrajectoryCb(const lipm_msgs::MotionControlGoalConstPtr &goal)
{
    if (!desiredTrajectoryAvailable)
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
        std::cout << "New Motion Trajectory Received" << std::endl;
    }
    else
    {
        feedback_.percent_completed = 0;
        as_->publishFeedback(feedback_);
        result_.status = 0;
        as_->setSucceeded(result_);
    }
}




void control::gaitPhaseCb(const std_msgs::StringConstPtr &msg)
{
    gait_phase_data.push(msg);
    if (gait_phase_data.size() > (int)freq / 20)
        gait_phase_data.pop();
}

void control::gaitPhase(const std_msgs::StringConstPtr &msg)
{
    double_support = false;
    right_support = false;
    left_contact = false;
    right_contact = false;

    if (msg->data.compare("double_support") == 0)
    {
        double_support = true;
        left_contact = true;
        right_contact = true;
    }
    else if (msg->data.compare("left_support") == 0)
    {
        left_contact = true;
    }
    else if (msg->data.compare("right_support") == 0)
    {
        right_contact = true;
        right_support = true;
    }


}

void control::wrenchLLegCb(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    wrenchLLeg_data.push(msg);
    if (wrenchLLeg_data.size() > (int)freq / 20)
        wrenchLLeg_data.pop();
}

void control::wrenchRLegCb(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    wrenchRLeg_data.push(msg);
    if (wrenchRLeg_data.size() > (int)freq / 20)
        wrenchRLeg_data.pop();
}

void control::wrenchLLeg(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    wrenchLLeg_msg = *msg;
    LLeg_GRF = Vector3d(wrenchLLeg_msg.wrench.force.x, wrenchLLeg_msg.wrench.force.y, wrenchLLeg_msg.wrench.force.z);
    LLeg_GRT = Vector3d(wrenchLLeg_msg.wrench.torque.x, wrenchLLeg_msg.wrench.torque.y, wrenchLLeg_msg.wrench.torque.z);
}
void control::wrenchRLeg(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    wrenchRLeg_msg = *msg;
    RLeg_GRF = Vector3d(wrenchRLeg_msg.wrench.force.x, wrenchRLeg_msg.wrench.force.y, wrenchRLeg_msg.wrench.force.z);
    RLeg_GRT = Vector3d(wrenchRLeg_msg.wrench.torque.x, wrenchRLeg_msg.wrench.torque.y, wrenchRLeg_msg.wrench.torque.z);
}

void control::CoMCb(const nav_msgs::OdometryConstPtr &msg)
{
    com_msg = *msg;
    com_data.push(msg);
    if (com_data.size() > (int)freq / 20)
        com_data.pop();
}

void control::jointStateCb(const sensor_msgs::JointStateConstPtr &msg)
{
    joint_data.push(msg);
    if (joint_data.size() > (int)freq / 20)
        joint_data.pop();
}

void control::ZMPCb(const geometry_msgs::PointStampedConstPtr &msg)
{
    zmp_data.push(msg);
    if (zmp_data.size() > (int)freq / 20)
        zmp_data.pop();
}
void control::joints(const sensor_msgs::JointStateConstPtr &msg)
{
    joint_state_msg = *msg;
    std::vector<double> pos_vector = joint_state_msg.position;
    double *pos_array = pos_vector.data();
    q = Eigen::Map<Eigen::Matrix<double, 26, 1>>(pos_array);
}




void control::odomCb(const nav_msgs::OdometryConstPtr &msg)
{
    odom_data.push(msg);
    if (odom_data.size() > (int)freq / 20)
        odom_data.pop();
}


void control::odom(const nav_msgs::OdometryConstPtr &msg)
{
    pwb = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    qwb = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Twb.linear() = qwb.toRotationMatrix();
    Twb.translation() = pwb;
    vwb = Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    omegawb = Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    odom_msg = *msg;
}


void control::LLegodomCb(const nav_msgs::OdometryConstPtr &msg)
{
    LLegodom_data.push(msg);
    if (LLegodom_data.size() > (int)freq / 20)
        LLegodom_data.pop();
}


void control::LLegodom(const nav_msgs::OdometryConstPtr &msg)
{
    pwl = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    qwl = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Twl.linear() = qwl.toRotationMatrix();
    Twl.translation() = pwl;
}

void control::RLegodomCb(const nav_msgs::OdometryConstPtr &msg)
{
    RLegodom_data.push(msg);
    if (RLegodom_data.size() > (int)freq / 20)
        RLegodom_data.pop();
}


void control::RLegodom(const nav_msgs::OdometryConstPtr &msg)
{
    pwr = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    qwr = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Twr.linear() = qwr.toRotationMatrix();
    Twr.translation() = pwr;
}


void control::com(const nav_msgs::OdometryConstPtr &msg)
{
    CoM = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    vCoM = Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    if (firstCoMVel)
    {
        aCoM.setZero();
        vCoM_ = vCoM;
        firstCoMVel = false;
    }
    else
    {
        aCoM = (vCoM - vCoM_) * freq;
        vCoM_ = vCoM;
    }
}

void control::zmp(const geometry_msgs::PointStampedConstPtr &msg)
{
    ZMP = Vector3d(msg->point.x, msg->point.y, msg->point.z);
}

void control::run()
{
    static ros::Rate rate(freq);

    while (ros::ok())
    {
        if (odom_data.size() > 0 && com_data.size() > 0 && joint_data.size() > 0 && zmp_data.size() > 0 && wrenchRLeg_data.size() > 0 && wrenchLLeg_data.size() > 0 && LLegodom_data.size()>0  && RLegodom_data.size()>0 && gait_phase_data.size()>0)
        {

            wrenchLLeg(wrenchLLeg_data.pop());
            wrenchRLeg(wrenchRLeg_data.pop());
            joints(joint_data.pop());
            odom(odom_data.pop());
            com(com_data.pop());
            zmp(zmp_data.pop());
            LLegodom(LLegodom_data.pop());
            RLegodom(RLegodom_data.pop());
            gaitPhase(gait_phase_data.pop());
            if (desiredTrajectoryAvailable && i < trajectorySize)
            {
                //Go To Walk Mode
                ZMP_ref = Eigen::Vector3d(VRPTrajectory.positions[i].x, VRPTrajectory.positions[i].y, VRPTrajectory.positions[i].z);
                CoM_ref = Eigen::Vector3d(CoMTrajectory.positions[i].x, CoMTrajectory.positions[i].y, CoMTrajectory.positions[i].z);
                vCoM_ref = Eigen::Vector3d(CoMTrajectory.linear_velocities[i].x, CoMTrajectory.linear_velocities[i].y, CoMTrajectory.linear_velocities[i].z);
                aCoM_ref = Eigen::Vector3d(CoMTrajectory.linear_accelerations[i].x, CoMTrajectory.linear_accelerations[i].y, CoMTrajectory.linear_accelerations[i].z);
                rf_pos_ref = Eigen::Vector3d(RLegTrajectory.positions[i].x, RLegTrajectory.positions[i].y, RLegTrajectory.positions[i].z);
                lf_pos_ref = Eigen::Vector3d(LLegTrajectory.positions[i].x, LLegTrajectory.positions[i].y, LLegTrajectory.positions[i].z);
                lf_orient_ref = Eigen::Quaterniond(LLegTrajectory.orientations[i].w, LLegTrajectory.orientations[i].x, LLegTrajectory.orientations[i].y, LLegTrajectory.orientations[i].z);
                rf_orient_ref = Eigen::Quaterniond(RLegTrajectory.orientations[i].w, RLegTrajectory.orientations[i].x, RLegTrajectory.orientations[i].y, RLegTrajectory.orientations[i].z);
                lf_vel_ref = Eigen::Vector3d(LLegTrajectory.linear_velocities[i].x, LLegTrajectory.linear_velocities[i].y, LLegTrajectory.linear_velocities[i].z);
                rf_vel_ref = Eigen::Vector3d(RLegTrajectory.linear_velocities[i].x, RLegTrajectory.linear_velocities[i].y, RLegTrajectory.linear_velocities[i].z);
                lf_ang_ref = Eigen::Vector3d(LLegTrajectory.angular_velocities[i].x, LLegTrajectory.angular_velocities[i].y, LLegTrajectory.angular_velocities[i].z);
                rf_ang_ref = Eigen::Vector3d(RLegTrajectory.angular_velocities[i].x, RLegTrajectory.angular_velocities[i].y, RLegTrajectory.angular_velocities[i].z);


                i++;
                //Check if the end of plan (eop) is reached
                if (i == trajectorySize)
                    eop = true;
            }
            else
            {
                //Go To Balance Mode and respect Joint Space Continuity
                if (eop)
                {
                    // jointNominalConfig.head(3) = pwb;
                    // jointNominalConfig(3) = qwb.w();
                    // jointNominalConfig(4) = qwb.x();
                    // jointNominalConfig(5) = qwb.y();
                    // jointNominalConfig(6) = qwb.z();
                    // jointNominalConfig.tail(26) = q;
                    jointNominalConfig = qd;
                    eop = false;
                }
                nao_whole_body_control->desired_pin->setBaseToWorldState(jointNominalConfig.head(3), Eigen::Quaterniond(jointNominalConfig(3), jointNominalConfig(4), jointNominalConfig(5), jointNominalConfig(6)));
                nao_whole_body_control->desired_pin->setBaseWorldVelocity(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
                nao_whole_body_control->desired_pin->updateJointConfig(joint_state_msg.name, jointNominalConfig.tail(26), jointNominalVelocity.tail(26));
                CoM_ref = nao_whole_body_control->desired_pin->comPosition();
                vCoM_ref.setZero();
                aCoM_ref.setZero();

                ZMP_ref = CoM_ref;
                ZMP_ref(2) = ZMP(2);
                lf_pos_ref = nao_whole_body_control->getDesiredLLegPosition();
                rf_pos_ref = nao_whole_body_control->getDesiredRLegPosition();


                lf_orient_ref = nao_whole_body_control->getDesiredLLegOrientation();
                rf_orient_ref = nao_whole_body_control->getDesiredRLegOrientation();

                lf_vel_ref.setZero();
                rf_vel_ref.setZero();
                lf_ang_ref.setZero();
                rf_ang_ref.setZero();

                i = 0;
                desiredTrajectoryAvailable = false;
            }
            humanoidGoal_.odom = odom_msg;
            humanoidGoal_.joint_state = joint_state_msg;

            //Define Tasks for Whole Body Control
            Eigen::Vector3d tempC = CoM_ref, tempV = vCoM_ref;
            if (CoM_Ad_enabled)
            {
                lc->Control(ZMP, CoM, vCoM, aCoM, CoM_ref, vCoM_ref, aCoM_ref);
                tempC = lc->getDesiredCoMPosition();
                tempV = lc->getDesiredCoMVelocity();
            }
            zd->computeDistribution(ZMP, LLeg_GRF,  RLeg_GRF,  pwl,  pwr,  right_support,  double_support);
            ps->footTorqueStabilizer(zd->tauld, zd->taurd, LLeg_GRT, RLeg_GRT, right_contact, left_contact);

            Vector3d RLeg_rpy = ps->getRightFootOrientation();
            Vector3d LLeg_rpy = ps->getLeftFootOrientation();

            Eigen::Quaterniond qrr = Eigen::AngleAxisd(RLeg_rpy(0), Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(RLeg_rpy(1), Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(RLeg_rpy(2), Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond qll = Eigen::AngleAxisd(LLeg_rpy(0), Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(LLeg_rpy(1), Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(LLeg_rpy(2), Eigen::Vector3d::UnitZ());

            //lf_orient_ref = lf_orient_ref * qll;
            //rf_orient_ref = rf_orient_ref * qrr;

            humanoidGoal_.CoM.linear_task.desired_position.x = tempC(0);
            humanoidGoal_.CoM.linear_task.desired_position.y = tempC(1);
            humanoidGoal_.CoM.linear_task.desired_position.z = tempC(2);
            humanoidGoal_.CoM.linear_task.desired_linear_velocity.x = tempV(0);
            humanoidGoal_.CoM.linear_task.desired_linear_velocity.y = tempV(1);
            humanoidGoal_.CoM.linear_task.desired_linear_velocity.z = tempV(2);

            humanoidGoal_.LLeg.linear_task.desired_position.x = lf_pos_ref(0);
            humanoidGoal_.LLeg.linear_task.desired_position.y = lf_pos_ref(1);
            humanoidGoal_.LLeg.linear_task.desired_position.z = lf_pos_ref(2);

            humanoidGoal_.LLeg.linear_task.desired_linear_velocity.x = lf_vel_ref(0);
            humanoidGoal_.LLeg.linear_task.desired_linear_velocity.y = lf_vel_ref(1);
            humanoidGoal_.LLeg.linear_task.desired_linear_velocity.z = lf_vel_ref(2);
            humanoidGoal_.LLeg.angular_task.desired_orientation.x = lf_orient_ref.x();
            humanoidGoal_.LLeg.angular_task.desired_orientation.y = lf_orient_ref.y();
            humanoidGoal_.LLeg.angular_task.desired_orientation.z = lf_orient_ref.z();
            humanoidGoal_.LLeg.angular_task.desired_orientation.w = lf_orient_ref.w();
            humanoidGoal_.LLeg.angular_task.desired_angular_velocity.x = lf_ang_ref(0);
            humanoidGoal_.LLeg.angular_task.desired_angular_velocity.y = lf_ang_ref(1);
            humanoidGoal_.LLeg.angular_task.desired_angular_velocity.z = lf_ang_ref(2);

            //RLeg Desired Trajectories
            humanoidGoal_.RLeg.linear_task.desired_position.x = rf_pos_ref(0);
            humanoidGoal_.RLeg.linear_task.desired_position.y = rf_pos_ref(1);
            humanoidGoal_.RLeg.linear_task.desired_position.z = rf_pos_ref(2);

            humanoidGoal_.RLeg.linear_task.desired_linear_velocity.x = rf_vel_ref(0);
            humanoidGoal_.RLeg.linear_task.desired_linear_velocity.y = rf_vel_ref(1);
            humanoidGoal_.RLeg.linear_task.desired_linear_velocity.z = rf_vel_ref(2);
            humanoidGoal_.RLeg.angular_task.desired_angular_velocity.x = rf_ang_ref(0);
            humanoidGoal_.RLeg.angular_task.desired_angular_velocity.y = rf_ang_ref(1);
            humanoidGoal_.RLeg.angular_task.desired_angular_velocity.z = rf_ang_ref(2);
            humanoidGoal_.RLeg.angular_task.desired_orientation.x = rf_orient_ref.x();
            humanoidGoal_.RLeg.angular_task.desired_orientation.y = rf_orient_ref.y();
            humanoidGoal_.RLeg.angular_task.desired_orientation.z = rf_orient_ref.z();
            humanoidGoal_.RLeg.angular_task.desired_orientation.w = rf_orient_ref.w();


            Quaterniond torso_orient_ref = rf_orient_ref.slerp(0.5,lf_orient_ref);
            // Vector3d eulerTorso = torso_orient_ref.toRotationMatrix().eulerAngles(0, 1, 2);
            // cout<<"euler "<<eulerTorso.transpose()<<endl;
            //torso_orient_ref = AngleAxisd(0, Vector3d::UnitX()) * AngleAxisd(0, Vector3d::UnitY())* AngleAxisd(eulerTorso(2), Vector3d::UnitZ());
            // cout<<"Torso "<<torso_orient_ref.x()<<" "<<torso_orient_ref.y()<<" "<<torso_orient_ref.z()<<" "<<torso_orient_ref.w()<<endl;
            humanoidGoal_.Torso.angular_task.desired_orientation.x = torso_orient_ref.x();
            humanoidGoal_.Torso.angular_task.desired_orientation.y = torso_orient_ref.y();
            humanoidGoal_.Torso.angular_task.desired_orientation.z = torso_orient_ref.z();
            humanoidGoal_.Torso.angular_task.desired_orientation.w = torso_orient_ref.w();
            humanoidGoal_.Torso.angular_task.desired_angular_velocity.x = 0;
            humanoidGoal_.Torso.angular_task.desired_angular_velocity.y = 0;
            humanoidGoal_.Torso.angular_task.desired_angular_velocity.z = 0;

            humanoidGoal_.Joints.resize(joint_state_msg.name.size());
            unsigned int j = 0;
            while (j < joint_state_msg.name.size())
            {
                humanoidGoal_.Joints[j].desired_angle = jointNominalConfig[j + 7];
                humanoidGoal_.Joints[j].weight = dof_weight;
                humanoidGoal_.Joints[j].gain = dof_gain;
                humanoidGoal_.Joints[j].name = joint_state_msg.name[j];
                j++;
            }
            nao_whole_body_control->controlCb(qd, dqd, humanoidGoal_);
        }
        // auto stop = high_resolution_clock::now();
        // auto duration = duration_cast<microseconds>(stop - start);
        // int loop_duration = duration.count();
        // cout << "LOOP Duration in microseconds " << loop_duration << endl;
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