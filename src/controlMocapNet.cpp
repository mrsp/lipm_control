#include "lipm_control/controlMocapNet.h"
#include <chrono>

using namespace std::chrono;

controlMocapNet::controlMocapNet(ros::NodeHandle nh_)
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

    n_p.param<std::string>("joint_cmd_topic", joint_cmd_topic, "/mocapnet/joint_states");



    //Load the CoM Admittance control Parameters
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

    humanoidGoal_.Head.angular_task.weight = Head_task_weight;
    humanoidGoal_.Head.angular_task.gain = QGain;

    humanoidGoal_.LHand.angular_task.weight = LHand_task_weight;
    humanoidGoal_.LHand.angular_task.gain = QGain;


    humanoidGoal_.RHand.angular_task.weight = RHand_task_weight;
    humanoidGoal_.RHand.angular_task.gain = QGain;

    dof_weight = DOF_weight;
    dof_gain = DOF_gain;

    //CoM Admittance control Module
    lc = new LIPMControl(hc, 1.0 / freq, Pcom_x, Pcom_y, Pzmp_x, Pzmp_y, Pdcm_x, Pdcm_y, Idcm_x, Idcm_y);
    zd = new ZMPDistributor(mass, g);

    double Tc = 0;
    double Kc = 0;
    double Ta = 0.001;
    double Ka = 0.1;
    double Tn = 0;
    double Kn = 0;

    n_p.param<double>("foot_damping_control_gain", Ka, 0.01);
    n_p.param<double>("foot_damping_control_time_constant", Ta, 0.001);
    n_p.param<double>("base_damping_control_gain", Kc, 0.01);
    n_p.param<double>("base_damping_control_time_constant", Tc, 0.001);


    ps = new postureStabilizer(1.0/freq, Kc, Tc, Ka, Ta, Kn, Tn);
    //Humanoid WBC Module
    humanoid_whole_body_control = new humanoid_wbc(nh);

    //LIPM control Parameters
    Twb = Eigen::Affine3d::Identity();
    pwb.setZero();
    vwb.setZero();
    omegawb.setZero();
    firstCoMVel = true;
    odom_sub = nh.subscribe(odom_topic, 1000, &controlMocapNet::odomCb, this);
    zmp_sub = nh.subscribe(zmp_topic, 1000, &controlMocapNet::ZMPCb, this);
    com_sub = nh.subscribe(com_topic, 1000, &controlMocapNet::CoMCb, this);
    
    wrenchLLeg_sub = nh.subscribe(wrenchLLeg_topic, 1000, &controlMocapNet::wrenchLLegCb, this);
    wrenchRLeg_sub = nh.subscribe(wrenchRLeg_topic, 1000, &controlMocapNet::wrenchRLegCb, this);
    
    LLegodom_sub = nh.subscribe(LLegodom_topic, 1000, &controlMocapNet::LLegodomCb, this);
    RLegodom_sub = nh.subscribe(RLegodom_topic, 1000, &controlMocapNet::RLegodomCb, this);
    gait_phase_sub = nh.subscribe(gait_phase_topic, 1000, &controlMocapNet::gaitPhaseCb, this);

    joint_state_sub = nh.subscribe(joint_state_topic, 1000, &controlMocapNet::jointStateCb, this);
    joint_cmd_sub = nh.subscribe(joint_state_topic, 1000, &controlMocapNet::jointStateCb, this);


    jointNominalConfig = humanoid_whole_body_control->jointNominalConfig;
    jointNominalVelocity = humanoid_whole_body_control->jointNominalVelocity;
    jointMocapNominalConfig = jointNominalConfig.tail(jointNominalConfig.size()-7);


    qd = jointNominalConfig;
    dqd = jointNominalVelocity;


    right_support = false;
    double_support = false;
    right_contact = false;
    left_contact = false;
    initialized = false;
    cout << "LIPM control Module with MocapNet  Initialized " << endl;
}



void controlMocapNet::gaitPhaseCb(const std_msgs::StringConstPtr &msg)
{
    gait_phase_data.push(msg);
    if (gait_phase_data.size() > (int)freq / 20)
        gait_phase_data.pop();
}

void controlMocapNet::gaitPhase(const std_msgs::StringConstPtr &msg)
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

void controlMocapNet::wrenchLLegCb(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    wrenchLLeg_data.push(msg);
    if (wrenchLLeg_data.size() > (int)freq / 20)
        wrenchLLeg_data.pop();
}

void controlMocapNet::wrenchRLegCb(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    wrenchRLeg_data.push(msg);
    if (wrenchRLeg_data.size() > (int)freq / 20)
        wrenchRLeg_data.pop();
}

void controlMocapNet::wrenchLLeg(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    wrenchLLeg_msg = *msg;
    LLeg_GRF = Vector3d(wrenchLLeg_msg.wrench.force.x, wrenchLLeg_msg.wrench.force.y, wrenchLLeg_msg.wrench.force.z);
    LLeg_GRT = Vector3d(wrenchLLeg_msg.wrench.torque.x, wrenchLLeg_msg.wrench.torque.y, wrenchLLeg_msg.wrench.torque.z);
}
void controlMocapNet::wrenchRLeg(const geometry_msgs::WrenchStampedConstPtr &msg)
{
    wrenchRLeg_msg = *msg;
    RLeg_GRF = Vector3d(wrenchRLeg_msg.wrench.force.x, wrenchRLeg_msg.wrench.force.y, wrenchRLeg_msg.wrench.force.z);
    RLeg_GRT = Vector3d(wrenchRLeg_msg.wrench.torque.x, wrenchRLeg_msg.wrench.torque.y, wrenchRLeg_msg.wrench.torque.z);
}

void controlMocapNet::CoMCb(const nav_msgs::OdometryConstPtr &msg)
{
    com_msg = *msg;
    com_data.push(msg);
    if (com_data.size() > (int)freq / 20)
        com_data.pop();
}

void controlMocapNet::jointStateCb(const sensor_msgs::JointStateConstPtr &msg)
{
    joint_data.push(msg);
    if (joint_data.size() > (int)freq / 20)
        joint_data.pop();
}

void controlMocapNet::ZMPCb(const geometry_msgs::PointStampedConstPtr &msg)
{
    zmp_data.push(msg);
    if (zmp_data.size() > (int)freq / 20)
        zmp_data.pop();
}
void controlMocapNet::joints(const sensor_msgs::JointStateConstPtr &msg)
{
    joint_state_msg = *msg;
    std::vector<double> pos_vector = joint_state_msg.position;
    q = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(pos_vector.data(), pos_vector.size());
}



void controlMocapNet::jointCmdCb(const sensor_msgs::JointStateConstPtr &msg)
{
    std::vector<double> pos_vector = msg->position;
    jointMocapNominalConfig = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(pos_vector.data(), pos_vector.size());

}



void controlMocapNet::odomCb(const nav_msgs::OdometryConstPtr &msg)
{
    odom_data.push(msg);
    if (odom_data.size() > (int)freq / 20)
        odom_data.pop();
}


void controlMocapNet::odom(const nav_msgs::OdometryConstPtr &msg)
{
    pwb = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    qwb = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Twb.linear() = qwb.toRotationMatrix();
    Twb.translation() = pwb;
    vwb = Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    omegawb = Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    odom_msg = *msg;

}


void controlMocapNet::LLegodomCb(const nav_msgs::OdometryConstPtr &msg)
{
    LLegodom_data.push(msg);
    if (LLegodom_data.size() > (int)freq / 20)
        LLegodom_data.pop();
}


void controlMocapNet::LLegodom(const nav_msgs::OdometryConstPtr &msg)
{
    pwl = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    qwl = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Twl.linear() = qwl.toRotationMatrix();
    Twl.translation() = pwl;
}

void controlMocapNet::RLegodomCb(const nav_msgs::OdometryConstPtr &msg)
{
    RLegodom_data.push(msg);
    if (RLegodom_data.size() > (int)freq / 20)
        RLegodom_data.pop();
}


void controlMocapNet::RLegodom(const nav_msgs::OdometryConstPtr &msg)
{
    pwr = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    qwr = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Twr.linear() = qwr.toRotationMatrix();
    Twr.translation() = pwr;
}


void controlMocapNet::com(const nav_msgs::OdometryConstPtr &msg)
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

void controlMocapNet::zmp(const geometry_msgs::PointStampedConstPtr &msg)
{
    ZMP = Vector3d(msg->point.x, msg->point.y, msg->point.z);
}

void controlMocapNet::run()
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
            if(!initialized)
            {
                jointNominalConfig.head(3) = pwb;
                jointNominalConfig(3) = qwb.w();
                jointNominalConfig(4) = qwb.x();
                jointNominalConfig(5) = qwb.y();
                jointNominalConfig(6) = qwb.z();
                initialized = true;

                humanoid_whole_body_control->desired_pin->setBaseToWorldState(jointNominalConfig.head(3), Eigen::Quaterniond(jointNominalConfig(3), jointNominalConfig(4), jointNominalConfig(5), jointNominalConfig(6)));
                humanoid_whole_body_control->desired_pin->setBaseWorldVelocity(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
                humanoid_whole_body_control->desired_pin->updateJointConfig(joint_state_msg.name, jointNominalConfig.tail(humanoid_whole_body_control->ndof), jointNominalVelocity.tail(humanoid_whole_body_control->ndof));

                lf_pos_ref = humanoid_whole_body_control->getDesiredLLegPosition();
                rf_pos_ref = humanoid_whole_body_control->getDesiredRLegPosition();


                lf_orient_ref = humanoid_whole_body_control->getDesiredLLegOrientation();
                rf_orient_ref = humanoid_whole_body_control->getDesiredRLegOrientation();

                lf_vel_ref.setZero();
                rf_vel_ref.setZero();
                lf_ang_ref.setZero();
                rf_ang_ref.setZero();

            }
            humanoid_whole_body_control->desired_pin->updateJointConfig(joint_state_msg.name, jointMocapNominalConfig.tail(humanoid_whole_body_control->ndof), jointNominalVelocity.tail(humanoid_whole_body_control->ndof));

            CoM_ref = humanoid_whole_body_control->desired_pin->comPosition();
            vCoM_ref.setZero();
            aCoM_ref.setZero();
            ZMP_ref = CoM_ref;
            ZMP_ref(2) = ZMP(2);

            lh_orient_ref = humanoid_whole_body_control->getDesiredLHandOrientation();
            rh_orient_ref = humanoid_whole_body_control->getDesiredRHandOrientation();
            h_orient_ref = humanoid_whole_body_control->getDesiredHeadOrientation();


            humanoidGoal_.odom = odom_msg;
            humanoidGoal_.joint_state = joint_state_msg;

            //Define Tasks for Whole Body control
            Eigen::Vector3d tempC = CoM_ref, tempV = vCoM_ref;
            if (CoM_Ad_enabled)
            {
                lc->Control(ZMP, CoM, vCoM, aCoM, CoM_ref, vCoM_ref, aCoM_ref);
                tempC = lc->getDesiredCoMPosition();
                tempV = lc->getDesiredCoMVelocity();
            }
            zd->computeDistribution(ZMP_ref, ZMP,  pwl,  pwr,  right_support,  double_support);
            ps->footTorqueStabilizer(zd->tauld, zd->taurd, LLeg_GRT, RLeg_GRT, right_contact, left_contact);

            Eigen::Quaterniond qrr = ps->getRightFootOrientation();
            Eigen::Quaterniond qll = ps->getLeftFootOrientation();
            lf_orient_ref = lf_orient_ref * qll;
            rf_orient_ref = rf_orient_ref * qrr;
            Quaterniond torso_orient_ref = rf_orient_ref.slerp(0.5,lf_orient_ref);

            ps->baseOrientationStabilizer(qwb,torso_orient_ref);
            torso_orient_ref = torso_orient_ref * ps->getBaseOrientation();


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




            humanoidGoal_.Torso.angular_task.desired_orientation.x = torso_orient_ref.x();
            humanoidGoal_.Torso.angular_task.desired_orientation.y = torso_orient_ref.y();
            humanoidGoal_.Torso.angular_task.desired_orientation.z = torso_orient_ref.z();
            humanoidGoal_.Torso.angular_task.desired_orientation.w = torso_orient_ref.w();
            humanoidGoal_.Torso.angular_task.desired_angular_velocity.x = 0;
            humanoidGoal_.Torso.angular_task.desired_angular_velocity.y = 0;
            humanoidGoal_.Torso.angular_task.desired_angular_velocity.z = 0;




            // humanoidGoal_.Head.angular_task.desired_orientation.x = h_orient_ref.x();
            // humanoidGoal_.Head.angular_task.desired_orientation.y = h_orient_ref.y();
            // humanoidGoal_.Head.angular_task.desired_orientation.z = h_orient_ref.z();
            // humanoidGoal_.Head.angular_task.desired_orientation.w = h_orient_ref.w();
            // humanoidGoal_.Head.angular_task.desired_angular_velocity.x = 0;
            // humanoidGoal_.Head.angular_task.desired_angular_velocity.y = 0;
            // humanoidGoal_.Head.angular_task.desired_angular_velocity.z = 0;


            // humanoidGoal_.LHand.angular_task.desired_orientation.x = lh_orient_ref.x();
            // humanoidGoal_.LHand.angular_task.desired_orientation.y = lh_orient_ref.y();
            // humanoidGoal_.LHand.angular_task.desired_orientation.z = lh_orient_ref.z();
            // humanoidGoal_.LHand.angular_task.desired_orientation.w = lh_orient_ref.w();
            // humanoidGoal_.LHand.angular_task.desired_angular_velocity.x = 0;
            // humanoidGoal_.LHand.angular_task.desired_angular_velocity.y = 0;
            // humanoidGoal_.LHand.angular_task.desired_angular_velocity.z = 0;


            // humanoidGoal_.RHand.angular_task.desired_orientation.x = rh_orient_ref.x();
            // humanoidGoal_.RHand.angular_task.desired_orientation.y = rh_orient_ref.y();
            // humanoidGoal_.RHand.angular_task.desired_orientation.z = rh_orient_ref.z();
            // humanoidGoal_.RHand.angular_task.desired_orientation.w = rh_orient_ref.w();
            // humanoidGoal_.RHand.angular_task.desired_angular_velocity.x = 0;
            // humanoidGoal_.RHand.angular_task.desired_angular_velocity.y = 0;
            // humanoidGoal_.RHand.angular_task.desired_angular_velocity.z = 0;

            humanoidGoal_.Joints.resize(joint_state_msg.name.size());
            unsigned int j = 0;
            while (j < joint_state_msg.name.size())
            {
                humanoidGoal_.Joints[j].desired_angle = jointMocapNominalConfig[j];
                humanoidGoal_.Joints[j].weight = dof_weight;
                humanoidGoal_.Joints[j].gain = dof_gain;
                humanoidGoal_.Joints[j].name = joint_state_msg.name[j];
                j++;
            }
            humanoid_whole_body_control->controlCb(qd, dqd, humanoidGoal_);
        }
        // auto stop = high_resolution_clock::now();
        // auto duration = duration_cast<microseconds>(stop - start);
        // int loop_duration = duration.count();
        // cout << "LOOP Duration in microseconds " << loop_duration << endl;
        rate.sleep();
        ros::spinOnce();
    }
}

controlMocapNet::~controlMocapNet()
{
}