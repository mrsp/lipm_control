#include "lipm_control/control.h"
#include <chrono>

using namespace std::chrono;

control::control(ros::NodeHandle nh_)
{
    nh = nh_;
    ros::NodeHandle n_p("~");
    double g, comZ, dt;
    n_p.param<double>("gravity", g, 9.80665);
    n_p.param<double>("hc", comZ, 0.879533781657);
    n_p.param<double>("control_frequency", freq, 100);
    n_p.param<std::string>("joint_state_topic", joint_state_topic, "/nao_raisim_ros/joint_states");
    n_p.param<std::string>("com_topic", com_topic, "/nao_raisim_ros/CoM");
    n_p.param<std::string>("odom_topic", odom_topic, "/nao_raisim_ros/odom");
    n_p.param<std::string>("action_server_topic", action_server_topic, "/nao_raisim_ros/whole_body_control");
    n_p.param<std::string>("zmp_topic", zmp_topic, "/nao_raisim_ros/ZMP");

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
    joint_state_sub = nh.subscribe(joint_state_topic, 1000, &control::jointStateCb, this);
    desiredTrajectoryAvailable = false;
    as_ = new actionlib::SimpleActionServer<lipm_msgs::MotionControlAction>(nh, "lipm_control/plan", boost::bind(&control::desiredTrajectoryCb, this, _1), false);
    as_->start();

    nao_whole_body_control = new nao_wbc(nh);
    lc = new LIPMControl(0.26818, 5.14, 1.0 / freq);
    desired_pin = new pin_wrapper("/home/master/catkin_ws/src/whole_body_ik/share/urdf/nao.urdf", true);

    jointNominalConfig.resize(33);
    jointNominalConfig.setZero();
    qd = jointNominalConfig;
    jointNominalConfig << 0, 0, 0.32, 1, 0, 0, 0,
        0.0, 0.0,
        0.0, 0.0, -0.3976, 0.85, -0.4427, -0.009,
        0.0, 0.0, -0.3976, 0.85, -0.4427, -0.009,
        1.5, 0.15, 0, -0.0349066, -1.5, 0,
        1.5, -0.15, 0, 0.0349066, 1.5, 0;

    jointVelocityTarget.resize(32);
    jointVelocityTarget.setZero();
    eop = false;
    cout << "LIPM Control Module Initialized " << endl;
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
    odom_msg = *msg;
    odom_data.push(msg);
    if (odom_data.size() > (int)freq / 20)
        odom_data.pop();
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
        auto start = high_resolution_clock::now();

        if (odom_data.size() > 0 && com_data.size() > 0 && joint_data.size() > 0 && zmp_data.size() > 0)
        {

            joints(joint_data.pop());
            odom(odom_data.pop());
            com(com_data.pop());
            zmp(zmp_data.pop());

            whole_body_ik_msgs::HumanoidGoal humanoidGoal_;
            double Gain = 0.4;
            double QGain = 0.4;
            humanoidGoal_.dt = 1.0 / freq;
            humanoidGoal_.odom = odom_msg;
            humanoidGoal_.joint_state = joint_state_msg;
            humanoidGoal_.CoM.linear_task.weight = 1e-1;
            humanoidGoal_.CoM.linear_task.gain = Gain;
            humanoidGoal_.LLeg.linear_task.weight = 1.0;
            humanoidGoal_.LLeg.linear_task.gain = Gain;
            humanoidGoal_.LLeg.angular_task.weight = 1.0;
            humanoidGoal_.LLeg.angular_task.gain = QGain;
            humanoidGoal_.RLeg.linear_task.weight = 1.0;
            humanoidGoal_.RLeg.linear_task.gain = Gain;
            humanoidGoal_.RLeg.angular_task.weight = 1.0;
            humanoidGoal_.RLeg.angular_task.gain = QGain;
            humanoidGoal_.Torso.angular_task.weight = 1e-3;
            humanoidGoal_.Torso.angular_task.gain = QGain;

            if (desiredTrajectoryAvailable && i < trajectorySize)
            {
                //Go To Walk Mode
                CoM_ref = Eigen::Vector3d(CoMTrajectory.positions[i].x, CoMTrajectory.positions[i].y, CoMTrajectory.positions[i].z);
                vCoM_ref = Eigen::Vector3d(CoMTrajectory.linear_velocities[i].x, CoMTrajectory.linear_velocities[i].y, CoMTrajectory.linear_velocities[i].z);
                aCoM_ref = Eigen::Vector3d(CoMTrajectory.linear_accelerations[i].x, CoMTrajectory.linear_accelerations[i].y, CoMTrajectory.linear_accelerations[i].z);
                rf_pos_ref = Eigen::Vector3d(RLegTrajectory.positions[i].x, RLegTrajectory.positions[i].y, RLegTrajectory.positions[i].z);
                lf_pos_ref = Eigen::Vector3d(LLegTrajectory.positions[i].x, LLegTrajectory.positions[i].y, LLegTrajectory.positions[i].z);
                i++;
                if (i == trajectorySize)
                    eop = true;
            }
            else
            {
                //Go To Balance Mode
                if (eop)
                {
                    jointNominalConfig = qd;
                    eop = false;
                }
                desired_pin->setBaseToWorldState(jointNominalConfig.head(3), Eigen::Quaterniond(jointNominalConfig(3), jointNominalConfig(4), jointNominalConfig(5), jointNominalConfig(6)));
                desired_pin->setBaseWorldVelocity(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
                desired_pin->updateJointConfig(joint_state_msg.name, jointNominalConfig.tail(26), jointVelocityTarget.tail(26));
                CoM_ref = desired_pin->comPosition();
                vCoM_ref = Eigen::Vector3d(0, 0, 0);
                aCoM_ref = Eigen::Vector3d(0, 0, 0);
                string lfoot_frame = "l_sole";
                string rfoot_frame = "r_sole";
                lf_pos_ref = desired_pin->linkPosition(lfoot_frame);
                rf_pos_ref = desired_pin->linkPosition(rfoot_frame);
                i = 0;
                desiredTrajectoryAvailable = false;
            }
            Eigen::Vector3d temp;
            lc->Control(ZMP, CoM, vCoM, aCoM, CoM_ref, vCoM_ref, aCoM_ref);
            temp = lc->getDesiredCoMPosition();
            //temp = CoM_ref;
            humanoidGoal_.CoM.linear_task.desired_position.x = temp(0);
            humanoidGoal_.CoM.linear_task.desired_position.y = temp(1);
            humanoidGoal_.CoM.linear_task.desired_position.z = temp(2);
            temp = lc->getDesiredCoMVelocity();
            //temp = vCoM_ref;
            humanoidGoal_.CoM.linear_task.desired_linear_velocity.x = temp(0);
            humanoidGoal_.CoM.linear_task.desired_linear_velocity.y = temp(1);
            humanoidGoal_.CoM.linear_task.desired_linear_velocity.z = temp(2);

            humanoidGoal_.LLeg.linear_task.desired_position.x = lf_pos_ref(0);
            humanoidGoal_.LLeg.linear_task.desired_position.y = lf_pos_ref(1);
            humanoidGoal_.LLeg.linear_task.desired_position.z = lf_pos_ref(2);

            humanoidGoal_.LLeg.linear_task.desired_linear_velocity.x = 0;
            humanoidGoal_.LLeg.linear_task.desired_linear_velocity.y = 0;
            humanoidGoal_.LLeg.linear_task.desired_linear_velocity.z = 0;
            humanoidGoal_.LLeg.angular_task.desired_orientation.x = 0;
            humanoidGoal_.LLeg.angular_task.desired_orientation.y = 0;
            humanoidGoal_.LLeg.angular_task.desired_orientation.z = 0;
            humanoidGoal_.LLeg.angular_task.desired_orientation.w = 1;
            humanoidGoal_.LLeg.angular_task.desired_angular_velocity.x = 0;
            humanoidGoal_.LLeg.angular_task.desired_angular_velocity.y = 0;
            humanoidGoal_.LLeg.angular_task.desired_angular_velocity.z = 0;

            //RLeg Desired Trajectories
            humanoidGoal_.RLeg.linear_task.desired_position.x = rf_pos_ref(0);
            humanoidGoal_.RLeg.linear_task.desired_position.y = rf_pos_ref(1);
            humanoidGoal_.RLeg.linear_task.desired_position.z = rf_pos_ref(2);

            humanoidGoal_.RLeg.linear_task.desired_linear_velocity.x = 0;
            humanoidGoal_.RLeg.linear_task.desired_linear_velocity.y = 0;
            humanoidGoal_.RLeg.linear_task.desired_linear_velocity.z = 0;
            humanoidGoal_.RLeg.angular_task.desired_angular_velocity.x = 0;
            humanoidGoal_.RLeg.angular_task.desired_angular_velocity.y = 0;
            humanoidGoal_.RLeg.angular_task.desired_angular_velocity.z = 0;
            humanoidGoal_.RLeg.angular_task.desired_orientation.x = 0;
            humanoidGoal_.RLeg.angular_task.desired_orientation.y = 0;
            humanoidGoal_.RLeg.angular_task.desired_orientation.z = 0;
            humanoidGoal_.RLeg.angular_task.desired_orientation.w = 1;

            humanoidGoal_.Torso.angular_task.desired_orientation.x = 0;
            humanoidGoal_.Torso.angular_task.desired_orientation.y = 0;
            humanoidGoal_.Torso.angular_task.desired_orientation.z = 0;
            humanoidGoal_.Torso.angular_task.desired_orientation.w = 1;
            humanoidGoal_.Torso.angular_task.desired_angular_velocity.x = 0;
            humanoidGoal_.Torso.angular_task.desired_angular_velocity.y = 0;
            humanoidGoal_.Torso.angular_task.desired_angular_velocity.z = 0;

            double dof_weight = 5e-5;
            double dof_gain = 0.8;
            humanoidGoal_.Joints.resize(joint_state_msg.name.size());
            //Define Tasks for Whole Body Control
            unsigned int j = 0;
            while (j < joint_state_msg.name.size())
            {
                humanoidGoal_.Joints[j].desired_angle = jointNominalConfig[j + 7];
                humanoidGoal_.Joints[j].weight = dof_weight;
                humanoidGoal_.Joints[j].gain = dof_gain;
                humanoidGoal_.Joints[j].name = joint_state_msg.name[j];
                j++;
            }
            nao_whole_body_control->controlCb(qd, humanoidGoal_);
        }
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        int loop_duration = duration.count();
        cout << "LOOP Duration in microseconds " << loop_duration << endl;
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