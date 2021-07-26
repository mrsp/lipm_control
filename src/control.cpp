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

    Twb = Eigen::Affine3d::Identity();
    pwb.setZero();
    vwb.setZero();
    omegawb.setZero();
    i = 0;

    trajectorySize = 0;
    odom_sub = nh.subscribe(odom_topic, 1000, &control::odomCb, this);
    com_sub = nh.subscribe(com_topic, 1000, &control::CoMCb, this);
    joint_state_sub = nh.subscribe(joint_state_topic, 1000, &control::jointStateCb, this);
    desiredTrajectoryAvailable = false;
    as_ = new actionlib::SimpleActionServer<lipm_msgs::MotionControlAction>(nh, "lipm_control/plan", boost::bind(&control::desiredTrajectoryCb, this, _1), false);
    as_->start();
    ac_ = new actionlib::SimpleActionClient<whole_body_ik_msgs::HumanoidAction>(action_server_topic, true);
    ac_->waitForServer();




  jointNominalConfig.resize(33);
  jointNominalConfig.setZero();
  jointNominalConfig << 0, 0, 0.32  , 1, 0, 0, 0,
      0.0, 0.0, 
      0.0, 0.0, -0.3976, 0.85, -0.4427, -0.009,
      0.0, 0.0, -0.3976, 0.85, -0.4427, -0.009,
      1.5, 0.15, 0, -0.0349066, -1.5, 0,
      1.5, -0.15, 0, 0.0349066,  1.5, 0;
 


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
    odom_data.push(msg);
    if (odom_data.size() > (int)freq / 20)
        odom_data.pop();
}

void control::CoMCb(const nav_msgs::OdometryConstPtr &msg)
{
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
    com_msg = *msg;
    CoM =  Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    vCoM = Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
}


void control::run()
{
    static ros::Rate rate(freq);

    while (ros::ok())
    {
    auto start = high_resolution_clock::now();

        if (odom_data.size() > 0 && com_data.size() > 0 && joint_data.size() > 0)
        {

            joints(joint_data.pop());
            odom(odom_data.pop());
            com(com_data.pop());


            if (desiredTrajectoryAvailable && i < trajectorySize)
            {

                std::cout << "Loop " << i << std::endl;
                whole_body_ik_msgs::HumanoidGoal humanoidGoal_;
                double Gain = 0.85;
                double QGain = 0.85;
                humanoidGoal_.dt = 1.0 / freq;
                humanoidGoal_.odom = odom_msg;
                humanoidGoal_.joint_state = joint_state_msg;

                Eigen::Vector3d temp;
                Eigen::Quaterniond tempq;

                temp = Eigen::Vector3d(CoMTrajectory.positions[i].x, CoMTrajectory.positions[i].y, CoMTrajectory.positions[i].z);
                humanoidGoal_.CoM.linear_task.desired_position.x = temp(0);
                humanoidGoal_.CoM.linear_task.desired_position.y = temp(1);
                humanoidGoal_.CoM.linear_task.desired_position.z = temp(2);
                temp = Eigen::Vector3d(CoMTrajectory.linear_velocities[i].x, CoMTrajectory.linear_velocities[i].y, CoMTrajectory.linear_velocities[i].z);
                humanoidGoal_.CoM.linear_task.desired_linear_velocity.x = temp(0);
                humanoidGoal_.CoM.linear_task.desired_linear_velocity.y = temp(1);
                humanoidGoal_.CoM.linear_task.desired_linear_velocity.z = temp(2);
                humanoidGoal_.CoM.linear_task.weight = 5e-2;
                humanoidGoal_.CoM.linear_task.gain = Gain;


                temp = Eigen::Vector3d(LLegTrajectory.positions[i].x, LLegTrajectory.positions[i].y, LLegTrajectory.positions[i].z);
                humanoidGoal_.LLeg.linear_task.desired_position.x = temp(0);
                humanoidGoal_.LLeg.linear_task.desired_position.y = temp(1);
                humanoidGoal_.LLeg.linear_task.desired_position.z = temp(2);
                humanoidGoal_.LLeg.linear_task.weight = 1.0;
                humanoidGoal_.LLeg.linear_task.gain = Gain;
                
                temp = Eigen::Vector3d(LLegTrajectory.linear_velocities[i].x, LLegTrajectory.linear_velocities[i].y, LLegTrajectory.linear_velocities[i].z);
                humanoidGoal_.LLeg.linear_task.desired_linear_velocity.x = 0;
                humanoidGoal_.LLeg.linear_task.desired_linear_velocity.y = 0;
                humanoidGoal_.LLeg.linear_task.desired_linear_velocity.z = 0;

                //tempq = Eigen::Quaterniond(LLegTrajectory.orientations[i].w(),LLegTrajectory.orientations[i].x(), LLegTrajectory.orientations[i].y(), LLegTrajectory.orientations[i].z());
                humanoidGoal_.LLeg.angular_task.desired_orientation.x = 0;
                humanoidGoal_.LLeg.angular_task.desired_orientation.y = 0;
                humanoidGoal_.LLeg.angular_task.desired_orientation.z = 0;
                humanoidGoal_.LLeg.angular_task.desired_orientation.w = 1;
                humanoidGoal_.LLeg.angular_task.weight = 1.0;
                humanoidGoal_.LLeg.angular_task.gain = QGain;
                
                temp = Eigen::Vector3d(LLegTrajectory.angular_velocities[i].x, LLegTrajectory.angular_velocities[i].y, LLegTrajectory.angular_velocities[i].z);
                humanoidGoal_.LLeg.angular_task.desired_angular_velocity.x = 0;
                humanoidGoal_.LLeg.angular_task.desired_angular_velocity.y = 0;
                humanoidGoal_.LLeg.angular_task.desired_angular_velocity.z = 0;

                //RLeg Desired Trajectories
                temp = Eigen::Vector3d(RLegTrajectory.positions[i].x, RLegTrajectory.positions[i].y, RLegTrajectory.positions[i].z);
                humanoidGoal_.RLeg.linear_task.desired_position.x = temp(0);
                humanoidGoal_.RLeg.linear_task.desired_position.y = temp(1);
                humanoidGoal_.RLeg.linear_task.desired_position.z = temp(2);
                humanoidGoal_.RLeg.linear_task.weight = 1.0;
                humanoidGoal_.RLeg.linear_task.gain = Gain;
                
                temp = Eigen::Vector3d(RLegTrajectory.linear_velocities[i].x, RLegTrajectory.linear_velocities[i].y, RLegTrajectory.linear_velocities[i].z);
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
                humanoidGoal_.RLeg.angular_task.weight = 1.0;
                humanoidGoal_.RLeg.angular_task.gain = QGain;

                humanoidGoal_.Torso.angular_task.desired_orientation.x = 0;
                humanoidGoal_.Torso.angular_task.desired_orientation.y = 0;
                humanoidGoal_.Torso.angular_task.desired_orientation.z = 0;
                humanoidGoal_.Torso.angular_task.desired_orientation.w = 1;
                humanoidGoal_.Torso.angular_task.desired_angular_velocity.x = 0;
                humanoidGoal_.Torso.angular_task.desired_angular_velocity.y = 0;
                humanoidGoal_.Torso.angular_task.desired_angular_velocity.z = 0;
                humanoidGoal_.Torso.angular_task.weight = 1e-3;
                humanoidGoal_.Torso.angular_task.gain = QGain;


                double dof_weight = 5e-5;
                double dof_gain = 0.8;
                humanoidGoal_.Joints.resize(joint_state_msg.name.size());
                //Define Tasks for Whole Body Control
                unsigned int j = 0;
                while (j < joint_state_msg.name.size())
                {
                    humanoidGoal_.Joints[j].desired_angle =  jointNominalConfig[j+7];
                    humanoidGoal_.Joints[j].weight = dof_weight;
                    humanoidGoal_.Joints[j].gain = dof_gain;
                    humanoidGoal_.Joints[j].name = joint_state_msg.name[j];
                    j++;
                }            

                ac_->sendGoal(humanoidGoal_);
                //ac_->waitForResult();
                i++;
            }
            else
            {
                i = 0;
                desiredTrajectoryAvailable = false;
            }
        }
        auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    int loop_duration = duration.count();
    cout<<"LOOP Duration in microseconds "<<loop_duration<<endl;
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