#include <lipm_control/postureStabilizer.h>

postureStabilizer::postureStabilizer(RobotParameters &robot_) : robot(robot_)
{

    resetFootTorqueStabilizer();
    resetFootForceStabilizer();
    resetBaseOrientationStabilizer();
    dt = robot.getWalkParameter(Ts);
    Tc_ = robot.getWalkParameter(Tc);
    Kc_ = robot.getWalkParameter(Kc);
    Ta_ = robot.getWalkParameter(Ta);
    Ka_ = robot.getWalkParameter(Ka);
    Tn_ = robot.getWalkParameter(Tn);
    Kn_ = robot.getWalkParameter(Kn);
    std::cout << "Real-time Posture Stabilizer Initialized Successfully" << std::endl;
}

void postureStabilizer::resetFootTorqueStabilizer()
{
    dL_Roll = 0.0;
    dL_Pitch = 0.0;
    dR_Roll = 0.0;
    dR_Pitch = 0.0;
}

void postureStabilizer::footTorqueStabilizer(Vector3d tauld, Vector3d taurd, Vector3d taul, Vector3d taur, bool right_contact, bool left_contact)
{
    Ta_ = robot.getWalkParameter(Ta);
    Ka_ = robot.getWalkParameter(Ka);
    dt = robot.getWalkParameter(Ts);

    if(left_contact)
    {
        dL_Roll = Ka_ * dt * (tauld(0) - taul(0)) + (1.0 - dt / Ta_) * dL_Roll;
        dL_Pitch = Ka_ * dt * (tauld(1) - taul(1)) + (1.0 - dt / Ta_) * dL_Pitch;
    }
    else
    {
        dL_Roll = 0.0;
        dL_Pitch = 0.0;
    }
    if(right_contact)
    {
        dR_Roll = Ka_ * dt * (taurd(0) - taur(0)) + (1.0 - dt / Ta_) * dR_Roll;
        dR_Pitch = Ka_ * dt * (taurd(1) - taur(1)) + (1.0 - dt / Ta_) * dR_Pitch;
    }
    else
    {
        dR_Roll = 0.0;
        dR_Pitch = 0.0;
    }
}

void postureStabilizer::resetFootForceStabilizer()
{
    dz = 0.0;
}

void postureStabilizer::footForceStabilizer(double flz, double frz, double flz_d, double frz_d)
{
    Tn_ = robot.getWalkParameter(Tn);
    Kn_ = robot.getWalkParameter(Kn);
    dt = robot.getWalkParameter(Ts);
    double deltaF = flz - frz;
    double deltaF_d = flz_d - frz_d;

    dz = Kn_ * dt * (deltaF_d - deltaF) + (1.0 - dt / Tn_) * dz;

    dLz = -0.5 * dz;
    dRz = 0.5 * dz;
}

void postureStabilizer::resetBaseOrientationStabilizer()
{
    dbase_Roll = 0.0;
    dbase_Pitch = 0.0;
}

void postureStabilizer::baseOrientationStabilizer(double base_Roll, double base_Pitch, double base_Roll_d, double base_Pitch_d)
{
    Vector3d dbase;
    dbase.setZero();
    Tc_ = robot.getWalkParameter(Tc);
    Kc_ = robot.getWalkParameter(Kc);
    dt = robot.getWalkParameter(Ts);

    dbase_Roll = Kc_ * dt * (base_Roll_d - base_Roll) + (1.0 - dt / Tc_) * dbase_Roll;
    dbase_Pitch = Kc_ * dt * (base_Pitch_d - base_Pitch) + (1.0 - dt / Tc_) * dbase_Pitch;

}

Vector3d postureStabilizer::getBaseOrientation()
{

    return Vector3d(dbase_Roll, dbase_Pitch, 0);
}

Vector3d postureStabilizer::getLeftFootOrientation()
{

    return Vector3d(dL_Roll, dL_Pitch, 0);
}

Vector3d postureStabilizer::getRightFootOrientation()
{
    return Vector3d(dR_Roll, dR_Pitch, 0);
}

double postureStabilizer::getRightFootVerticalPosition()
{
    return dRz;
}

double postureStabilizer::getLeftFootVerticalPosition()
{
    return dLz;
}
