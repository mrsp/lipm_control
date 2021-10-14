#include <lipm_control/postureStabilizer.h>

postureStabilizer::postureStabilizer(double dt_, double Kc_, double Tc_, double Ka_, double Ta_, double Kn_, double Tn_) 
{
    resetFootTorqueStabilizer();
    resetFootForceStabilizer();
    resetBaseOrientationStabilizer();
    dt = dt_;
    Tc = Tc_;
    Kc = Kc_;
    Ta = Ta_;
    Ka = Ka_;
    Tn = Tn_;
    Kn = Kn_;
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
    if(left_contact)
    {
        dL_Roll = Ka * dt * (tauld(0) - taul(0)) + (1.0 - dt / Ta) * dL_Roll;
        dL_Pitch = Ka * dt * (tauld(1) - taul(1)) + (1.0 - dt / Ta) * dL_Pitch;
    }
    else
    {
        dL_Roll = 0.0;
        dL_Pitch = 0.0;
    }
    if(right_contact)
    {
        dR_Roll = Ka * dt * (taurd(0) - taur(0)) + (1.0 - dt / Ta) * dR_Roll;
        dR_Pitch = Ka * dt * (taurd(1) - taur(1)) + (1.0 - dt / Ta) * dR_Pitch;
    }
    else
    {
        dR_Roll = 0.0;
        dR_Pitch = 0.0;
    }
    // std::cout<<"Left  A"<<dL_Roll<<" "<<dL_Pitch<<std::endl;
    // std::cout<<"Left  T"<<tauld.transpose()<<" "<<taul.transpose()<<std::endl;

    // std::cout<<"Right A"<<dR_Roll<<" "<<dR_Pitch<<std::endl;
    // std::cout<<"Right  T"<<taurd.transpose()<<" "<<taur.transpose()<<std::endl;

}

void postureStabilizer::resetFootForceStabilizer()
{
    dz = 0.0;
}

void postureStabilizer::footForceStabilizer(double flz, double frz, double flz_d, double frz_d)
{
    double deltaF = flz - frz;
    double deltaF_d = flz_d - frz_d;

    dz = Kn * dt * (deltaF_d - deltaF) + (1.0 - dt / Tn) * dz;
    dLz = -0.5 * dz;
    dRz = 0.5 * dz;
}

void postureStabilizer::resetBaseOrientationStabilizer()
{
    dbase_Roll = 0.0;
    dbase_Pitch = 0.0;
    dbase.setIdentity();
}

void postureStabilizer::baseOrientationStabilizer(Quaterniond qm, Quaterniond qref)
{

    Vector3d rot_error = logMap((qm.inverse()* qref).toRotationMatrix());

    dbase_Roll = Kc * dt * (rot_error(0)) + (1.0 - dt / Tc) * dbase_Roll;
    dbase_Pitch = Kc * dt * (rot_error(1)) + (1.0 - dt / Tc) * dbase_Pitch;
    dbase = expMap(Vector3d(dbase_Roll,dbase_Pitch,0));
}

Quaterniond postureStabilizer::getBaseOrientation()
{

    return Quaterniond(dbase);
}

Quaterniond postureStabilizer::getLeftFootOrientation()
{

    return Quaterniond(rotation_from_euler(dL_Roll, dL_Pitch, 0));
}

Quaterniond postureStabilizer::getRightFootOrientation()
{
    return Quaterniond(rotation_from_euler(dR_Roll, dR_Pitch, 0));
}

Vector3d postureStabilizer::getRightFootVerticalPosition()
{
    return Vector3d(0,0,dRz);
}

Vector3d postureStabilizer::getLeftFootVerticalPosition()
{
    return Vector3d(0,0,dLz);
}



Eigen::Matrix3d postureStabilizer::rotation_from_euler(double roll, double pitch, double yaw)
{
    // roll and pitch and yaw in radians
    double su = sin(roll);
    double cu = cos(roll);
    double sv = sin(pitch);
    double cv = cos(pitch);
    double sw = sin(yaw);
    double cw = cos(yaw);
    Eigen::Matrix3d Rot_matrix(3, 3);
    Rot_matrix(0, 0) = cv*cw;
    Rot_matrix(0, 1) = su*sv*cw - cu*sw;
    Rot_matrix(0, 2) = su*sw + cu*sv*cw;
    Rot_matrix(1, 0) = cv*sw;
    Rot_matrix(1, 1) = cu*cw + su*sv*sw;
    Rot_matrix(1, 2) = cu*sv*sw - su*cw;
    Rot_matrix(2, 0) = -sv;
    Rot_matrix(2, 1) = su*cv;
    Rot_matrix(2, 2) = cu*cv;
    return Rot_matrix;
}