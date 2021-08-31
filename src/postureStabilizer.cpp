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
}

void postureStabilizer::baseOrientationStabilizer(double base_Roll, double base_Pitch, double base_Roll_d, double base_Pitch_d)
{

    dbase_Roll = Kc * dt * (base_Roll_d - base_Roll) + (1.0 - dt / Tc) * dbase_Roll;
    dbase_Pitch = Kc * dt * (base_Pitch_d - base_Pitch) + (1.0 - dt / Tc) * dbase_Pitch;

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

Vector3d postureStabilizer::getRightFootVerticalPosition()
{
    return Vector3d(0,0,dRz);
}

Vector3d postureStabilizer::getLeftFootVerticalPosition()
{
    return Vector3d(0,0,dLz);
}
