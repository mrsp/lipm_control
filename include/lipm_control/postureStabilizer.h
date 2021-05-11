#ifndef __POSTURESTABILIZER_H__
#define __POSTURESTABILIZER_H__
#include <lipm_control/RobotParameters.h>
#include <eigen3/Eigen/Dense>
//#include "ZMPDistributor.h"

using namespace Eigen;

class postureStabilizer
{
private:
    RobotParameters robot;

public:
    /** Ankle PD Control **/
    double Kc_, Tc_, Ka_, Ta_, Kn_, Tn_, dt;
    double dbase_Roll, dbase_Pitch, dL_Roll, dL_Pitch, dR_Roll, dR_Pitch, dLz, dRz, dz;

    postureStabilizer(RobotParameters &robot_);

    void resetFootTorqueStabilizer();
    void footTorqueStabilizer(Vector3d tauld, Vector3d taurd, Vector3d taul, Vector3d taur, bool right_contact, bool left_contact);
    void resetFootForceStabilizer();
    void footForceStabilizer(double flz, double frz, double flz_d, double frz_d);
    void resetBaseOrientationStabilizer();
    void baseOrientationStabilizer(double base_Roll, double base_Pitch, double base_Roll_d, double base_Pitch_d);
    Vector3d getBaseOrientation();
    Vector3d getLeftFootOrientation();
    Vector3d getRightFootOrientation();
    double getRightFootVerticalPosition();
    double getLeftFootVerticalPosition();
};
#endif
