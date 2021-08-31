#ifndef __POSTURESTABILIZER_H__
#define __POSTURESTABILIZER_H__
#include <eigen3/Eigen/Dense>
#include <iostream>
using namespace Eigen;

class postureStabilizer
{
private:
    double dbase_Roll, dbase_Pitch, dL_Roll, dL_Pitch, dR_Roll, dR_Pitch, dLz, dRz, dz;
public:
    /** Ankle PD Control **/
    double Kc, Tc, Ka, Ta, Kn, Tn, dt;

    postureStabilizer(double dt_, double Kc_, double Tc_, double Ka_, double Ta_, double Kn_, double Tn_);
    void resetFootTorqueStabilizer();
    void footTorqueStabilizer(Vector3d tauld, Vector3d taurd, Vector3d taul, Vector3d taur, bool right_contact, bool left_contact);
    void resetFootForceStabilizer();
    void footForceStabilizer(double flz, double frz, double flz_d, double frz_d);
    void resetBaseOrientationStabilizer();
    void baseOrientationStabilizer(double base_Roll, double base_Pitch, double base_Roll_d, double base_Pitch_d);
    Vector3d getBaseOrientation();
    Vector3d getLeftFootOrientation();
    Vector3d getRightFootOrientation();
    Vector3d getRightFootVerticalPosition();
    Vector3d getLeftFootVerticalPosition();
};
#endif
