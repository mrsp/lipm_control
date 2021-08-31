#include <lipm_control/ZMPDistributor.h>
/* alpha = 1.0 -> right */

ZMPDistributor::ZMPDistributor(double mass, double g)
{
    frd.setZero();
    fld.setZero();
    tauld.setZero();
    taurd.setZero();
    tau0.setZero();
    a = 0.000;
    maxForceReadingL = mass * g;
    maxForceReadingR = mass * g;
    std::cout << "ZMP Distributor Initialized Successfully" << std::endl;
}

void ZMPDistributor::computeDistribution(Vector3d pZMPd, Vector3d fl, Vector3d fr, Vector3d pL, Vector3d pR, bool right_support, bool double_support)
{
    tauld.setZero();
    taurd.setZero();
    tau0.setZero();
    fld.setZero();
    frd.setZero();

    if (!double_support)
    {
        a = 0.0;
        if (right_support)
            a = 1.0;
    }
    else
    {
        Vector3d vLR = pR - pL;
        Vector3d vLZMPd = pZMPd - pL;
        Vector3d v3 = vLR * (vLZMPd.transpose() * vLR) / (vLR.transpose() * vLR);
        Vector3d pa = pL + v3;

        //Compute Distribution parameter a
        a = (pa - pL).norm() / ((pL - pR).norm());
        if (a > 1.0)
            a = 1.0;
    }

    frd(2) = a * maxForceReadingR;
    fld(2) = (1.0 - a) * maxForceReadingL;

    tau0 = -(pR - pZMPd).cross(frd) - (pL - pZMPd).cross(fld);
    taurd = a * tau0;
    tauld = (1 - a) * tau0;
    // if(tau0(0)<0)
    // {
    //     taurd(0) = tau0(0);
    //     tauld(0) = 0;
    // }
    // else
    // {
    //     tauld(0) = tau0(0);
    //     taurd(0) = 0;
    // }

}
