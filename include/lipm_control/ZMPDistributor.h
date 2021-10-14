#ifndef  __ZMPDistributor_H__
#define  __ZMPDistributor_H__
#include <eigen3/Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;



class ZMPDistributor
{

private:
    double maxForceReadingL, maxForceReadingR;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ZMPDistributor(double mass, double g);
    Vector3d frd, fld, tauld, taurd, tau0;
    double a;
    void computeDistribution(Vector3d pZMPd, Vector3d pZMP, Vector3d pL, Vector3d pR, bool right_support, bool double_support);

    Vector2d computePointLineIntersection(Vector2d point_, Vector2d linepoint0_, Vector2d linepoint1_)
    {   
        Vector2d res;
        res.setZero();
        double dx = linepoint0_(0)-linepoint1_(0);
        double dy = linepoint0_(1)-linepoint1_(1);
        double mag = sqrt(dx*dx + dy*dy);
        dx /= mag;
        dy /= mag;
        // translate the point and get the dot product
        double lambda = (dx * (point_(0) - linepoint1_(0))) + (dy * (point_(1) - linepoint1_(1)));
        res(0) = (dx * lambda) + linepoint1_(0);
        res(1) = (dy * lambda) + linepoint1_(1);
        return res;
    }
};
#endif
