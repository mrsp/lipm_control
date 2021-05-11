#ifndef __FOOTPOLYGON_H__
#define __FOOTPOLYGON_H__

#include <lipm_control/RobotParameters.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;
  

class FootPolygon
{

    private:
        RobotParameters robot;
        double stepXF, stepXH, stepYL, stepYR;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vector3d footLF, footRF, footLH, footRH, footA;
        Matrix<double,4,2> polygon;
        int numNodes;
        FootPolygon(RobotParameters& robot_);
        void setPolygon(Vector3d footA_, Affine3d T);
        bool checkIn(double x_, double y_);  
        bool pnpoly(double x_, double y_);
        double getAngle(double x1, double y1, double x2, double y2);

};


#endif