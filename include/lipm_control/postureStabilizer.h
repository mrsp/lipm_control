#ifndef __POSTURESTABILIZER_H__
#define __POSTURESTABILIZER_H__
#include <eigen3/Eigen/Dense>
#include <iostream>
using namespace Eigen;

class postureStabilizer
{
private:
    double dbase_Roll, dbase_Pitch, dL_Roll, dL_Pitch, dR_Roll, dR_Pitch, dLz, dRz, dz;
    Matrix3d dbase;
public:
    /** Ankle PD Control **/
    double Kc, Tc, Ka, Ta, Kn, Tn, dt;

    postureStabilizer(double dt_, double Kc_, double Tc_, double Ka_, double Ta_, double Kn_, double Tn_);
    void resetFootTorqueStabilizer();
    void footTorqueStabilizer(Vector3d tauld, Vector3d taurd, Vector3d taul, Vector3d taur, bool right_contact, bool left_contact);
    void resetFootForceStabilizer();
    void footForceStabilizer(double flz, double frz, double flz_d, double frz_d);
    void resetBaseOrientationStabilizer();
    void baseOrientationStabilizer(Quaterniond qm, Quaterniond qref);
    Quaterniond getBaseOrientation();
    Quaterniond getLeftFootOrientation();
    Quaterniond getRightFootOrientation();
    Vector3d getRightFootVerticalPosition();
    Vector3d getLeftFootVerticalPosition();
    Matrix3d rotation_from_euler(double roll, double pitch, double yaw);
    inline Eigen::Vector3d logMap(const Eigen::Matrix3d &R_)
    {
        Eigen::Vector3d w;
        double acosV = (R_(0, 0) + R_(1, 1) + R_(2, 2) - 1.) * 0.5;
        double theta = std::acos(std::min(std::max(acosV, -1.), 1.));

        w = Eigen::Vector3d(R_(2, 1) - R_(1, 2), R_(0, 2) - R_(2, 0), R_(1, 0) - R_(0, 1));
        w *= sinc_inv(theta) * 0.5;

        return w;
    }
    double sinc_inv(const double x)
    {
    constexpr double taylor_0_bound = std::numeric_limits<double>::epsilon();
    constexpr double taylor_2_bound = std::sqrt(taylor_0_bound);
    constexpr double taylor_n_bound = std::sqrt(taylor_2_bound);

    // We use the 4th order taylor series around 0 of x/sin(x) to compute
    // this function:
    //
    //     x^2  7x^4
    // 1 + ── + ──── + O(x^6)
    //     6    360
    // this approximation is valid around 0.
    // if x is far from 0, our approximation is not valid
    // since x^6 becomes non neglectable we use the normal computation of the function
    // (i.e. taylor_2_bound^6 + taylor_0_bound == taylor_0_bound but
    //       taylor_n_bound^6 + taylor_0_bound != taylor_0).

    if(std::abs(x) >= taylor_n_bound)
    {
        return (x / std::sin(x));
    }
    else
    {
        // x is bellow taylor_n_bound so we don't care of the 6th order term of
        // the taylor series.
        // We set the 0 order term.
        double result = 1;

        if(std::abs(x) >= taylor_0_bound)
        {
        // x is above the machine epsilon so x^2 is meaningful.
        double x2 = x * x;
        result += x2 / 6;

        if(std::abs(x) >= taylor_2_bound)
        {
            // x is upper the machine sqrt(epsilon) so x^4 is meaningful.
            result += 7 * (x2 * x2) /360;
        }
        }

        return (result);
    }
    }

	/** @brief Computes the exponential map according to the Rodriquez Formula for component in so(3)
	 *  @param omega 3D twist in so(3) algebra
	 *  @return   3x3 Rotation in  SO(3) group
	 */
	inline Matrix<double, 3, 3> expMap(
			Vector3d omega) {

		Matrix<double, 3, 3> res;
		double omeganorm;

		omeganorm = omega.norm();
		res =  Matrix<double, 3, 3>::Identity();

		if(omeganorm > std::numeric_limits<double>::epsilon())
		{
			Matrix<double, 3, 3>  omega_skew;
			omega_skew = Matrix<double, 3, 3>::Zero();

			omega_skew = wedge(omega);
			res += omega_skew * (sin(omeganorm) / omeganorm);
			res += (omega_skew * omega_skew) * (
					(1.000 - cos(omeganorm)) / (omeganorm * omeganorm));
		}

		return res;
	}
    inline Eigen::Matrix3d wedge(Eigen::Vector3d v)
    {
        Eigen::Matrix3d skew;
        skew = Eigen::Matrix3d::Zero();
        skew(0, 1) = -v(2);
        skew(0, 2) = v(1);
        skew(1, 2) = -v(0);
        skew(1, 0) = v(2);
        skew(2, 0) = -v(1);
        skew(2, 1) = v(0);

        return skew;
    }

};
#endif
