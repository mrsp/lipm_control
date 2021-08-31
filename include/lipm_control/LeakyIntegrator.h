#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

class LeakyIntegrator
{
public:
  /** Add constant input for a fixed duration.
   *
   * \param value Constant input.
   *
   * \param dt Fixed duration.
   *
   */
  void add(const Vector3d &value, double dt);

  /** Evaluate the output of the integrator.
   *
   */
  const Vector3d &eval() const
  {
    return integral_;
  }

  LeakyIntegrator();
  /** Get leak rate.
   *
   */
  inline double rate() const
  {
    return rate_;
  }

  /** Set output saturation. Disable by providing a negative value.
   *
   * \param sat_ Output will saturate between -sat_ and +sat_.
   *
   */
  inline void setSaturation(Vector3d u_sat_, Vector3d l_sat_)
  {
    saturation_ = true;
    u_sat = u_sat_;
    l_sat = l_sat_;
  }

  /** Reset integral to zero.
   *
   */
  inline void setZero()
  {
    integral_.setZero();
  }
  inline void setInitialState(Vector3d value_)
  {
    integral_ = value_;
  }

  /** Set the leak rate of the integrator.
   *
   * \param rate New leak rate. When rate = 0 LeakyIntegrator turns to a common integrator
   *
   */
  inline void setRate(double rate)
  {
    rate_ = rate;
  }

private:
  void saturate();
  Vector3d integral_;
  double rate_;
  bool saturation_;
  Vector3d u_sat, l_sat;
};
