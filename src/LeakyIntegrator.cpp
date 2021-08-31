#include <lipm_control/LeakyIntegrator.h>

LeakyIntegrator::LeakyIntegrator()
{
  integral_.setZero();
  rate_ = 0.1;
  saturation_ = false;
}
void LeakyIntegrator::add(const Vector3d &value, double dt)
{

  double leak_ = (rate_ * dt) > 1.0 ? 1.0 : (rate_ * dt); //For numerical stability
  integral_ = (1.0 - leak_) * integral_ + dt * value;
}

void LeakyIntegrator::saturate()
{
  if (integral_(0) < l_sat(0))
  {
    integral_(0) = l_sat(0);
  }
  else if (integral_(0) > u_sat(0))
  {
    integral_(0) = u_sat(0);
  }

  if (integral_(1) < l_sat(1))
  {
    integral_(1) = l_sat(1);
  }
  else if (integral_(1) > u_sat(1))
  {
    integral_(1) = u_sat(1);
  }
  if (integral_(2) < l_sat(2))
  {
    integral_(2) = l_sat(2);
  }
  else if (integral_(2) > u_sat(2))
  {
    integral_(2) = u_sat(2);
  }



}
