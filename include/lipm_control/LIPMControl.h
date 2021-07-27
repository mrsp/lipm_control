class LIPMControl
{
  Vector3d CoM_c, dCoM_c, ddCoM_c, dCoM_c_;
  Vector3d dCoM_d_, ZMP_d, ZMP_c;
  Vector3d DCM_ref, dDCM_ref, DCM, dDCM;
  Vector3d sumDCM;
  double  g, h, mass, dt;
  bool initialized;

public:
  Matrix3d Kzmp, Kdcm, Kcom, Kddcm, Kidcm;
  double omega;
  LIPMControl(double h_, double mass_, double dt_)
  {
    dt = dt_;
    mass = mass_;
    h = h_;
    g = 9.81;
    omega = sqrt(g / h);
    initialized = false;
    Kzmp.setZero();
    Kcom.setZero();
    Kdcm.setZero();
    Kidcm.setZero();
    Kddcm.setZero();
    sumDCM.setZero();


    Kzmp(0, 0) = 7 / omega;
    Kzmp(1, 1) = 7 / omega;

    Kcom(0, 0) = 0.02;
    Kcom(1, 1) = 0.01;
    Kcom(2, 2) = 0.00;

    Kdcm(0, 0) = 1.0 + 40 / omega;
    Kdcm(1, 1) = 1.0 + 40 / omega;

    Kddcm(0, 0) = 15 / omega;
    Kddcm(1, 1) = 15 / omega;

    Kidcm(0, 0) = 10 / omega;
    Kidcm(1, 1) = 10 / omega;

    ddCoM_c.setZero();
    dCoM_c_.setZero();
    DCM.setZero();
    DCM_ref.setZero();
    dDCM_ref.setZero();
    dDCM.setZero();
  }

  void Control(Vector3d ZMP, Vector3d CoM, Vector3d dCoM, Vector3d ddCoM, Vector3d CoM_d, Vector3d dCoM_d, Vector3d ddCoM_d)
  {
    if (!initialized)
    {
      CoM_c = CoM_d;
      dCoM_c = dCoM_d;
      initialized = true;
      return;
    }

    //Compute Desired ZMP based on the LIPM
    ZMP_d = CoM_d - ddCoM_d / (omega * omega);
    ZMP_d(2) = ZMP(2);

    DCM_ref = computeDCM(CoM_d, dCoM_d);
    dDCM_ref = computeDCM(dCoM_d, ddCoM_d);
    DCM = computeDCM(CoM, dCoM);
    dDCM = computeDCM(dCoM, ddCoM);

    //Compute Command ZMP
    //ZMP_c = ZMP_d - Kdcm * (DCM_ref - DCM) - Kddcm * (dDCM_ref - dDCM) - Kidcm * sumDCM; 
    ZMP_c = ZMP_d - Kdcm * (DCM_ref - DCM) + Kzmp  * (ZMP_d - ZMP) - Kidcm * sumDCM;
    sumDCM += (DCM_ref - DCM) * dt;
    // cout<<"Desired ZMP "<<ZMP_d.transpose()<< "CMD ZMP "<<ZMP_c.transpose()<<" Measured ZMP "<<ZMP.transpose()<<endl;
    // cout<<"Desired CoM "<<CoM_d.transpose()<<" Measured CoM "<<CoM.transpose()<< "Desired acc "<< ddCoM_d.transpose()<<endl;

    //Compute Command CoM
    dCoM_c = dCoM_d - Kcom * (ZMP_c - ZMP);
    ddCoM_c = (dCoM_c - dCoM_c_) / dt;
    dCoM_c_ = dCoM_c;

    CoM_c = CoM_c + dCoM_c * dt;
    // cout<<"Command CoM "<<CoM_c.transpose()<<" Measured CoM "<<CoM.transpose()<<endl;
    // cout<<"Desired DCM "<<DCM_ref.transpose()<<" Measured DCM "<<DCM.transpose()<<endl;
  }

  Vector3d computeDCM(Vector3d CoM, Vector3d dCoM)
  {
    return CoM + 1 / omega * dCoM;
  }

  Vector3d getDesiredCoMPosition()
  {
    return CoM_c;
  }
  Vector3d getDesiredCoMVelocity()
  {
    return dCoM_c;
  }
  Vector3d getDesiredCoMAcceleration()
  {
    return ddCoM_c;
  }
  Vector3d getDesiredZMP()
  {
    return ZMP_c;
  }
};