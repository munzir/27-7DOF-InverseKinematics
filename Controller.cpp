/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.hpp"
//==============================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot,
                       dart::dynamics::BodyNode* _endEffector)
  : mRobot(_robot),
    mEndEffector(_endEffector)
{
  assert(_robot != nullptr);
  assert(_endEffector != nullptr);
  int dof = mRobot->getNumDofs();
  
  // ======================= Tunable Parameters
  bool isBetaConsistent;
  double currentLimitFactor;
  mKp.setZero();
  mKv.setZero();
  mKpOr.setZero();
  mZeroCol.setZero();
  mZero7Col.setZero();
  mKvJoint.setZero();
  mWReg.setZero();
  currLow << -9.5, -9.5, -7.5, -7.5, -5.5, -5.5, -5.5;
  currHigh << 9.5, 9.5, 7.5, 7.5, 5.5, 5.5, 5.5;


  Configuration * cfg = Configuration::create();
  const char * scope = "";
  const char * configFile = "../controlParams.cfg";
  const char * str;
  std::istringstream stream;
  double newDouble;
  
  try {
    cfg->parse(configFile);
    
    isBetaConsistent = cfg->lookupBoolean(scope, "betaConsistent");
    cout << "betaConsistent: " << (isBetaConsistent?"true":"false") << endl;

    currentLimitFactor = min(1.0, max(0.0, (double)cfg->lookupFloat(scope, "currentLimitFactor")));
    cout << "currentLimitFactor: " << currentLimitFactor << endl;
    mCurLim << 9.5, 9.5, 7.5, 7.5, 5.5, 5.5, 5.5;
    mCurLim *= currentLimitFactor;

    mWPos = cfg->lookupFloat(scope, "wPos");
    cout << "wPos: " << mWPos << endl;

    mGainFactor = cfg->lookupFloat(scope, "gainFactor");
    cout << "gainFactor: " << mGainFactor << endl;

    str = cfg->lookupString(scope, "Kp");
    stream.str(str); for(int i=0; i<3; i++) stream >> mKp(i, i); stream.clear(); mKp *= mGainFactor;
    cout << "Kp: " << mKp(0, 0) << ", " << mKp(1, 1) << ", " << mKp(2, 2) << endl;

    str = cfg->lookupString(scope, "Kv");
    stream.str(str); for(int i=0; i<3; i++) stream >> mKv(i, i); stream.clear(); mKv *= mGainFactor;
    cout << "Kv: " << mKv(0, 0) << ", " << mKv(1, 1) << ", " << mKv(2, 2) << endl;

    str = cfg->lookupString(scope, "KvJoint");
    stream.str(str); for(int i=0; i<7; i++) stream >> mKvJoint(i, i); stream.clear(); mKvJoint *= mGainFactor;
    cout << "KvJoint: " << mKvJoint(0, 0) << ", " << mKvJoint(1, 1) << ", " << mKvJoint(2, 2) 
                        << ", " << mKvJoint(3, 3) << ", " << mKvJoint(4 ,4) << ", " << mKvJoint(5, 5)
                        << ", " << mKvJoint(6, 6) << ", " << endl;

    mWOr = cfg->lookupFloat(scope, "wOr");
    cout << "wOr: " << mWOr << endl;

    str = cfg->lookupString(scope, "KpOr");
    stream.str(str); for(int i=0; i<3; i++) stream >> mKpOr(i, i); stream.clear(); mKpOr *= mGainFactor;
    cout << "KpOr: " << mKpOr(0, 0) << ", " << mKpOr(1, 1) << ", " << mKpOr(2, 2) << endl;

    str = cfg->lookupString(scope, "KvOr");
    stream.str(str); for(int i=0; i<3; i++) stream >> mKvOr(i, i); stream.clear(); mKvOr *= mGainFactor;
    cout << "KvOr: " << mKvOr(0, 0) << ", " << mKvOr(1, 1) << ", " << mKvOr(2, 2) << endl;

    str = cfg->lookupString(scope, "wReg");
    stream.str(str); for(int i=0; i<7; i++) stream >> mWReg(i, i); stream.clear();
    cout << "wReg: "; for(int i=0; i<6; i++) cout << mWReg(i, i) << ", "; cout << mWReg(6, 6) << endl;

    mKvReg = cfg->lookupFloat(scope, "KvReg"); mKvReg *= mGainFactor;
    cout << "KvReg: " << mKvReg << endl; 

    mdtFixed = cfg->lookupBoolean(scope, "dtFixed");
    cout << "dtFixed: " << (mdtFixed?"true":"false") << endl;

    mdt = cfg->lookupFloat(scope, "dt");
    cout << "dt: " << mdt << endl;

    mCompensateFriction = cfg->lookupBoolean(scope, "compensateFriction");
    cout << "compensateFriction: " << (mCompensateFriction?"true":"false") << endl;

    mPredictFriction = cfg->lookupBoolean(scope, "predictFriction");
    cout << "predictFriction: " << (mPredictFriction?"true":"false") << endl;

  } catch(const ConfigurationException & ex) {
    cerr << ex.c_str() << endl;
    cfg->destroy();
  }

  // ============================= load dynamic parameters and set them in the robot
  Eigen::MatrixXd beta 
      = readInputFileAsMatrix(isBetaConsistent?\
        "../../20c-RidgeRegression_arm/betaConsistent/betaConsistent.txt":\
        "../../20c-RidgeRegression_arm/betaFull/betaFull.txt");
  int numBodies = mRobot->getNumBodyNodes();
  int paramsPerBody = 13;
  mRotorInertia.setZero();
  mViscousFriction.setZero();
  mCoulombFriction.setZero();

  for(int i=1; i<numBodies; i++) {
    int ind = paramsPerBody*(i-1);
    double m = beta(ind + 0);
    Eigen::Vector3d mCOM = beta.block(ind+1, 0, 3, 1);
    double xx = beta(ind + 4);
    double yy = beta(ind + 5);
    double zz = beta(ind + 6);
    double xy = beta(ind + 7);
    double xz = beta(ind + 8);
    double yz = beta(ind + 9);

    // *** WARNING *** Adding these Beta Parameters results in failure in the simulation
    // mRobot->getBodyNode(i)->setMass(m);
    // mRobot->getBodyNode(i)->setLocalCOM(mCOM/m);
    // mRobot->getBodyNode(i)->setMomentOfInertia(xx, yy, zz, xy, xz, yz);

    mRotorInertia(i-1, i-1) = beta(ind + 10)*mGR_array[i-1]*mGR_array[i-1];
    mViscousFriction(i-1, i-1) = beta(ind + 11);
    mCoulombFriction(i-1, i-1) = beta(ind + 12);
  
    torqueLow(i-1) = mKm_array[i-1]*mGR_array[i-1]*currLow(i-1);
    torqueHigh(i-1) = mKm_array[i-1]*mGR_array[i-1]*currHigh(i-1);
  }

  // *** NOTE *** Beta Frictions have been divided by 10 for simulation to work
  //Set beta Coulomb/Viscous Frictions
  for (int i = 0; i < 7; i++){
	  std::size_t index = 0;
	  mRobot->getJoint(i)->setCoulombFriction(index,mCoulombFriction(i,i));
	  mRobot->getJoint(i)->setDampingCoefficient(index,mViscousFriction(i,i)/10);
  }

 
  // ============================= set mStep to zero
  mSteps = 0;
}

//==============================================================================
Controller::~Controller(){}

//==============================================================================

struct OptParams{
  Eigen::Matrix<double, -1, 7> P;
  Eigen::VectorXd b;
};

//========================================================================
void constraintFunc(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data) {
 
  OptParams* constParams = reinterpret_cast<OptParams *>(f_data);
  if (grad != NULL) {
    for(int i=0; i<m; i++) {
      for(int j=0; j<n; j++){
        grad[i*n+j] = constParams->P(i, j);
      }
    }
  }

  Eigen::Matrix<double, 7, 1> X;
  for(size_t i=0; i<n; i++) X(i) = x[i];
 
  Eigen::VectorXd mResult;
  mResult = constParams->P*X - constParams->b;
  for(size_t i=0; i<m; i++) {
    result[i] = mResult(i);
  }
}

//==============================================================================
double optFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
  OptParams* optParams = reinterpret_cast<OptParams *>(my_func_data);
  Eigen::Matrix<double, 7, 1> X(x.data());

  if (!grad.empty()) {
    Eigen::Matrix<double, 7, 1> mGrad = optParams->P.transpose()*(optParams->P*X - optParams->b);
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  return (0.5 * pow((optParams->P*X - optParams->b).norm(), 2));
}

Eigen::VectorXd sigmoid(Eigen::VectorXd x) {
  Eigen::VectorXd y = x;
  for(int i=0; i<x.size(); i++) {
    y(i) = 2/(1 + exp(-2*x(i))) - 1;
  }
  return y;
}

//==============================================================================
unsigned long Controller::get_time(){
  struct timeval tv;
  gettimeofday(&tv, NULL);
  unsigned long ret = tv.tv_usec;
  ret /= 10;
  ret += (tv.tv_sec*100000);
  return ret;

}

//==============================================================================
void Controller::update(const Eigen::Vector3d& _targetPosition, const Eigen::Vector3d& _targetRPY)
{
  mSteps++;

  if(mSteps == 1) {
    mStartTime = get_time();
  }

  using namespace dart;
  
  double currentTime = (get_time() - mStartTime)/100000.0;
  double dt = (mdtFixed? mdt : (currentTime - mPriorTime));


  // ============================ Optimizer ============================

  // End-effector Position
  Eigen::Vector3d x = mEndEffector->getTransform().translation();
  Eigen::Vector3d dx = mEndEffector->getLinearVelocity();
  Eigen::Vector3d dxref = -mKp*(x - _targetPosition);
  math::LinearJacobian Jv = mEndEffector->getLinearJacobian();       // 3 x n
  math::LinearJacobian dJv = mEndEffector->getLinearJacobianDeriv();  // 3 x n
  //Px - b = Jv*dq - dxref
  Eigen::Matrix<double, 3, 7> PPos = Jv;
  Eigen::Vector3d bPos = -(-dxref) ;

 // End-effector Orientation
  Eigen::Quaterniond quat(mEndEffector->getTransform().rotation());
  double quat_w = quat.w(); 
  Eigen::Vector3d quat_xyz(quat.x(), quat.y(), quat.z());
  if(quat_w < 0) {quat_w *= -1.0; quat_xyz *= -1.0;}
  Eigen::Quaterniond quatRef(Eigen::AngleAxisd(_targetRPY(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(_targetRPY(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(_targetRPY(2), Eigen::Vector3d::UnitZ()));
  double quatRef_w = quatRef.w(); 
  Eigen::Vector3d quatRef_xyz(quatRef.x(), quatRef.y(), quatRef.z());
  if(quatRef_w < 0) { quatRef_w *= -1.0; quatRef_xyz *= -1.0; }
  Eigen::Vector3d quatError_xyz = quatRef_w*quat_xyz - quat_w*quatRef_xyz + quatRef_xyz.cross(quat_xyz);
  Eigen::Vector3d w = mEndEffector->getAngularVelocity();
  Eigen::Vector3d dwref = -mKpOr*quatError_xyz - mKvOr*w;
  math::AngularJacobian Jw = mEndEffector->getAngularJacobian();       // 3 x n
  math::AngularJacobian dJw = mEndEffector->getAngularJacobianDeriv();  // 3 x n
  Eigen::Matrix<double, 3, 7> POr = Jw;
  Eigen::Vector3d bOr = -(dJw*mdq - dwref);
  

  // Speed Regulation
  /*
  Eigen::MatrixXd PReg = Eigen::Matrix<double, 7, 7>::Identity();
  Eigen::MatrixXd bReg = -mKvReg*mdq;
  */

   // Optimizer stuff
  nlopt::opt opt(nlopt::LD_SLSQP, 7);
  OptParams optParams;
  std::vector<double> dq_vec(7);
  double minf;

  // Perform optimization to find joint speeds
  Eigen::MatrixXd P(PPos.rows() + POr.rows() /*+ PReg.rows()*/, PPos.cols() );
  P << mWPos*PPos,
       mWOr*POr/*,
       mWReg*PReg*/;
  
  Eigen::VectorXd b(bPos.rows() + bOr.rows() /*+ bReg.rows()*/, bPos.cols() );
  b << mWPos*bPos,
       mWOr*bOr/*,
       mWReg*bReg*/;
       
  optParams.P = P;

  optParams.b = b;
  opt.set_min_objective(optFunc, &optParams);
  opt.set_xtol_rel(1e-4);
  opt.set_maxtime(0.005);
  opt.optimize(dq_vec, minf);
  


  // =============================== PID ==============================

  Eigen::Matrix<double, 7, 1> dq_target(dq_vec.data()); 
  Eigen::Matrix<double, 7, 1> dq = mRobot->getVelocities();
  
  dqref << 10, 0, 0, 0, 0, 0, 0;

  Eigen::Matrix<double, 7, 1> dq_cmd = mKvJoint*(dq_target - dq);



  Eigen::Matrix<double, 7, 1> torque_cmd;
  for(int i = 0; i<7; i++){
  	torque_cmd(i) = std::max(torqueLow(i), std::min(torqueHigh(i), dq_cmd(i))); 
  }
  


  // =========================== Set Forces ===========================

  cout << "Torque Command:  " << torque_cmd(0) << " " << torque_cmd(1) << "  " << torque_cmd(2) << "  " << torque_cmd(3) << "  " 
  							  << torque_cmd(4) << "  " << torque_cmd(5) << "  " << torque_cmd(6) << endl;
  
  mRobot->setForces(torque_cmd);

  mPriorTime = currentTime;
}

//==============================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const
{
  return mRobot;
}

//==============================================================================
dart::dynamics::BodyNode* Controller::getEndEffector() const
{
  return mEndEffector;
}

//==============================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/)
{
}

