/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "ctrl_fulcrumteaching.h"

using namespace mcx;

void FulcrumTeaching::create_(const char* name, parameter_server::Parameter* parameterServer, uint64_t dtMicroS) {}

bool FulcrumTeaching::initPhase1_() {
  using namespace mcx::parameter_server;
  addParameter("engage", ParameterType::PARAMETER, &engage_);

  addParameter("numberOfCalculatedPoints", ParameterType::OUTPUT, &numberOfCalculatedPoints_);
  addParameter("calculatedFulcrumPose", ParameterType::OUTPUT, calculatedFulcrumPose_.data(),
               calculatedFulcrumPose_.size());

  return true;
}

bool FulcrumTeaching::initPhase2_() { return true; }

bool FulcrumTeaching::startOp_() { return true; }

bool FulcrumTeaching::stopOp_() { return true; }

bool FulcrumTeaching::iterateOp_(const container::TaskTime& systemTime, container::UserTime* userTime) {
  if (engage_) {
    const math::Vector3D manipulatorPositionNew{manipulatorPoseNew_[0], manipulatorPoseNew_[1], manipulatorPoseNew_[2]};
    const math::Vector3D manipulatorPositionOld{manipulatorPoseOld_[0], manipulatorPoseOld_[1], manipulatorPoseOld_[2]};
    const math::Rotation rotManipulator2InertialNew(manipulatorPoseNew_);
    const math::Rotation rotManipulator2InertialOld(manipulatorPoseOld_);

    // some shortcuts
    const math::Vector3D vectorNew = rotManipulator2InertialNew.getMatrix().dot(linkPoseLocal_);
    const math::Vector3D vectorOld = rotManipulator2InertialOld.getMatrix().dot(linkPoseLocal_);

    // don't calculate if the manipulator does not move enough
    const math::Vector3D increment = vectorNew - vectorOld;
    static constexpr auto EPS = 1e-4;
    if (increment.norm() > EPS) {
      const math::Vector3D positionDiff = manipulatorPositionNew - manipulatorPositionOld;
      const double pVnVo = vectorNew.dot(vectorOld);
      const double pVnVd = vectorNew.dot(positionDiff);
      const double pVoVd = vectorOld.dot(positionDiff);
      const double pVnVn = vectorNew.dot(vectorNew);
      const double pVoVo = vectorOld.dot(vectorOld);

      const double den = -pVnVn * pVoVo + pVnVo * pVnVo;

      const double ln = (pVnVd * pVoVo - pVoVd * pVnVo) / den;
      const double lo = (-pVoVd * pVnVn + pVnVd * pVnVo) / den;
      // discard the point if it is too much longer than the instrument length, else save the mean
      if ((ln > 0) && (lo > 0) && (ln < 1.5) && (lo < 1.5)) {
        numberOfCalculatedPoints_++;

        const math::Vector3D resultVn = manipulatorPositionNew + vectorNew * ln;
        const math::Vector3D resultVo = manipulatorPositionOld + vectorOld * lo;

        sumCalculatedPosition_ = sumCalculatedPosition_ + (resultVn + resultVo) * 0.5;
      }
    }

    if (numberOfCalculatedPoints_ >= 10000) {
      calculatedFulcrumPose_ = manipulatorPoseNew_;
      math::Vector3D calculatedFulcrumPosition = sumCalculatedPosition_ * (1.0 / numberOfCalculatedPoints_);
      for (size_t i = 0; i < 3; i++) {
        calculatedFulcrumPose_[i] = calculatedFulcrumPosition[i];
      }
      sumCalculatedPosition_ = 0;
      numberOfCalculatedPoints_ = 0;
      engage_ = false;
    }
  }
  manipulatorPoseOld_ = manipulatorPoseNew_;
  return true;
}
