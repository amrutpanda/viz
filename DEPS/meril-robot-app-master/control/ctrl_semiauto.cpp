/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "ctrl_semiauto.h"

namespace control {

SemiautoGenerator::SemiautoGenerator(unsigned int numberOfJoints)
    : posActual_(numberOfJoints), posTarget_(numberOfJoints), posSetpoint_(numberOfJoints),
      velSetpoint_(numberOfJoints), accSetpoint_(numberOfJoints) {}

bool SemiautoGenerator::initPhase1_() {
  targetHandle_ = addParameterVec("target", mcx::parameter_server::ParameterType::INPUT, posTarget_);

  addParameter("maxVel", mcx::parameter_server::ParameterType::PARAMETER, &maxVelocity_);
  addParameter("maxAcc", mcx::parameter_server::ParameterType::PARAMETER, &maxAcceleration_);

  addParameter("runSpeedFactorOut", mcx::parameter_server::ParameterType::OUTPUT, &runSpeedFactor_);
  addParameter("localTimeOut", mcx::parameter_server::ParameterType::OUTPUT, &localTime_);
  addParameter("engagedOut", mcx::parameter_server::ParameterType::OUTPUT, &engaged_);
  addParameter("motionGenStateOut", mcx::parameter_server::ParameterType::OUTPUT, &state_);

  addParameterVec("posOut", mcx::parameter_server::ParameterType::OUTPUT, posSetpoint_);
  addParameterVec("velOut", mcx::parameter_server::ParameterType::OUTPUT, velSetpoint_);
  addParameterVec("accOut", mcx::parameter_server::ParameterType::OUTPUT, accSetpoint_);

  return true;
}

bool SemiautoGenerator::iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) {
  using namespace mcx::motion;

  if ((!engagedPrev_ && engaged_) || targetHandle_.isUpdated()) {
    if (trajJointP2p_.compute(posActual_, posTarget_, maxVelocity_, maxAcceleration_)) {
      localTime_ = 0;
      state_ = MotionGeneratorStates::RUNNING_S;
    } else {
      log_error("Semi-auto failed to compute trajectory");
    }
  }

  if (state_ == MotionGeneratorStates::RUNNING_S) {
    if (!trajJointP2p_.update(localTime_, posSetpoint_, velSetpoint_, accSetpoint_)) {
      state_ = MotionGeneratorStates::MOTION_ALLOWED_S;
    } else {
      localTime_ += getDtSec() * runSpeedFactor_;
    }
  } else {
    posSetpoint_ = posActual_;
  }

  engagedPrev_ = engaged_;

  return true;
}

} // namespace control