/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "ctrl_cartposeratelimiter.h"

#include <any>

using namespace mcx;

void CartPoseRateLimiter::create_(const char* name, parameter_server::Parameter* parameterServer, uint64_t dtMicroS) {}

bool CartPoseRateLimiter::initPhase1_() {
  using namespace mcx::parameter_server;
  addParameterVec("input", ParameterType::INPUT, cartPoseRaw_);

  addParameter("maxTranslationalRate", ParameterType::PARAMETER, &maxTranslationalRate_);
  addParameter("maxRotationalRate", ParameterType::PARAMETER, &maxRotationalRate_);
  addParameter("omega", ParameterType::PARAMETER, &omega_);

  addParameter("isLimiting", ParameterType::OUTPUT, &isLimiting_);
  addParameter("magnitude", ParameterType::OUTPUT, &magnitude_);
  addParameterVec("output", ParameterType::OUTPUT, cartPoseRateLimited_);
  return true;
}

bool CartPoseRateLimiter::initPhase2_() { return true; }

bool CartPoseRateLimiter::startOp_() { return true; }

bool CartPoseRateLimiter::stopOp_() { return true; }

bool CartPoseRateLimiter::iterateOp_(const container::TaskTime& systemTime, container::UserTime* userTime) {
  // set the limited pose to the reference pose if reset is true
  if (reset_) {
    poseRateLimited_ = poseReference_;
    reset_ = false;
  }

  const double dt = getDtSec();

  isLimiting_ = false;
  // limit position
  const math::Position positionRaw = poseRaw_.getPosition();
  math::Position positionRateLimited = poseRateLimited_.getPosition();
  const math::Position positionDelta = positionRaw - positionRateLimited;

  double magnitude = positionDelta.norm();
  magnitude_ = positionDelta.norm();
  // const double fractionPosition = std::min(std::fabs(maxTranslationalRate_ * dt / magnitude_), 1.0);
  math::Position fractionPosition = {1.0, 1.0, 1.0};
  for (int i = 0; i < 3; i++) {
    magnitude = std::fabs(positionDelta[i]) / dt;
    if (magnitude > maxTranslationalRate_) {
      fractionPosition[i] = std::min(maxTranslationalRate_ / magnitude, 1.0);
      isLimiting_ = true;
    }
  }

  const double alfa = omega_ * dt / (1 + omega_ * dt);
  velocity_ += ((positionDelta * fractionPosition) - velocity_) * alfa; // lowpass-1 filter
  positionRateLimited += velocity_;

  poseRateLimited_.setPosition(positionRateLimited);

  // limit rotation
  const math::Quaternion rotationRaw = poseRaw_.getRotation().getQuaternion();
  math::Quaternion rotationRateLimited = poseRateLimited_.getRotation().getQuaternion();
  const math::Quaternion rotationDelta = rotationRateLimited.transpose() * rotationRaw;

  if (const double theta = math::normalizeAngle(rotationDelta.getAngle(), 0);
      std::fabs(theta) >= std::numeric_limits<double>::epsilon()) {
    if (const double fractionRotation = std::fabs(maxRotationalRate_ * getDtSec() / theta); fractionRotation < 1.0) {
      rotationRateLimited = rotationRateLimited.slerp(rotationRaw, fractionRotation, false);
      isLimiting_ = true;
    } else {
      rotationRateLimited = rotationRaw;
    }
  } else {
    rotationRateLimited = rotationRaw;
  }

  poseRateLimited_.setRotation(math::Rotation(rotationRateLimited));

  // update IO
  cartPoseRaw_ = poseRaw_.getCartPose6();
  cartPoseRateLimited_ = poseRateLimited_.getCartPose6();

  return true;
}
