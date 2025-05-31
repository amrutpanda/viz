/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "ctrl_cartesianposeconstraint.h"
#include <utility>

using namespace mcx;

// public
CartesianPoseConstraint::CartesianPoseConstraint(std::string constraintName)
    : constraintName_(std::move(constraintName)) {}

math::Vector6D CartesianPoseConstraint::updateConstraint(const math::Vector6D& rawPose) {
  // This part would typically be in the iterateOP(). the calculateConstraint() is called every sample and updates the
  // constrainPose
  rawPose_ = rawPose;
  if (isEnabled_) {
    calculateConstraint();
  } else {
    setViolating(false);
    constrainedPose_ = rawPose;
    jumpCorrectedPose_ = rawPose;
  }
  return constrainedPose_;
}

// protected
void CartesianPoseConstraint::updateSoftBraking(double newPositionConverted) {
  isBraking_ = false;
  if (!brakingInitialized_) {
    jumpCorrectedPose_ = rawPose_;
    brakingInitialized_ = true;
  }

  if (enableSoftBraking_) {
    double limitingFactor = 0;
    // soft limit, only limit in case of going into the constraint
    if (constraintDirection_ == STAY_INSIDE) {
      // stay inside
      if ((newPositionConverted >= brakingPoint_) &&
          (newPositionConverted - constrainedPositionConverted_ > std::numeric_limits<double>::epsilon()) &&
          (newPositionConverted <= constraintBoundary_)) {
        limitingFactor = 1.0 - (newPositionConverted - brakingPoint_) / brakingRange_;
        if (limitingFactor < 0) {
          limitingFactor = 0;
        }
        isBraking_ = true;
      }
    } else {
      // stay outside
      if ((newPositionConverted <= brakingPoint_) &&
          (newPositionConverted - constrainedPositionConverted_ < -std::numeric_limits<double>::epsilon()) &&
          (newPositionConverted >= constraintBoundary_)) {
        limitingFactor = 1.0 - (brakingPoint_ - newPositionConverted) / brakingRange_;
        if (limitingFactor < 0) {
          limitingFactor = 0;
        }
        isBraking_ = true;
      }
    }

    if (isBraking() || isViolating()) {
      math::Vector6D deltaPose = constrainedPose_ - jumpCorrectedPose_;
      const double deltaMagnitude =
          std::sqrt(deltaPose[0] * deltaPose[0] + deltaPose[1] * deltaPose[1] + deltaPose[2] * deltaPose[2]);
      if (const double stepSize =
              maxPositionJump_ + (deltaMagnitude - maxPositionJump_) * limitingFactor * limitingFactor;
          stepSize != 0) {
        if (const double deltaRatio = deltaMagnitude / stepSize; deltaRatio > 1) {
          for (size_t i = 0; i < 3; i++) {
            jumpCorrectedPose_[i] += deltaPose[i] / deltaRatio;
          }
        } else {
          jumpCorrectedPose_ = constrainedPose_;
        }
      }
    } else {
      jumpCorrectedPose_ = constrainedPose_;
    }
  } else {
    jumpCorrectedPose_ = constrainedPose_;
  }
  constrainedPose_ = jumpCorrectedPose_;
}

void CartesianPoseConstraint::create_(const char* name, parameter_server::Parameter* parameterServer,
                                      uint64_t dtMicroS) {}

bool CartesianPoseConstraint::initPhase1_() {
  using namespace mcx::parameter_server;
  // Input
  addParameter("disable", ParameterType::INPUT, &disable_);

  // Parameter
  addParameter("enable", ParameterType::PARAMETER, &enable_);
  addParameter("constraintDirection", ParameterType::PARAMETER, &constraintDirection_);
  addParameter("constraintBoundary", ParameterType::PARAMETER, &constraintBoundary_);
  addParameter("softBraking/enableSoftBraking", ParameterType::PARAMETER, &enableSoftBraking_);
  addParameter("softBraking/brakingRange", ParameterType::PARAMETER, &brakingRange_);
  addParameter("softBraking/maxPositionJump", ParameterType::PARAMETER, &maxPositionJump_);

  // Output
  addParameter("isEnabled", ParameterType::OUTPUT, &isEnabled_);
  addParameter("isViolating", ParameterType::OUTPUT, &isViolating_);
  addParameter("isBraking", ParameterType::OUTPUT, &isBraking_);
  addParameter("referencePose", ParameterType::OUTPUT, referencePose_.data(), referencePose_.size());
  addParameter("rawPose", ParameterType::OUTPUT, rawPose_.data(), rawPose_.size());
  addParameter("constrainedPose", ParameterType::OUTPUT, constrainedPose_.data(), constrainedPose_.size());
  addParameter("softBraking/jumpCorrectedPose", ParameterType::OUTPUT, jumpCorrectedPose_.data(),
               jumpCorrectedPose_.size());
  addParameter("constrainedPositionConverted", ParameterType::OUTPUT, &constrainedPositionConverted_);
  addParameter("limitingVector", ParameterType::OUTPUT, limitingVector_.data(), limitingVector_.size());
  return true;
}

bool CartesianPoseConstraint::initPhase2_() { return true; }

bool CartesianPoseConstraint::startOp_() { return true; }

bool CartesianPoseConstraint::stopOp_() { return true; }

bool CartesianPoseConstraint::iterateOp_(const container::TaskTime& systemTime, container::UserTime* userTime) {
  isEnabled_ = enable_ && !disable_;
  if ((constraintDirection_ != STAY_INSIDE) && (constraintDirection_ != STAY_OUTSIDE)) {
    constraintDirection_ = STAY_INSIDE;
  }

  if (constraintDirection_ == STAY_INSIDE) {
    brakingPoint_ = constraintBoundary_ - brakingRange_;
  } else {
    brakingPoint_ = constraintBoundary_ + brakingRange_;
  }

  isLimiting_ = isViolating_ | isBraking_;

  calculateLimitingVector();

  calculateConvertedConstrainedPosition();

  // check if the target position is currently in the braking area
  if (constraintDirection_ == STAY_INSIDE) {
    // stay inside
    if (constrainedPositionConverted_ >= brakingPoint_) {
      isInBrakingArea_ = true;
    }
  } else {
    // stay outside
    if (constrainedPositionConverted_ <= brakingPoint_) {
      isInBrakingArea_ = true;
    }
  }

  return true;
}
