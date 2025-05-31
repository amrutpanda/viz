/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "ctrl_cartesianplaneconstraint.h"

using namespace mcx;
// public:
CartesianPlaneConstraint::CartesianPlaneConstraint(const std::string& constraintName)
    : CartesianPoseConstraint(constraintName) {}

void CartesianPlaneConstraint::calculateConstraint() {
  setViolating(false);
  math::Vector6D referencePoseInertialFrame = getReferencePose();
  math::Vector6D rawPoseInertialFrame = getRawPose();

  const math::Rotation rotRefFrame2InertialFrame(referencePoseInertialFrame);
  const math::Rotation rotInertialFrame2RefFrame(rotRefFrame2InertialFrame.getMatrix().transpose());

  math::Vector3D refPositionRefFrame = rotInertialFrame2RefFrame.getMatrix().dot(
      math::Vector3D(referencePoseInertialFrame[0], referencePoseInertialFrame[1], referencePoseInertialFrame[2]));
  math::Vector3D rawPositionRefFrame = rotInertialFrame2RefFrame.getMatrix().dot(
      math::Vector3D(rawPoseInertialFrame[0], rawPoseInertialFrame[1], rawPoseInertialFrame[2]));

  const double distance = rawPositionRefFrame[2] - refPositionRefFrame[2];
  math::Vector6D constrainedPoseInertialFrame = rawPoseInertialFrame;
  static constexpr auto EPS = 1e-9;
  if (std::fabs(distance - getConstraintBoundary()) > EPS) {
    if (((getConstraintDirection() == STAY_INSIDE) && (distance > getConstraintBoundary())) ||
        ((getConstraintDirection() == STAY_OUTSIDE) && (distance < getConstraintBoundary()))) {
      setViolating(true);
      math::Vector3D constrainedPositionRefFrame = rawPositionRefFrame;
      constrainedPositionRefFrame[2] = refPositionRefFrame[2] + getConstraintBoundary();
      math::Vector3D constrainedPositionInertialFrame =
          rotRefFrame2InertialFrame.getMatrix().dot(constrainedPositionRefFrame);
      for (size_t i = 0; i < 3; i++) {
        constrainedPoseInertialFrame[i] = constrainedPositionInertialFrame[i];
      }
    }
  }
  setConstrainedPose(constrainedPoseInertialFrame);

  updateSoftBraking(distance);
}

void CartesianPlaneConstraint::calculateLimitingVector() {
  // get the rotations
  const math::Vector6D referencePoseInertialFrame = getReferencePose();

  const math::Rotation rotRefFrame2InertialFrame(referencePoseInertialFrame);

  // get the raw vector
  math::Vector3D limitingVectorRefFrame(0, 0, -1);

  // set the vector according to the constraint direction
  if (getConstraintDirection() == STAY_OUTSIDE) {
    limitingVectorRefFrame[2] = 1;
  }

  // transform the vector back to the inertial frame
  const math::Vector3D limitingVectorInertialFrame = rotRefFrame2InertialFrame.getMatrix().dot(limitingVectorRefFrame);
  // normalize the vector
  limitingVectorRefFrame.normalize();

  setLimitingVector(limitingVectorInertialFrame);
}

void CartesianPlaneConstraint::calculateConvertedConstrainedPosition() {
  math::Vector6D referencePoseInertialFrame = getReferencePose();
  math::Vector6D constrainedPoseInertialFrame = getConstrainedPose();

  const math::Rotation rotRefFrame2InertialFrame(referencePoseInertialFrame);
  const math::Rotation rotInertialFrame2RefFrame(rotRefFrame2InertialFrame.getMatrix().transpose());

  math::Vector3D refPositionRefFrame = rotInertialFrame2RefFrame.getMatrix().dot(
      math::Vector3D(referencePoseInertialFrame[0], referencePoseInertialFrame[1], referencePoseInertialFrame[2]));
  math::Vector3D constrainedPositionRefFrame = rotInertialFrame2RefFrame.getMatrix().dot(math::Vector3D(
      constrainedPoseInertialFrame[0], constrainedPoseInertialFrame[1], constrainedPoseInertialFrame[2]));

  const double distance = constrainedPositionRefFrame[2] - refPositionRefFrame[2];

  setConvertedConstrainedPosition(distance);
}

// protected
bool CartesianPlaneConstraint::initPhase1_() {
  using namespace mcx::parameter_server;
  CartesianPoseConstraint::initPhase1_();

  return true;
}
