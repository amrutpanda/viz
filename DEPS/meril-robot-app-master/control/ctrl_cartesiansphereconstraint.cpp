/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "ctrl_cartesiansphereconstraint.h"

using namespace mcx;
// public:
CartesianSphereConstraint::CartesianSphereConstraint(const std::string& constraintName)
    : CartesianPoseConstraint(constraintName) {}

void CartesianSphereConstraint::calculateConstraint() {
  setViolating(false);
  math::Vector6D referencePoseInertialFrame = getReferencePose();
  math::Vector6D rawPoseInertialFrame = getRawPose();

  const math::Rotation rotRefFrame2InertialFrame(referencePoseInertialFrame);
  const math::Rotation rotInertialFrame2RefFrame(rotRefFrame2InertialFrame.getMatrix().transpose());

  const math::Vector3D referencePositionRefFrame = rotInertialFrame2RefFrame.getMatrix().dot(
      math::Vector3D(referencePoseInertialFrame[0], referencePoseInertialFrame[1], referencePoseInertialFrame[2]));

  const math::Vector3D rawPositionRefFrame = rotInertialFrame2RefFrame.getMatrix().dot(
      math::Vector3D(rawPoseInertialFrame[0], rawPoseInertialFrame[1], rawPoseInertialFrame[2]));

  const math::Vector3D positionDifference = rawPositionRefFrame - referencePositionRefFrame;
  const double radius = positionDifference.norm();

  math::Vector3D constrainedPositionRefFrame = rawPositionRefFrame;
  if (getConstraintBoundary() == 0) {
    constrainedPositionRefFrame = referencePositionRefFrame;
    setViolating(true);
  } else {
    if (double ratio = radius / getConstraintBoundary(); std::fabs(ratio - 1.0) > 1e-9) {
      if (((getConstraintDirection() == STAY_INSIDE) && (ratio > 1.0)) ||
          ((getConstraintDirection() == STAY_OUTSIDE) && (ratio < 1.0))) {
        if (ratio == 0.0) {
          ratio = 1.0;
        }
        constrainedPositionRefFrame = positionDifference * (1.0 / ratio) + referencePositionRefFrame;
        setViolating(true);
      }
    }
  }

  math::Vector6D constrainedPoseInertialFrame = rawPoseInertialFrame;
  if (isViolating()) {
    math::Vector3D constrainedPositionInertialFrame =
        rotRefFrame2InertialFrame.getMatrix().dot(constrainedPositionRefFrame);
    for (size_t i = 0; i < 3; i++) {
      constrainedPoseInertialFrame[i] = constrainedPositionInertialFrame[i];
    }
  }
  setConstrainedPose(constrainedPoseInertialFrame);

  updateSoftBraking(radius);
}

void CartesianSphereConstraint::calculateLimitingVector() {
  // get the rotations
  math::Vector6D referencePoseInertialFrame = getReferencePose();
  math::Vector6D constrainedPoseInertialFrame = getConstrainedPose();

  const math::Rotation rotRefFrame2InertialFrame(referencePoseInertialFrame);
  const math::Rotation rotInertialFrame2RefFrame(rotRefFrame2InertialFrame.getMatrix().transpose());

  const math::Vector3D referencePositionRefFrame = rotInertialFrame2RefFrame.getMatrix().dot(
      math::Vector3D(referencePoseInertialFrame[0], referencePoseInertialFrame[1], referencePoseInertialFrame[2]));

  const math::Vector3D constrainedPositionRefFrame = rotInertialFrame2RefFrame.getMatrix().dot(math::Vector3D(
      constrainedPoseInertialFrame[0], constrainedPoseInertialFrame[1], constrainedPoseInertialFrame[2]));

  // get the raw vector
  math::Vector3D limitingVectorRefFrame = referencePositionRefFrame - constrainedPositionRefFrame;

  // set the vector according to the constraint direction
  if (getConstraintDirection() == STAY_OUTSIDE) {
    limitingVectorRefFrame = limitingVectorRefFrame * -1;
  }

  // transform the vector back to the inertial frame
  const math::Vector3D limitingVectorInertialFrame = rotRefFrame2InertialFrame.getMatrix().dot(limitingVectorRefFrame);

  // normalize the vector
  limitingVectorRefFrame.normalize();

  setLimitingVector(limitingVectorInertialFrame);
}

void CartesianSphereConstraint::calculateConvertedConstrainedPosition() {
  math::Vector6D referencePose = getReferencePose();
  math::Vector6D constrainedPose = getConstrainedPose();

  const math::Vector3D referencePosition(referencePose[0], referencePose[1], referencePose[2]);

  const math::Vector3D constrainedPosition(constrainedPose[0], constrainedPose[1], constrainedPose[2]);

  const math::Vector3D positionDifference = constrainedPosition - referencePosition;

  setConvertedConstrainedPosition(positionDifference.norm());
}

// protected
bool CartesianSphereConstraint::initPhase1_() {
  using namespace mcx::parameter_server;
  CartesianPoseConstraint::initPhase1_();
  return true;
}
