/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "ctrl_instrumentcontrol.h"
#include "ctrl_cartesiancylinderconstraint.h"
#include "ctrl_cartesiansphereconstraint.h"
#include "mcx/control3/ctrl_helper.h"

using namespace mcx;

namespace control {

InstrumentControl::InstrumentControl() {
  // set the number of joints
  jointPositions_.resize(NR_OF_INSTRUMENT_JOINTS);
  jointPositionsUpperLimit_.resize(NR_OF_INSTRUMENT_JOINTS);
  jointPositionsLowerLimit_.resize(NR_OF_INSTRUMENT_JOINTS);
  // construct constraints
  cartPoseConstraints_["cylinder"] = std::make_unique<CartesianCylinderConstraint>("cylinderConstraint");
  cartPoseConstraints_["maxInsertionDepth"] =
      std::make_unique<CartesianSphereConstraint>("maxInsertionDepthConstraint");
  cartPoseConstraints_["minInsertionDepth"] =
      std::make_unique<CartesianSphereConstraint>("minInsertionDepthConstraint");
  cartPoseConstraints_["plane"] = std::make_unique<CartesianPlaneConstraint>("planeConstraint");
}

void InstrumentControl::updateInverseKin(const math::CartPose6& instrumentToolPoseTarget,
                                         math::CartPose6& manipulatorToolPoseTarget,
                                         control3::JointPositions& instrumentJointPositionsTarget,
                                         math::CartPose6& instrumentToolPoseCorrectedTarget) {
  // update the instrument tip position target for constraints
  instrumentToolPoseCorrectedTarget = updateInstrumentConstraints(instrumentToolPoseTarget);

  // update inverse kinematics
  calculateInverseKin(instrumentToolPoseCorrectedTarget, manipulatorToolPoseTarget, instrumentJointPositionsTarget);
}

void InstrumentControl::updateForwardKin(const math::CartPose6& manipulatorToolPoseActual,
                                         const control3::JointPositions& instrumentJointPositionsActual,
                                         math::CartPose6& instrumentToolPoseActual) {
  // update pose matrix
  math::Pose robotPose(manipulatorToolPoseActual);

  math::Pose rollPose(0, math::Rotation(instrumentJointPositionsActual[3], 0, 0));

  math::Pose wristPose(link1TipPoseLocal_, math::Rotation(0, instrumentJointPositionsActual[0], 0));

  math::Pose toolPose(link2TipPoseLocal_, math::Rotation(0, 0, instrumentJointPositionsActual[1]));

  math::Pose tipPose(link3TipPoseLocal_, math::Rotation(0, 0, 0));

  // get roll joint pos in the inertial reference frame
  math::Pose rollPoseIF;
  rollPoseIF.setHomogeneous(robotPose.getHomogeneous().dot(rollPose.getHomogeneous()));

  // get wrist joint pos in the inertial reference frame
  math::Pose wristPoseIF;
  wristPoseIF.setHomogeneous(rollPoseIF.getHomogeneous().dot(wristPose.getHomogeneous()));

  // get tool joint pos in the inertial reference frame
  math::Pose toolPoseIF;
  toolPoseIF.setHomogeneous(wristPoseIF.getHomogeneous().dot(toolPose.getHomogeneous()));

  // update the point where fulcrum position is taught (or take manual input)

    if (fulcrumTeachSetting_ == fulcrumTeachSetting::LINK1TIP) {
      fulcrumReferenceLocal_[2] = getLink1Length();
    } else if (fulcrumTeachSetting_ == fulcrumTeachSetting::LINK2TIP) {
      fulcrumReferenceLocal_[2] = getLink1Length() + getLink2Length();
    } else if (fulcrumTeachSetting_ == fulcrumTeachSetting::LINK3TIP) {
      fulcrumReferenceLocal_[2] = getLink1Length() + getLink2Length() + getLink3Length();
    }

  // get fulcrum reference frame in the inertial reference frame
  math::Pose fulcPoseIF;
  fulcPoseIF.setHomogeneous(robotPose.getHomogeneous().dot(math::Pose(fulcrumReferenceLocal_).getHomogeneous()));
  fulcrumReferenceActual_ = fulcPoseIF.getVector6D();

  if (teachFulcrum_) {
    cartPoseConstraints_["maxInsertionDepth"]->setConstraintBoundary(DEFAULT_MAX_INSERTION_DEPTH);
    cartPoseConstraints_["minInsertionDepth"]->setConstraintBoundary(DEFAULT_MIN_INSERTION_DEPTH);
    fulcrumPose_ = fulcrumReferenceActual_;
    fulcrumIsStored_ = true;
    teachFulcrum_ = false;

    // calculate the difference between local and global fulcrum pose. this dPose
    for (size_t idx = 0; idx < fulcrumPortPositionOffset_.size(); idx++) {
      fulcrumPortPositionOffset_[idx] = fulcrumPose_[idx] - fulcrumPortPosition_[idx];
    }
  }

  // get tool joint pos in the inertial reference frame
  // tipPoseIF.setHomogeneous(robotPose.getHomogeneous().dot(wristPose.getHomogeneous().dot(toolPose.getHomogeneous().dot(tipPose_.getHomogeneous()))));
  math::Pose tipPoseIF;
  tipPoseIF.setHomogeneous(toolPoseIF.getHomogeneous().dot(tipPose.getHomogeneous()));

  // set the instrument tool pose
  if (decoupleInstrumentKinematics_) {
    instrumentToolPoseActual[0] = wristPoseIF.getX();
    instrumentToolPoseActual[1] = wristPoseIF.getY();
    instrumentToolPoseActual[2] = wristPoseIF.getZ();
    instrumentToolPoseActual[3] = toolPoseIF.getPsi();
    instrumentToolPoseActual[4] = toolPoseIF.getTheta();
    instrumentToolPoseActual[5] = toolPoseIF.getPhi();
  } else {
    if (pointOfInterestAtTip_) {
      instrumentToolPoseActual = tipPoseIF.getCartPose6();
    } else {
      instrumentToolPoseActual = toolPoseIF.getCartPose6();
    }
  }
}

void InstrumentControl::calculateInverseKin(math::CartPose6& instrumentToolPoseTarget,
                                            math::CartPose6& manipulatorToolPoseTarget,
                                            control3::JointPositions& instrumentJointPositionsTarget) {
  if (decoupleInstrumentKinematics_) {
    /// decoupled translation and rotation kinematics solver
    // calculate position
    mcx::math::Position instrumentPosition =
        mcx::math::Position(instrumentToolPoseTarget[0], instrumentToolPoseTarget[1], instrumentToolPoseTarget[2]);
    mcx::math::Position fulcrumPosition = mcx::math::Position(fulcrumPose_[0], fulcrumPose_[1], fulcrumPose_[2]);
    mcx::math::Position vecFulcrumToInstrument = instrumentPosition - fulcrumPosition;
    mcx::math::Quaternion rotManipulatorToInertialCorrected{manipulatorToolPoseTarget[3], manipulatorToolPoseTarget[4],
                                                            manipulatorToolPoseTarget[5]};

    if (double lenFulcrumToInstrument = vecFulcrumToInstrument.norm();
        lenFulcrumToInstrument >= std::numeric_limits<double>::epsilon()) {
      double ratio = (getLink1Length() - lenFulcrumToInstrument) / lenFulcrumToInstrument;
      math::Position vecManipulatorToFulcrum = vecFulcrumToInstrument * ratio;
      math::Position manipulatorPosition = fulcrumPosition - vecManipulatorToFulcrum;
      manipulatorToolPoseTarget[0] = manipulatorPosition[0];
      manipulatorToolPoseTarget[1] = manipulatorPosition[1];
      manipulatorToolPoseTarget[2] = manipulatorPosition[2];

      // calculate orientation
      math::Quaternion rotManipulatorToInertial(manipulatorToolPoseTarget[3], manipulatorToolPoseTarget[4],
                                                manipulatorToolPoseTarget[5]);
      math::Quaternion rotInertialToManipulator(rotManipulatorToInertial.transpose());

      math::Position orientationVecCurrent(0, 0, 1);
      math::Position orientationVecNew = rotInertialToManipulator.getMatrix().dot(vecManipulatorToFulcrum);
      orientationVecNew.normalize();

      math::Position orientationVecDiff = orientationVecNew - orientationVecCurrent;

      double lA = orientationVecCurrent.norm();
      double lB = orientationVecNew.norm();
      double lC = orientationVecDiff.norm();

      double rotationAngle = std::acos((lA * lA + lB * lB - lC * lC) / (2 * lA * lB));

      math::Position rotationAxis = orientationVecCurrent.cross(orientationVecNew);
      rotationAxis.normalize();

      math::Quaternion rotDelta;
      rotDelta.setW(cos(rotationAngle / 2));
      rotDelta.setX(sin(rotationAngle / 2) * rotationAxis[0]);
      rotDelta.setY(sin(rotationAngle / 2) * rotationAxis[1]);
      rotDelta.setZ(sin(rotationAngle / 2) * rotationAxis[2]);

      rotManipulatorToInertialCorrected = rotManipulatorToInertial * rotDelta;
    }
    // instrument joint angles
    // first get the euler rates and converts them to body rate
    math::Vector3D instrumentEulerDot;
    instrumentEulerDot[0] =
        math::normalizeAngle(instrumentToolPoseTarget[3] - instrumentToolPoseTarget_[3], 0) / getDtSec();
    instrumentEulerDot[1] =
        math::normalizeAngle(instrumentToolPoseTarget[4] - instrumentToolPoseTarget_[4], 0) / getDtSec();
    instrumentEulerDot[2] =
        math::normalizeAngle(instrumentToolPoseTarget[5] - instrumentToolPoseTarget_[5], 0) / getDtSec();

    math::Vector3D instrumentPQR;
    double instrumentPhi = instrumentToolPoseTarget_[5];
    double instrumentTheta = instrumentToolPoseTarget_[4];
    instrumentPQR[0] = instrumentEulerDot[2] - instrumentEulerDot[0] * sin(instrumentTheta);
    instrumentPQR[1] =
        instrumentEulerDot[1] * cos(instrumentPhi) + instrumentEulerDot[0] * cos(instrumentTheta) * sin(instrumentPhi);
    instrumentPQR[2] =
        -instrumentEulerDot[1] * sin(instrumentPhi) + instrumentEulerDot[0] * cos(instrumentTheta) * cos(instrumentPhi);

    // set the joint angles
    // first check if the rotation direction needs to be flipped or not
    for (size_t idx = 0; idx < reverseBodyRotation_.size(); idx++) {
      if (reverseBodyRotation_[idx]) {
        instrumentPQR[idx] *= -1.0;
      }
    }

    // calculate the joint angles
    // Rx
    instrumentJointPositionsTarget[YAW_JOINT] += instrumentPQR[0] * getDtSec();

    // check the yaw joint position limit
    if ((instrumentJointPositionsTarget[YAW_JOINT] > jointPositionsUpperLimit_[YAW_JOINT]) ||
        (instrumentJointPositionsTarget[YAW_JOINT] < jointPositionsLowerLimit_[YAW_JOINT])) {
      double jointTargetLimited =
          mcx::control3::limit(instrumentJointPositionsTarget[YAW_JOINT], jointPositionsLowerLimit_[YAW_JOINT],
                               jointPositionsUpperLimit_[YAW_JOINT]);
      mcx::math::Quaternion rotDelta(0.0, 0.0, instrumentJointPositionsTarget[YAW_JOINT] - jointTargetLimited);
      // adjust the cart targets
      mcx::math::Quaternion rotTipToInertial(instrumentToolPoseTarget[3], instrumentToolPoseTarget[4],
                                             instrumentToolPoseTarget[5]);
      rotTipToInertial = rotTipToInertial * rotDelta.transpose();
      rotTipToInertial.getEuler(instrumentToolPoseTarget[3], instrumentToolPoseTarget[4], instrumentToolPoseTarget[5]);
      // set the joint position target corrected for the limit
      instrumentJointPositionsTarget[YAW_JOINT] = jointTargetLimited;
    }

    // Ry (body q): pitch
    instrumentJointPositionsTarget[PITCH_JOINT] += instrumentPQR[1] * getDtSec();

    // check the pitch joint position limit
    if ((instrumentJointPositionsTarget[PITCH_JOINT] > jointPositionsUpperLimit_[PITCH_JOINT]) ||
        (instrumentJointPositionsTarget[PITCH_JOINT] < jointPositionsLowerLimit_[PITCH_JOINT])) {
      double jointTargetLimited =
          mcx::control3::limit(instrumentJointPositionsTarget[PITCH_JOINT], jointPositionsLowerLimit_[PITCH_JOINT],
                               jointPositionsUpperLimit_[PITCH_JOINT]);
      mcx::math::Quaternion rotDelta(0.0, instrumentJointPositionsTarget[PITCH_JOINT] - jointTargetLimited, 0.0);
      // adjust the cart targets
      mcx::math::Quaternion rotTipToInertial(instrumentToolPoseTarget[3], instrumentToolPoseTarget[4],
                                             instrumentToolPoseTarget[5]);
      rotTipToInertial = rotTipToInertial * rotDelta.transpose();
      rotTipToInertial.getEuler(instrumentToolPoseTarget[3], instrumentToolPoseTarget[4], instrumentToolPoseTarget[5]);
      // set the joint position target corrected for the limit
      instrumentJointPositionsTarget[PITCH_JOINT] = jointTargetLimited;
    }

    // Rz (body r): roll
    if (enableInstrumentRoll_) {
      // convert body rate p into instrument roll joint
      instrumentJointPositionsTarget[ROLL_JOINT] += instrumentPQR[2] * getDtSec();
      rotManipulatorToInertialCorrected.getEuler(manipulatorToolPoseTarget[3], manipulatorToolPoseTarget[4],
                                                 manipulatorToolPoseTarget[5]);

      // check the roll joint position limit
      if ((instrumentJointPositionsTarget[ROLL_JOINT] > jointPositionsUpperLimit_[ROLL_JOINT]) ||
          (instrumentJointPositionsTarget[ROLL_JOINT] < jointPositionsLowerLimit_[ROLL_JOINT])) {
        double jointTargetLimited =
            mcx::control3::limit(instrumentJointPositionsTarget[ROLL_JOINT], jointPositionsLowerLimit_[ROLL_JOINT],
                                 jointPositionsUpperLimit_[ROLL_JOINT]);
        mcx::math::Quaternion rotDelta(instrumentJointPositionsTarget[ROLL_JOINT] - jointTargetLimited, 0.0, 0.0);
        // adjust the cart targets
        mcx::math::Quaternion rotTipToInertial(instrumentToolPoseTarget[3], instrumentToolPoseTarget[4],
                                               instrumentToolPoseTarget[5]);
        rotTipToInertial = rotTipToInertial * rotDelta.transpose();
        rotTipToInertial.getEuler(instrumentToolPoseTarget[3], instrumentToolPoseTarget[4],
                                  instrumentToolPoseTarget[5]);
        // set the joint position target corrected for the limit
        instrumentJointPositionsTarget[ROLL_JOINT] = jointTargetLimited;
      }
    } else {
      // convert body rate Rz into manipulator Rz
      math::Quaternion rotManipulatorRz{instrumentPQR[2] * getDtSec(), 0.0, 0.0};
      math::Quaternion rotManipulatorToInertialFinal = rotManipulatorToInertialCorrected * rotManipulatorRz;
      rotManipulatorToInertialFinal.getEuler(manipulatorToolPoseTarget[3], manipulatorToolPoseTarget[4],
                                             manipulatorToolPoseTarget[5]);
    }

    instrumentToolPoseTarget_ = instrumentToolPoseTarget;
  } else {
    /// coupled translation and rotation kinematics solver
    // first convert everything thing into tip's body frame
    math::Rotation rotTip2Inertial(instrumentToolPoseTarget);
    math::Rotation rotInertial2Tip(rotTip2Inertial.getMatrix().transpose());
    // first express everything in the body frame of the instrument tip
    mcx::math::Quaternion rotTipToInertial(instrumentToolPoseTarget[3], instrumentToolPoseTarget[4],
                                           instrumentToolPoseTarget[5]);
    mcx::math::Quaternion rotInertialToTip = rotTipToInertial.transpose();

    // set the point of interest. If the POI is at the yaw joint, set the length of link 3 to zero
    math::Position link3Local = link3TipPoseLocal_;
    if (!pointOfInterestAtTip_) {
      link3Local[2] = 0;
    }
    // TF denotes the body frame of the instrument tip
    math::Position posFulcrumTF;
    math::Position posTipTF;
    math::Position posJ2TF;
    posFulcrumTF =
        rotInertialToTip.rotateVector(mcx::math::Position(fulcrumPose_[0], fulcrumPose_[1], fulcrumPose_[2]));
    posTipTF = rotInertialToTip.rotateVector(
        mcx::math::Position{instrumentToolPoseTarget[0], instrumentToolPoseTarget[1], instrumentToolPoseTarget[2]});
    posJ2TF = posTipTF - link3Local;

    mcx::math::Position vecJ2ToTip;
    if (link3Local.norm() < std::numeric_limits<double>::epsilon()) {
      vecJ2ToTip = mcx::math::Position{0, 0, 1};
    } else {
      vecJ2ToTip = link3Local;
    }
    mcx::math::Position vecFulcrumToJ2 = posJ2TF - posFulcrumTF;
    mcx::math::Position vecFulcrumToJ2YZ{0.0, vecFulcrumToJ2[1], vecFulcrumToJ2[2]};

    mcx::math::Vector3D cp = vecFulcrumToJ2YZ.cross(vecJ2ToTip);
    double sign = (cp[0] >= 0) ? 1.0 : -1.0;
    double num = cp.norm();
    double den = vecFulcrumToJ2YZ.norm() * vecJ2ToTip.norm();

    if (std::fabs(den) >= std::numeric_limits<double>::epsilon()) {
      instrumentJointPositionsTarget[YAW_JOINT] = std::asin(mcx::control3::limit(sign * num / den, -1.0, 1.0));
    }

    // check the joint position limit
    if ((instrumentJointPositionsTarget[YAW_JOINT] > jointPositionsUpperLimit_[YAW_JOINT]) ||
        (instrumentJointPositionsTarget[YAW_JOINT] < jointPositionsLowerLimit_[YAW_JOINT])) {
      double jointTargetLimited =
          mcx::control3::limit(instrumentJointPositionsTarget[YAW_JOINT], jointPositionsLowerLimit_[YAW_JOINT],
                               jointPositionsUpperLimit_[YAW_JOINT]);
      mcx::math::Quaternion rotDelta(0.0, 0.0, instrumentJointPositionsTarget[YAW_JOINT] - jointTargetLimited);
      // adjust the cart targets
      rotTipToInertial = rotTipToInertial * rotDelta.transpose();
      rotInertialToTip = rotTipToInertial.transpose();
      posFulcrumTF =
          rotInertialToTip.rotateVector(mcx::math::Position(fulcrumPose_[0], fulcrumPose_[1], fulcrumPose_[2]));
      posTipTF = rotInertialToTip.rotateVector(
          mcx::math::Position{instrumentToolPoseTarget[0], instrumentToolPoseTarget[1], instrumentToolPoseTarget[2]});
      posJ2TF = posTipTF - link3Local;
      rotTipToInertial.getEuler(instrumentToolPoseTarget[3], instrumentToolPoseTarget[4], instrumentToolPoseTarget[5]);
      // set the joint position target corrected for the limit
      instrumentJointPositionsTarget[YAW_JOINT] = jointTargetLimited;
    }

    // get the J1 position and orientation
    mcx::math::Quaternion rotJ2(0.0, 0.0, instrumentJointPositionsTarget[YAW_JOINT]);
    mcx::math::Quaternion rotJ2T = rotJ2.transpose();
    mcx::math::Position posJ1TF = posJ2TF - rotJ2T.rotateVector(link2TipPoseLocal_);

    mcx::math::Position vecJ1ToJ2;
    if (link2TipPoseLocal_.norm() < std::numeric_limits<double>::epsilon()) {
      vecJ1ToJ2 = rotJ2T.rotateVector(mcx::math::Position{0, 0, 1});
    } else {
      vecJ1ToJ2 = rotJ2T.rotateVector(link2TipPoseLocal_);
    }
    mcx::math::Position vecFulcrumToJ1 = posJ1TF - posFulcrumTF;

    cp = vecFulcrumToJ1.cross(vecJ1ToJ2);
    sign = (rotJ2.rotateVector(cp)[1] >= 0) ? 1.0 : -1.0;
    num = cp.norm();
    den = vecJ1ToJ2.norm() * vecFulcrumToJ1.norm();

    if (std::fabs(den) >= std::numeric_limits<double>::epsilon()) {
      instrumentJointPositionsTarget[PITCH_JOINT] = std::asin(mcx::control3::limit(sign * num / den, -1.0, 1.0));
    }

    // check the joint position limit
    if ((instrumentJointPositionsTarget[PITCH_JOINT] > jointPositionsUpperLimit_[PITCH_JOINT]) ||
        (instrumentJointPositionsTarget[PITCH_JOINT] < jointPositionsLowerLimit_[PITCH_JOINT])) {
      double jointTargetLimited =
          mcx::control3::limit(instrumentJointPositionsTarget[PITCH_JOINT], jointPositionsLowerLimit_[PITCH_JOINT],
                               jointPositionsUpperLimit_[PITCH_JOINT]);
      mcx::math::Quaternion rotDelta(0.0, instrumentJointPositionsTarget[PITCH_JOINT] - jointTargetLimited, 0.0);
      // adjust the cart targets
      rotTipToInertial = rotTipToInertial * rotJ2T * rotDelta.transpose() * rotJ2;
      rotInertialToTip = rotTipToInertial.transpose();
      posFulcrumTF =
          rotInertialToTip.rotateVector(mcx::math::Position(fulcrumPose_[0], fulcrumPose_[1], fulcrumPose_[2]));
      posTipTF = rotInertialToTip.rotateVector(
          mcx::math::Position{instrumentToolPoseTarget[0], instrumentToolPoseTarget[1], instrumentToolPoseTarget[2]});
      posJ2TF = posTipTF - link3Local;
      posJ1TF = posJ2TF - rotJ2T.rotateVector(link2TipPoseLocal_);

      rotTipToInertial.getEuler(instrumentToolPoseTarget[3], instrumentToolPoseTarget[4], instrumentToolPoseTarget[5]);
      // set the joint position target corrected for the limit
      instrumentJointPositionsTarget[PITCH_JOINT] = jointTargetLimited;
    }

    // get the robot position in the inertial frame (IF)
    math::Quaternion rotJ1(0.0, instrumentJointPositionsTarget[PITCH_JOINT], 0.0);
    math::Quaternion rotJ1T = rotJ1.transpose();

    mcx::math::Position posManipulatorTF = posJ1TF - rotJ2T.rotateVector(rotJ1T.rotateVector(link1TipPoseLocal_));
    mcx::math::Position posManipulatorIF = rotTipToInertial.rotateVector(posManipulatorTF);
    manipulatorToolPoseTarget[0] = posManipulatorIF[0];
    manipulatorToolPoseTarget[1] = posManipulatorIF[1];
    manipulatorToolPoseTarget[2] = posManipulatorIF[2];

    // get the euler angles of the manipulator
    math::Quaternion rotJ1ToInertial = rotTipToInertial * rotJ2T;
    math::Quaternion rotManipulatorToInertial;
    if (enableInstrumentRoll_) {
      math::Quaternion rotRollToInertial = rotJ1ToInertial * rotJ1T;

      math::Quaternion rotManipulatorOldToInertial(manipulatorToolPoseTarget[3], manipulatorToolPoseTarget[4],
                                                   manipulatorToolPoseTarget[5]);
      math::Quaternion rotInertialToManipulatorOld = rotManipulatorOldToInertial.transpose();

      // get the relative rotation target
      math::Quaternion rotRollToManipulatorOld(rotInertialToManipulatorOld * rotRollToInertial);
      math::Vector3D eulerAngles;
      rotRollToManipulatorOld.getEuler(eulerAngles[0], eulerAngles[1], eulerAngles[2]);

      // set the roll joint angle
      instrumentJointPositionsTarget[ROLL_JOINT] = mcx::math::unwrap2(jointPositions_[ROLL_JOINT], eulerAngles[0]);

      // check the joint position limit
      if ((instrumentJointPositionsTarget[ROLL_JOINT] > jointPositionsUpperLimit_[ROLL_JOINT]) ||
          (instrumentJointPositionsTarget[ROLL_JOINT] < jointPositionsLowerLimit_[ROLL_JOINT])) {
        double jointTargetLimited =
            mcx::control3::limit(instrumentJointPositionsTarget[ROLL_JOINT], jointPositionsLowerLimit_[ROLL_JOINT],
                                 jointPositionsUpperLimit_[ROLL_JOINT]);
        math::Quaternion rotDelta(instrumentJointPositionsTarget[ROLL_JOINT] - jointTargetLimited, 0.0, 0.0);
        // adjust the cart targets
        rotTipToInertial = rotTipToInertial * rotJ2T * rotJ1T * rotDelta.transpose() * rotJ1 * rotJ2;
        rotJ1ToInertial = rotTipToInertial * rotJ2T;
        rotRollToInertial = rotJ1ToInertial * rotJ1T;

        rotTipToInertial.getEuler(instrumentToolPoseTarget[3], instrumentToolPoseTarget[4],
                                  instrumentToolPoseTarget[5]);
        // set the joint position target corrected for the limit
        instrumentJointPositionsTarget[ROLL_JOINT] = jointTargetLimited;
      }

      // calculate the manipulator orientation
      math::Quaternion rotRollToManipulator(instrumentJointPositionsTarget[ROLL_JOINT], 0.0, 0.0);
      math::Quaternion rotManipulator2Roll = rotRollToManipulator.transpose();
      rotManipulatorToInertial = rotRollToInertial * rotManipulator2Roll;
    } else {
      math::Quaternion rotRoll(jointPositions_[ROLL_JOINT], 0.0, 0.0);
      math::Quaternion rotJ1ToManipulator = rotRoll * rotJ1;
      math::Quaternion rotManipulatorToJ1 = rotJ1ToManipulator.transpose();

      rotManipulatorToInertial = rotJ1ToInertial * rotManipulatorToJ1;
    }
    rotManipulatorToInertial.getEuler(manipulatorToolPoseTarget[3], manipulatorToolPoseTarget[4],
                                      manipulatorToolPoseTarget[5]);
  }
}

void InstrumentControl::teachMaxInsertionDepth(double maxInsertionLimit) {
  const double minInsertionLimit = cartPoseConstraints_["minInsertionDepth"]->getConstraintBoundary();
  cartPoseConstraints_["maxInsertionDepth"]->setConstraintBoundary(
      (maxInsertionLimit <= minInsertionLimit) ? minInsertionLimit : maxInsertionLimit);
}

void InstrumentControl::resetFulcrumPose(const mcx::math::CartPose6& fulcrumPoseReset) {
  fulcrumPose_ = fulcrumPoseReset;
  fulcrumIsStored_ = false;
}

math::CartPose6 InstrumentControl::updateInstrumentConstraints(math::CartPose6 instrumentPoseTarget) const {
  for (auto& constraint : cartPoseConstraints_) {
    instrumentPoseTarget = constraint.second->updateConstraint(instrumentPoseTarget);
  }
  return instrumentPoseTarget;
}

bool InstrumentControl::isConstraintViolated() const {
  return std::ranges::any_of(cartPoseConstraints_,
                             [](const auto& constraint) { return constraint.second->isViolating(); });
}

bool InstrumentControl::isConstraintLimiting() const {
  return std::ranges::any_of(cartPoseConstraints_,
                             [](const auto& constraint) { return constraint.second->isLimiting(); });
}

math::Vector3D InstrumentControl::calculateConstraintForce() const {
  math::Vector3D cartConstraintForce{};

  for (auto& constraint : cartPoseConstraints_) {

    if (constraint.second->isEnabled()) {
      double deltaPositionConverted = 0;

      const auto constraintDirection = constraint.second->getConstraintDirection();
      const auto currentPositionConverted = constraint.second->getConvertedConstrainedPosition();
      const auto brakingPoint = constraint.second->getBrakingPoint();
      bool isInBrakingArea = false;
      if (((constraintDirection == CartesianPoseConstraint::STAY_INSIDE) &&
           (currentPositionConverted >= brakingPoint)) ||
          ((constraintDirection == CartesianPoseConstraint::STAY_OUTSIDE) &&
           (currentPositionConverted <= brakingPoint))) {
        isInBrakingArea = true;
        deltaPositionConverted = std::fabs(currentPositionConverted - brakingPoint);
      }

      if (isInBrakingArea) {
        const double stiffnessCoeff = constraintForceStiffness_ / constraint.second->getBrakingRange();
        const double constraintForceStiffness =
            control3::limit(deltaPositionConverted * stiffnessCoeff, 0.0, constraintForceStiffness_);
        const math::Vector3D forceVector =
            constraint.second->getLimitingVector() * deltaPositionConverted * constraintForceStiffness;
        cartConstraintForce += forceVector;
      }
    }
  }

  return cartConstraintForce;
}

double InstrumentControl::getLink1Length() const { return link1TipPoseLocal_.norm(); }

double InstrumentControl::getLink2Length() const { return link2TipPoseLocal_.norm(); }

double InstrumentControl::getLink3Length() const { return link3TipPoseLocal_.norm(); }

math::Matrix6x6 InstrumentControl::getJacobian(const math::CartPose6& manipulatorPose,
                                               const control3::JointPositions& jointPositions) const {
  // set unit vectors
  const math::Vector3D nx{1, 0, 0};
  const math::Vector3D ny{0, 1, 0};
  const math::Vector3D nz{0, 0, 1};

  // get the current joint angles
  const double pitchJointAngle = jointPositions[PITCH_JOINT];
  const double yawJointAngle = jointPositions[YAW_JOINT];

  // get the transformation matrices
  const math::Rotation rotManipulator2Inertial(manipulatorPose[3], manipulatorPose[4], manipulatorPose[5]);
  const math::Rotation rotWrist2Manipulator(0, pitchJointAngle, 0);
  const math::Rotation rotTool2Wrist(0, 0, yawJointAngle);
  const math::Rotation rotWrist2Inertial(rotManipulator2Inertial.getMatrix().dot(rotWrist2Manipulator.getMatrix()));
  const math::Rotation rotTool2Inertial(rotWrist2Inertial.getMatrix().dot(rotTool2Wrist.getMatrix()));
  const math::Rotation rotInertial2Manipulator(rotManipulator2Inertial.getMatrix().transpose());

  // assume that the link is entirely on the local z axis
  double l0 = link1TipPoseLocal_[2];
  double l2 = link2TipPoseLocal_[2];
  double l3 = link3TipPoseLocal_[2];
  if (!pointOfInterestAtTip_) {
    l3 = 0.0;
  }

  // calculate the length of the imaginary link l1
  const math::Position manipulator2FulcrumPosition =
      math::Position{fulcrumPose_[0], fulcrumPose_[1], fulcrumPose_[2]} -
      math::Position{manipulatorPose[0], manipulatorPose[1], manipulatorPose[2]};
  const double l1 = l0 - rotInertial2Manipulator.getMatrix().dot(manipulator2FulcrumPosition)[2];

  // get the rotation vectors
  const math::Vector3D pt{0, 0, l3};
  const math::Vector3D pw{0, -l3 * sin(yawJointAngle), l2 + l3 * cos(yawJointAngle)};
  const math::Vector3D pf{(l2 + l3 * cos(yawJointAngle)) * sin(pitchJointAngle), -l3 * sin(yawJointAngle),
                          l1 + (l2 + l3 * cos(yawJointAngle)) * cos(pitchJointAngle)};

  // get each column of the linear velocity jacobian matrix
  std::array<math::Vector3D, 6> jvCols;
  jvCols[0] = rotManipulator2Inertial.getMatrix().dot(nx.cross(pf));
  jvCols[1] = rotManipulator2Inertial.getMatrix().dot(ny.cross(pf));
  jvCols[2] = rotManipulator2Inertial.getMatrix().dot(nz.cross(pf));
  jvCols[3] = rotManipulator2Inertial.getMatrix().dot(nz);
  jvCols[4] = rotWrist2Inertial.getMatrix().dot(ny.cross(pw));
  jvCols[5] = rotTool2Inertial.getMatrix().dot(nx.cross(pt));

  // get each column of the rotational velocity jacobian matrix
  std::array<math::Vector3D, 6> joCols;
  joCols[0] = rotManipulator2Inertial.getMatrix().dot(nx);
  joCols[1] = rotManipulator2Inertial.getMatrix().dot(ny);
  joCols[2] = rotManipulator2Inertial.getMatrix().dot(nz);
  joCols[3] = {0, 0, 0};
  joCols[4] = rotWrist2Inertial.getMatrix().dot(ny);
  joCols[5] = rotTool2Inertial.getMatrix().dot(nx);

  // set the jacobian matrix
  math::Matrix6x6 jac;
  for (size_t col = 0; col < 6; col++) {
    for (size_t row = 0; row < 3; row++) {
      jac(row, col) = jvCols[col][row];
      jac(row + 3, col) = joCols[col][row];
    }
  }

  return jac;
}

void InstrumentControl::create_(const char* name, parameter_server::Parameter* parameterServer, uint64_t dtMicroS) {
  for (auto& constraint : cartPoseConstraints_) {
    std::string moduleName = "constraint/" + constraint.second->getName();
    createSubmodule(constraint.second.get(), moduleName.c_str());
  }
}

bool InstrumentControl::initPhase1_() {
  using namespace mcx::parameter_server;

  addParameter("teachFulcrum", ParameterType::INPUT, &teachFulcrum_);
  addParameter("instrumentStraighten/rollStraightenEnable", ParameterType::PARAMETER, &instrumentRollStraightenEnable_);

  addParameterVec("jointPositions", ParameterType::INPUT, jointPositions_);
  addParameterVec("manipulatorToolPose", ParameterType::INPUT, manipulatorToolPose_);

  addParameter("fulcrumPose", ParameterType::OUTPUT, fulcrumPose_.data(), fulcrumPose_.size());
  addParameter("fulcrumPortPosition", ParameterType::OUTPUT, fulcrumPortPosition_.data(),
               fulcrumPortPosition_.size());
  addParameter("fulcrumPortPositionOffset", ParameterType::INPUT, fulcrumPortPositionOffset_.data(),
               fulcrumPortPositionOffset_.size());
  addParameter("fulcrumIsStored", ParameterType::OUTPUT, &fulcrumIsStored_);
  addParameter("fulcrumReferenceLocal", ParameterType::PARAMETER, fulcrumReferenceLocal_.data(),
               fulcrumReferenceLocal_.size());

  addParameter("link1TipPoseLocal", ParameterType::PARAMETER, link1TipPoseLocal_.data(), link1TipPoseLocal_.size());
  addParameter("link2TipPoseLocal", ParameterType::PARAMETER, link2TipPoseLocal_.data(), link2TipPoseLocal_.size());
  addParameter("link3TipPoseLocal", ParameterType::PARAMETER, link3TipPoseLocal_.data(), link3TipPoseLocal_.size());

  addParameter("constraint/insertionDepth/teachInsertionDepthLimit", ParameterType::INPUT, &teachInsertionDepthLimit_);
  addParameter("constraint/insertionDepth/resetInsertionDepthLimit", ParameterType::INPUT, &resetInsertionDepthLimit_);
  addParameter("constraint/insertionDepth/minInstrumentInsertionDepth", ParameterType::PARAMETER,
               &minInstrumentInsertionDepth_);
  addParameter("constraint/insertionDepth/instrumentInsertionDepthCalibrationOffset", ParameterType::PARAMETER,
               &instrumentInsertionDepthCalibrationOffset_);

  addParameter("fulcrumValidityTolerance", ParameterType::PARAMETER, &fulcrumValidityTolerance_);
  addParameter("fulcrumPortExternalLength", ParameterType::PARAMETER, &fulcrumPortExternalLength_);

  addParameter("fulcrumPositionError/magnitude", ParameterType::OUTPUT, &fulcrumPositionError_);
  addParameter("fulcrumPositionError/x", ParameterType::OUTPUT, &fulcrumPositionErrorX_);
  addParameter("fulcrumPositionError/y", ParameterType::OUTPUT, &fulcrumPositionErrorY_);

  addParameter("decoupleInstrumentKinematics", ParameterType::PARAMETER, &decoupleInstrumentKinematics_);

  addParameter("forceFeedback/cartConstraint/constraintForceStiffness", ParameterType::PARAMETER,
               &constraintForceStiffness_);
  addParameterVec("forceFeedback/cartConstraint/cartConstraintWrench", ParameterType::OUTPUT, cartConstraintForce_);

  addParameter("constraint/insertionDepth/currentInstrumentInsertionDepth", ParameterType::OUTPUT,
               &instrumentInsertionDepth_);
  addParameter("constraint/insertionDepth/insertionDepthLimitReached", ParameterType::OUTPUT,
               &minInsertionDepthLimitReached_);

  addParameter("outsideFulcrumPort", ParameterType::OUTPUT, &outsideFulcrumPort_);

  addParameter("fulcrumReferenceActual", ParameterType::OUTPUT, fulcrumReferenceActual_.data(),
               fulcrumReferenceActual_.size());

  addParameter("enableInstrumentRoll", ParameterType::PARAMETER, &enableInstrumentRoll_);

  addParameter("pointOfInterestAtTip", ParameterType::PARAMETER, &pointOfInterestAtTip_);

  addParameter("fulcrumTeachSetting", ParameterType::PARAMETER, &fulcrumTeachSetting_);

  return true;
}

bool InstrumentControl::initPhase2_() { return true; }

bool InstrumentControl::startOp_() { return true; }

bool InstrumentControl::stopOp_() { return true; }

bool InstrumentControl::iterateOp_(const container::TaskTime& systemTime, container::UserTime* userTime) {
  // calculate the distance from manipulator to fulcrum
  const math::Quaternion rotManipulator2Inertial(manipulatorToolPose_[3], manipulatorToolPose_[4],
                                                 manipulatorToolPose_[5]);
  const math::Quaternion rotInertial2Manipulator(rotManipulator2Inertial.transpose());

  const math::Position fulcrumPosition =
      rotInertial2Manipulator.getMatrix().dot(math::Position(fulcrumPose_[0], fulcrumPose_[1], fulcrumPose_[2]));
  const math::Position manipulatorPosition = rotInertial2Manipulator.getMatrix().dot(
      math::Position(manipulatorToolPose_[0], manipulatorToolPose_[1], manipulatorToolPose_[2]));

  math::Position manipulator2Fulcrum = fulcrumPosition - manipulatorPosition;

  // check fulcrum validity
  fulcrumPositionErrorX_ = manipulator2Fulcrum[0];
  fulcrumPositionErrorY_ = manipulator2Fulcrum[1];
  fulcrumPositionError_ = sqrt(pow(fulcrumPositionErrorX_, 2) + pow(fulcrumPositionErrorY_, 2));
  if (fulcrumPositionError_ > fulcrumValidityTolerance_) {
    fulcrumValid_ = false;
  } else {
    fulcrumValid_ = true;
  }

  // set the minimum insertion depth
  const double minInsertionLimit = minInstrumentInsertionDepth_ + instrumentInsertionDepthCalibrationOffset_;
  cartPoseConstraints_["minInsertionDepth"]->setConstraintBoundary(minInsertionLimit);

  // calculate the current insertion depth
  double instrumentInsertionDepthOld = instrumentInsertionDepth_;
  if (decoupleInstrumentKinematics_) {
    instrumentInsertionDepth_ = std::min(getLink1Length() - manipulator2Fulcrum[2],
                                         cartPoseConstraints_["minInsertionDepth"]->getConvertedConstrainedPosition());
  } else {
    double instrumentTotalLength;
    if (pointOfInterestAtTip_) {
      instrumentTotalLength = getLink1Length() + getLink2Length() + getLink3Length();
    } else {
      instrumentTotalLength = getLink1Length() + getLink2Length();
    }
    instrumentInsertionDepth_ = std::min(instrumentTotalLength - manipulator2Fulcrum[2],
                                         cartPoseConstraints_["minInsertionDepth"]->getConvertedConstrainedPosition());
  }

  isInstrumentInserting_ = false;
  isInstrumentRetracting_ = false;
  if (instrumentInsertionDepth_ > instrumentInsertionDepthOld + INSTRUMENT_INSERTION_SPEED_THRESHOLD) {
    isInstrumentInserting_ = true;
  } else if (instrumentInsertionDepth_ < instrumentInsertionDepthOld - INSTRUMENT_INSERTION_SPEED_THRESHOLD) {
    isInstrumentRetracting_ = true;
  }

  // set the maximum insertion depth
  if (teachInsertionDepthLimit_ && fulcrumValid_) {
    const double maxInsertionLimit = instrumentInsertionDepth_ + instrumentInsertionDepthCalibrationOffset_;
    teachMaxInsertionDepth(maxInsertionLimit); // teach current insertion depth
  }

  if (resetInsertionDepthLimit_) {
    teachMaxInsertionDepth(DEFAULT_MAX_INSERTION_DEPTH); // reset current insertion depth to DEFAULT value (0.5 m)
  }

  // check if the tip is outside the fulcrum port
  if (fulcrumValid_) {
    if (instrumentInsertionDepth_ <= -fulcrumPortExternalLength_) {
      outsideFulcrumPort_ = true;
    } else {
      outsideFulcrumPort_ = false;
    }
  }

  // check the minimum insertion depth
  if ((cartPoseConstraints_["minInsertionDepth"]->isViolating()) ||
      (instrumentInsertionDepth_ <= cartPoseConstraints_["minInsertionDepth"]->getConstraintBoundary())) {
    minInsertionDepthLimitReached_ = true;
  } else {
    minInsertionDepthLimitReached_ = false;
  }

  // check the maximum insertion depth
  if (cartPoseConstraints_["maxInsertionDepth"]->isViolating() && !outsideFulcrumPort_) {
    maxInsertionDepthLimitReached_ = true;
  } else {
    maxInsertionDepthLimitReached_ = false;
  }

  // set the insertion depth reached flag
  insertionDepthLimitReached_ = minInsertionDepthLimitReached_ || maxInsertionDepthLimitReached_;

  for (auto& constraint : cartPoseConstraints_) {
    constraint.second->setReferencePose(fulcrumPose_);
    constraint.second->iterate(systemTime, userTime);
  }

  //  for (auto& generator : jointSetpointGenerators_) {
  //    generator->iterate(systemTime, userTime);
  //  }

  //  checkStraightened();

  cartConstraintForce_ = calculateConstraintForce();

  return true;
}

} // end namespace control
