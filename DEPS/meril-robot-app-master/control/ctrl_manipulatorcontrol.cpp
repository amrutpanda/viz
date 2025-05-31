/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "ctrl_manipulatorcontrol.h"
#include "ctrl_math.h"
#include "mcx/control3/ctrl_helper.h"
#include "mcx/control3/ctrl_watchdog.h"
#include <mcx/core.h>
#include <mcx/mechanics.h>
#include <meril/lgc_meril_modes_common.h>

namespace control {

// constexpr unsigned int INSTRUMENT_START_INDEX = 5;
constexpr unsigned int ROBOT_SYMBOLIC_POSITIONS_INDEX = 0;
constexpr unsigned int ROBOT_SYMBOLIC_POSITIONS_SIZE = 6;
constexpr unsigned int INSTRUMENT_SYMBOLIC_POSITIONS_INDEX = 5; // including instrument roll
constexpr unsigned int INSTRUMENT_SYMBOLIC_POSITIONS_SIZE = 5;

/**
 * Constructs a ManipulatorControl object, initializing control systems and data structures
 * required to manage the manipulator's joints and behavior. This includes setting up joint
 * mode switches, position targets, compliance controllers, positioning symbolic states,
 * and related mechanisms.
 *
 * @param mechanismModule A mechanism module responsible for managing the manipulator's
 * mechanical components and dynamics.
 * @param features A structure containing information about the manipulator, including its
 * number of joints and associated features.
 * @return A ManipulatorControl instance with initialized joint control components,
 * position management systems, symbolic position mappings, and setpoint generators
 * for instrument exchanges.
 *
 * Additional Details:
 * - Resizes and initializes control-related vectors based on the number of joints.
 * - Configures symbolic positions for both the manipulator and its instruments.
 * - Creates setpoint generators for instrument exchange operations.
 * - Establishes a position limiter force for maintaining joint operation constraints.
 */
ManipulatorControl::ManipulatorControl(std::unique_ptr<mcx::mechanics::MechanismModule> mechanismModule,
                                       feature::Manipulator features)
    : features_{features}, driveMode_{features.numberOfJoints}, jointReferenceGenerator_{features.numberOfJoints},
      jointLimiterGainLookup_{6}, semiAutoModeSwitch_{features.numberOfJoints},
      semiAutoMotionGenerator_{features.numberOfJoints}, jointModeSwitch_{features.numberOfJoints},
      manualModeSwitch_{features.numberOfJoints}, pauseModeSwitch_{features.numberOfJoints},
      jointPVATargetFilter_{features.numberOfJoints}, jointPvaActual_{features.numberOfJoints},
      jointPvaLimitedReference_{features.numberOfJoints}, idJointTorqueActual_{features.numberOfJoints},
      idJointTorqueReference_{features.numberOfJoints}, jointComplianceControllers_{features.numberOfJoints},
      jointCompliancePvaInput_{features.numberOfJoints}, jointPvaTarget_{features.numberOfJoints},
      jointSymbolicPositions_{ROBOT_SYMBOLIC_POSITIONS_SIZE, symbolicPositionNames_},
      instrumentSymbolicPositions_{INSTRUMENT_SYMBOLIC_POSITIONS_SIZE, instrumentPositionNames_},
      jointKalmanObservers_{features.numberOfJoints}, jointPVTEstimate_{features.numberOfJoints} {

  jointIsOpenLoop_.resize(features.numberOfJoints);
  hostInJointVelocity_.resize(features.numberOfJoints);
  jointManualPositionsTarget_.resize(features.numberOfJoints);
  jointAutoPositionsTarget_.resize(features.numberOfJoints);
  manualModeSwitchPositions_.resize(features.numberOfJoints);
  jointTorquesActual_.resize(features.numberOfJoints);
  idJointVirtualMassActual_.resize(features.numberOfJoints);
  instrumentExchangeInitialJointPositions_.resize(features.numberOfJoints);
  planningMode_.jointPositionsTarget.resize(features.numberOfJoints);
  planningMode_.jointPositionsRawTarget.resize(features.numberOfJoints);
  mechanismModule_ = std::move(mechanismModule);
  numberOfJoints_ = features.numberOfJoints;

  numberOfManipulatorJoints_ = mechanismModule_->mechanism().numberOfJoints();
  jointPositionLimiterForce_ = std::make_unique<JointPositionLimiterForce<NR_JOINT_FORCE_FEEDBACK>>();
}

void ManipulatorControl::create_(const char* name, mcx::parameter_server::Parameter* parameterServer,
                                 uint64_t dtMicroS) {

  createSubmodule(&semiAutoModeSwitch_, "semiAutoModeSwitch");
  createSubmodule(&semiAutoMotionGenerator_, "semiAutoMotionGenerator");
  createSubmodule(&semiAutoTimeScaleSwitch_, "semiAutoTimeScaleSwitch");

  createSubmodule(mechanismModule_.get(), "mechanism");
  // createSubmodule(&hostInJointPosition_, "hostInJointPosition");
  createSubmodule(&jointReferenceGenerator_, "jointReferenceGenerator");
  createSubmodule(&jointModeSwitch_, "jointModeSwitch");
  createSubmodule(&manualModeSwitch_, "manualModeSwitch");
  createSubmodule(&pauseModeSwitch_, "pauseModeSwitch");
  createSubmodule(&jointPVATargetFilter_, "jointPVATargetFilter");

  createSubmodules(mcx::utils::make_span(jointLimiterGainLookup_), "cartTransScaling/jointLimiterGainLookup");
  createSubmodule(&cartTransGainLookup_, "cartTransGainLookup");
  createSubmodule(&cartTransReferenceGenerator_, "cartTransReferenceGenerator");
  createSubmodule(&hostInToolPosition_, "hostInTool/Position");
  createSubmodule(&hostInToolOrientation_, "hostInTool/Orientation");

  createSubmodule(&manipulabilityGainLookup_, "manipulabilityGainLookup");
  createSubmodule(&manipulabilityDetector_, "manipulabilityDetector");

  createSubmodule(&jointComplianceModeSwitch_, "jointCompliance/ModeSwitch");
  createSubmodule(&jointComplianceControllers_, "jointCompliance/Controllers");

  createSubmodule(&instrument_, "instrument");
  createSubmodule(&instrumentPinchTargetFilter_, "instrument/pinch/TargetFilter");

  createSubmodule(&cartPoseRateLimiter_, "cartPoseRateLimiter");

  createSubmodule(&fulcrumPositionErrorLookup_, "fulcrumPositionErrorLookup");

  createSubmodule(&jointSymbolicPositions_, "jointSymbolicPositions");
  createSubmodule(&instrumentSymbolicPositions_, "instrumentSymbolicPositions");

  createSubmodule(&pinchController_, "pinchControl");

  createSubmodule(jointPositionLimiterForce_.get(), "forceFeedback/jointLimiterForce");

  createSubmodules(mcx::utils::make_span(jointKalmanObservers_), "jointStateEstimation/jointKalmanObserver");
}

bool ManipulatorControl::initPhase1_() {
  using ParamType = mcx::parameter_server::ParameterType;

  // is the manipulator in simulation mode
  addParameter("isInSimulation", ParamType::INPUT, &isInSimulation_);

  // cartesian translation limiting
  addParameter("cartTransScaling/jointLimitsScalingFactor", ParamType::OUTPUT, &jointLimitsScalingFactor_);
  addParameter("cartTransScaling/fulcrumErrorScalingFactor", ParamType::OUTPUT, &fulcrumErrorScalingFactor_);
  addParameter("cartTransScaling/jointPositionLimitGain", ParamType::PARAMETER, &jointPositionLimitGain_);
  addParameter("cartTransScaling/jointPositionLimitOffset", ParamType::PARAMETER, &jointPositionLimitOffset_);

  // Input from Actuator Control Loop
  addParameterVec("jointPositionsActual", ParamType::INPUT, jointPvaActual_.pos);
  addParameterVec("jointVelocitiesActual", ParamType::INPUT, jointPvaActual_.vel);
  addParameterVec("jointAccelerationsActual", ParamType::INPUT, jointPvaActual_.acc);
  addParameterVec("jointTorquesActual", ParamType::INPUT, jointTorquesActual_);

  addParameterVec("jointStateEstimation/jointPositions", ParamType::OUTPUT, jointPVTEstimate_.pos);
  addParameterVec("jointStateEstimation/jointVelocities", ParamType::OUTPUT, jointPVTEstimate_.vel);
  addParameterVec("jointStateEstimation/jointTorques", ParamType::OUTPUT, jointPVTEstimate_.acc);
  addParameter("jointStateEstimation/enableJointStateEstimationForAdmittance", ParamType::PARAMETER,
               &enableJointStateEstimationForAdmittance_);

  addParameterVec("jointPositionsTarget", ParamType::OUTPUT, jointPvaTarget_.pos);
  addParameterVec("jointVelocitiesTarget", ParamType::OUTPUT, jointPvaTarget_.vel);
  addParameterVec("jointAccelerationsTarget", ParamType::OUTPUT, jointPvaTarget_.acc);

  addParameterVec("jointIsOpenLoop", ParamType::INPUT, jointIsOpenLoop_);

  addParameterVec("jointPositionsLimitedReference", ParamType::INPUT, jointPvaLimitedReference_.pos);
  addParameterVec("jointVelocitiesLimitedReference", ParamType::INPUT, jointPvaLimitedReference_.vel);
  addParameterVec("jointAccelerationsLimitedReference", ParamType::INPUT, jointPvaLimitedReference_.acc);

  addParameterVec("driveMode", ParamType::INPUT, driveMode_);

  addParameter("hostInInstrumentPinch", ParamType::INPUT, &hostInInstrumentPinch_);
  //  addParameter("instrument/resetFulcrum", ParamType::INPUT, &fulcrum_.reset);
  addParameter("instrument/pinch/target", ParamType::OUTPUT, &instrumentPinchTarget_);
  addParameter("instrument/pinch/jog", ParamType::INPUT, &instrumentPinchJog_);
  addParameter("instrument/pinch/enable", ParamType::INPUT, &instrumentPinchEnable_);
  addParameter("instrument/pinch/isTracking", ParamType::OUTPUT, &instrumentPinchIsTracking_);
  addParameter("instrument/pinch/startTrackingTolerance", ParamType::PARAMETER,
               &instrumentPinchStartTrackingTolerance_);
  addParameter("instrument/instrumentCalibrationDepth", ParamType::PARAMETER, &instrumentCalibrationDepth_);

  // addParameter("enableStiffAxesLimiter", ParamType::OUTPUT, &enableStiffAxesLimiter_);

  // Inverse Dynamics
  addParameterVec("idJointTorque/actual/gravity", ParamType::OUTPUT, idJointTorqueActual_.torqueGravity);
  addParameterVec("idJointTorque/actual/dynamic", ParamType::OUTPUT, idJointTorqueActual_.torqueDynamic);
  addParameterVec("idJointTorque/actual/total", ParamType::OUTPUT, idJointTorqueActual_.torque);
  addParameterVec("idJointTorque/reference/gravity", ParamType::OUTPUT, idJointTorqueReference_.torqueGravity);
  addParameterVec("idJointTorque/reference/dynamic", ParamType::OUTPUT, idJointTorqueReference_.torqueDynamic);
  addParameterVec("idJointTorque/reference/total", ParamType::OUTPUT, idJointTorqueReference_.torque);

  addParameterVec("idJointVirtualMassActual", ParamType::OUTPUT, idJointVirtualMassActual_);

  // Compliance
  addParameter("jointCompliance/enable", ParamType::OUTPUT, &enableJointComplianceMode_);

  // Semiauto Mode
  activateSemiAutoHandle_ = addParameter("activateSemiAuto", ParamType::INPUT, &activateSemiAuto_);

  // Manual Cart Mode
  // Host Tool Velocity
  hostInToolVelocityHandle_ = addParameterVec("hostInToolVelocity", ParamType::INPUT, hostInToolVelocity_);

  addParameterVec("jointAutoPositionsTarget", ParamType::OUTPUT, jointAutoPositionsTarget_);

  // Manual Join Mode
  // Host Joint Velocity
  hostInJointVelocityHandle_ = addParameterVec("hostInJointVelocity", ParamType::INPUT, hostInJointVelocity_);
  addParameter("hostInJointVelocityGain", ParamType::INPUT, &hostInJointVelocityGain_);
  addParameter("hostInSignalTimeout", ParamType::PARAMETER, &jointVelocityWatchdog_.timeoutSec);

  addParameterVec("globalPoses/manipulatorBasePoseGlobal", ParamType::OUTPUT, globalPoses_.manipulatorBasePose);
  addParameterVec("globalPoses/fulcrumPoseGlobal", ParamType::OUTPUT, globalPoses_.fulcrumPose);
  addParameterVec("globalPoses/instrumentToolPoseGlobal", ParamType::OUTPUT, globalPoses_.instrumentToolPose);
  addParameterVec("globalPoses/handControllerPose", ParamType::INPUT, globalPoses_.handControllerPose);
  addParameterVec("globalPoses/screenPose", ParamType::INPUT, globalPoses_.screenPose);
  addParameterVec("globalPoses/cameraPose", ParamType::INPUT, globalPoses_.cameraPose);
  addParameterVec("globalPoses/baseOrientationOffset", ParamType::INPUT, globalPoses_.baseOrientationOffset);
  addParameterVec("globalPoses/fulcrumPortPose", ParamType::INPUT, globalPoses_.fulcrumPortPose);

  addParameterVec("manipulatorToolPoseActual", ParamType::OUTPUT, manipulatorToolPoseActual_);
  addParameterVec("manipulatorToolPoseTarget", ParamType::OUTPUT, manipulatorToolPoseTarget_);
  addParameterVec("manipulatorToolPoseLimitedReference", ParamType::OUTPUT, manipulatorToolPoseLimitedReference_);

  addParameterVec("instrumentToolPoseTarget", ParamType::OUTPUT, instrumentToolPoseTarget_);
  addParameterVec("instrumentToolPoseConstrainedTarget", ParamType::OUTPUT, instrumentToolPoseConstrainedTarget_);
  addParameterVec("instrumentToolPoseActual", ParamType::OUTPUT, instrumentToolPoseActual_);
  addParameterVec("instrumentToolPoseLimitedReference", ParamType::OUTPUT, instrumentToolPoseLimitedReference_);

  addParameterVec("jointManualPositionsTarget", ParamType::OUTPUT, jointManualPositionsTarget_);

  addParameter("fulcrumWatchdog/timeoutSec", ParamType::PARAMETER, &fulcrum_.watchdog.timeoutSec);
  addParameter("fulcrumWatchdog/timerSec", ParamType::OUTPUT, &fulcrum_.watchdog.timerSec);
  addParameter("fulcrumWatchdog/active", ParamType::OUTPUT, &fulcrum_.watchdog.active);

  // Camera handling
  addParameterVec("camera/cameraPose", ParamType::OUTPUT, cameraPose_);
  addParameterVec("camera/cameraPoseViewOffset", ParamType::PARAMETER, cameraPoseViewOffset_);
  addParameterVec("camera/cameraPoseView", ParamType::OUTPUT,
                  cameraPoseView_); // this is linked to the other cameraPose(s)
  addParameter("camera/manipulatorToCameraViewRotationAngle", ParamType::OUTPUT,
               &manipulatorToCameraViewRotationAngle_);
  addParameter("camera/manipulatorRollStraighteningPosition", ParamType::PARAMETER,
               &manipulatorRollStraighteningPosition_);
  addParameter("camera/manipulatorRollStraighteningToCameraView", ParamType::PARAMETER,
               &manipulatorRollStraighteningToCameraView_);
  addParameter("camera/instrumentToCameraRzOffset", ParamType::OUTPUT, &instrumentToCameraRzOffset_);
  addParameter("camera/syncInstrumentRotationToCamera", ParamType::PARAMETER, &syncInstrumentRotationToCamera_);
  // camera dual hand controller control
  addParameterVec("camera/cameraControl/velocityOutput", ParamType::OUTPUT, cameraControl_.velocityOutput);
  addParameterVec("camera/cameraControl/velocityInput", ParamType::INPUT, cameraControl_.velocityInput);
  addParameter("camera/cameraControl/velocityDeadband", ParamType::PARAMETER, &cameraControl_.velocityDeadband);
  addParameterVec("camera/cameraControl/velocityGain", ParamType::PARAMETER, cameraControl_.velocityGain);
  addParameter("camera/cameraControl/translationScale", ParamType::INPUT, &cameraControl_.translationScale);
  addParameter("camera/cameraControl/orientationScale", ParamType::INPUT, &cameraControl_.orientationScale);
  addParameter("camera/cameraControl/reverseDirection", ParamType::OUTPUT, &cameraControl_.reverseDirection);
  addParameter("camera/cameraControl/enable", ParamType::INPUT, &cameraControl_.enable);

  addParameter("poseSync/poseSyncInSurgicalEnable", ParamType::PARAMETER, &poseSyncInSurgicalEnable_);
  addParameter("poseSync/poseSyncAngleThreshold", ParamType::PARAMETER, &poseSyncAngleThreshold_);
  addParameter("poseSync/poseSyncReferenceChanged", ParamType::OUTPUT, &poseSyncReferenceChanged_);
  addParameter("poseSync/poseSyncReferencePsi", ParamType::OUTPUT, &poseSyncReferencePsi_);
  addParameter("poseSync/poseSyncReferenceTheta", ParamType::OUTPUT, &poseSyncReferenceTheta_);
  addParameter("poseSync/poseSyncReferencePhi", ParamType::OUTPUT, &poseSyncReferencePhi_);

  // Joint Position limit selector
  for (auto& el : jointPositionLimitSettings_) {
    std::string paramName = fmt::format("jointLimitSettings/{}", el.first);
    el.second.jointPositionUpperLimits.resize(numberOfJoints_);
    addParameterVec(fmt::format("{}/jointPositionUpperLimits", paramName).c_str(), ParamType::PARAMETER,
                    el.second.jointPositionUpperLimits);
    el.second.jointPositionLowerLimits.resize(numberOfJoints_);
    addParameterVec(fmt::format("{}/jointPositionLowerLimits", paramName).c_str(), ParamType::PARAMETER,
                    el.second.jointPositionLowerLimits);
  }

  // External Interface
  addParameter(":fromMerilMode", ParamType::INPUT, &merilMode_.in);
  addParameter(":toMerilMode", ParamType::OUTPUT, &merilMode_.out);

  addParameter(":fromState", ParamType::INPUT, &state_.in);
  addParameter(":fromMode", ParamType::INPUT, &mode_.in);
  auto interpInGroupHandle = addParameter(":fromInterp", ParamType::INPUT, &interpreter_.in);
  interpreter_.inHandles.move_command_ptr = interpInGroupHandle.getHandle("move_command_ptr");
  interpreter_.inHandles.move_command_ptr.setUserGroup(mcx::parameter_server::UserGroup::SYSTEM);
  interpreter_.inHandles.motion_gen_command = interpInGroupHandle.getHandle("motion_gen_command");

  addParameter(":toMode", ParamType::OUTPUT, &mode_.out);
  addParameter(":toInterp", ParamType::OUTPUT, &interpreter_.out);

  // total force feedback
  addParameterVec("forceFeedback/wrenchControllerFrame", ParamType::OUTPUT, forceFeedbackWrenchControllerFrame_);

  // force feedback scaling
  addParameter("forceFeedback/handController/engageButton", ParamType::INPUT, &handcontrollerEngageButton_);
  addParameter("forceFeedback/handController/rateLimit", ParamType::PARAMETER, &handControllerEngageRateLimit_);
  addParameter("forceFeedback/handController/omega", ParamType::PARAMETER, &handControllerEngageRateLimitOmega_);

  addParameter("jointAutoStraightenInsertionDepth", ParamType::PARAMETER, &jointAutoStraightenInsertionDepth_);

  // pinch controller
  addParameter("pinchControl/targetForce", ParamType::PARAMETER, &pinchControlTargetForce_);

  // planning mode
  addParameter("planningMode/enable", ParamType::INPUT, &planningMode_.enable);
  addParameter("planningMode/setFulcrum", ParamType::INPUT, &planningMode_.setFulcrum);
  addParameterVec("planningMode/instrumentToolPoseTarget", ParamType::INPUT, planningMode_.instrumentToolPoseTarget);
  addParameterVec("planningMode/fulcrumPose", ParamType::INPUT, planningMode_.fulcrumPose);
  addParameterVec("planningMode/instrumentToolPoseActual", ParamType::OUTPUT, planningMode_.instrumentToolPoseActual);
  addParameterVec("planningMode/manipulatorToolPose", ParamType::OUTPUT, planningMode_.manipulatorToolPose);
  addParameterVec("planningMode/jointPositionsTarget", ParamType::OUTPUT, planningMode_.jointPositionsTarget);
  addParameterVec("planningMode/jointPositionsRawTarget", ParamType::OUTPUT, planningMode_.jointPositionsRawTarget);

  return true;
}

bool ManipulatorControl::startOp_() {
  jointVelocityWatchdog_.init(getDtSec(), JointVelocities(features_.numberOfJoints));
  toolVelocityWatchdog_.init(getDtSec(), {});
  activateSemiAutoWatchdog_.init(getDtSec(), false);

  // Reset (non-logic controlled) switches to initial state
  jointComplianceModeSwitch_.resetState(enableJointComplianceMode_);
  return true;
}

/**
 * Transforms a vector from the hand controller's coordinate frame to the inertial frame.
 * This function assists in converting position, orientation, or other vector data associated
 * with the controller into a globally recognized inertial coordinate system, typically used
 * for consistent spatial representation and calculations.
 *
 * @param handControllerVector A vector represented in the coordinate frame of the hand controller.
 * The vector could denote position, velocity, or any other spatial quantity relative to the hand controller.
 * @param transformationMatrix A transformation matrix that defines the rotational and translational
 * relationship between the hand controller's frame and the inertial frame. Assumes uniform scaling.
 * @return The provided vector transformed into the inertial frame, maintaining the relationships
 * defined by the transformation matrix.
 *
 * Additional Details:
 * - Ensures accuracy by correctly applying the transformation matrix on the hand controller's vector.
 * - This operation is critical for aligning localized data with a globally recognized inertial reference.
 */
mcx::math::Quaternion ManipulatorControl::handcontrollerToInertialFrame(bool syncCart){
  // Define the rotations from the hand controller to inertial reference frame
  poseSyncReferenceChanged_ = false;
  if (syncCart || !merilMode_.out.isCameraRobot) {
    mcx::math::Quaternion rotCamera2Inertial(cameraPose_[3], cameraPose_[4], cameraPose_[5]);
    mcx::math::Quaternion rotController2Camera(
        mcx::math::Quaternion(globalPoses_.screenPose[3], globalPoses_.screenPose[4], globalPoses_.screenPose[5]).transpose() *
        mcx::math::Quaternion(globalPoses_.handControllerPose[3], globalPoses_.handControllerPose[4], globalPoses_.handControllerPose[5]));
    mcx::math::Quaternion rotController2Inertial(rotCamera2Inertial * rotController2Camera);
    // Get the delta rotation
    mcx::math::Quaternion rotDeltaQ = rotController2Inertial.transpose() * rotController2Inertial_;
    rotDeltaQ.normalize();
    if (const double rotDeltaAngle = std::fabs(2 * acos(rotDeltaQ.w())); rotDeltaAngle > poseSyncAngleThreshold_) {
      poseSyncReferenceChanged_ = true;
      rotController2Inertial_ = rotController2Inertial;
      rotController2Inertial_.getEuler(poseSyncReferencePsi_, poseSyncReferenceTheta_, poseSyncReferencePhi_);
    }
  }
  return rotController2Inertial_.transpose();
}

/**
 * Updates the manipulators' joint positions in manual cartesian mode.
 * This function integrates inputs from a variety of sources including GUI, hand controllers,
 * and cartesian references to calculate the manipulator's joint positions. It considers synchronization
 * between the manipulator's tool, camera, and inertial reference frames. Additionally, it applies
 * constraints and rate limiting to maintain operational smoothness during motion.
 *
 * @param systemTime The reference system clock from the control system.
 * @param userTime A pointer to a user-modifiable time reference, aligned with the `systemTime`.
 * @param manipulabilityGain A scalar that adjusts cartesian space gain to handle the manipulator’s motion.
 * @param syncJointPositions The joint positions at the last synchronized state.
 * @param syncToolCoordinates The tool's pose in cartesian space at the last synchronized state.
 * @param syncManipulatorCoordinates The manipulator-specific pose coordinates in reference to its base or context.
 * @return The updated joint positions considering all transformations and constraints.
 *
 * Notes on Calculation:
 * - Synchronizes cartesian inputs and hand controllers to calculate tool pose updates.
 * - Takes into account angular offsets and resets hand controller references as necessary.
 * - Applies rate limiters to cartesian and tool space poses for smooth transitions.
 * - Implements reverse kinematics to determine the resulting joint positions from cartesian targets.
 *
 * Key Features:
 * - Incorporates additional joystick and GUI-derived translations and rotations.
 * - Updates references based on synchronization of manipulator, tool, and camera relationships.
 * - Ensures proper constraints and rate limits for motion safety.
 */
const JointPositions& ManipulatorControl::manualCartMode(const mcx::container::TaskTime& systemTime,
                                                         mcx::container::UserTime* userTime, double manipulabilityGain,
                                                         const JointPositions& syncJointPositions,
                                                         const CartPose6& syncToolCoordinates,
                                                         const CartPose6& syncManipulatorCoordinates) {
  const auto syncCart = syncCartJogModeIntegrator();
  if (syncCart) {
    rotManipulatorSyncReference_.set(syncToolCoordinates[3], syncToolCoordinates[4], syncToolCoordinates[5]);
    double instrumentToCameraRzOffset = getInstrumentToCameraRotationRzOffset();
    instrumentToCameraRzOffset_ = std::round(instrumentToCameraRzOffset / M_PI_2) * M_PI_2;
  }

  // update handcontroller position to the inertial frame
  // todo: How often does this need to happen? only on parameter update and not realtime?
  mcx::math::Quaternion rotInertial2Controller = handcontrollerToInertialFrame(syncCart);

  // update input from the GUI
  toolVelocityWatchdog_.timeoutSec = jointVelocityWatchdog_.timeoutSec;
  toolVelocityWatchdog_.check(hostInToolVelocityHandle_, hostInToolVelocity_);

  // Camera dual handcontroller
  mcx::math::Vector3D hostInToolDeltaPos;
  mcx::math::Vector3D hostInToolDeltaRot;

  // copy input fom camera flipped
  cameraControl_.reverseDirection = merilMode_.in.cameraDirectionIsReversed;
  if (merilMode_.out.isCameraRobot) {
    // camera movement [x,y,z,Rz]
    if (cameraControl_.enable) {
      cameraControl_.velocityOutput = cameraMoveVelocity(
          cameraControl_.velocityInput, cameraControl_.velocityDeadband, cameraControl_.velocityGain,
          cameraControl_.translationScale, cameraControl_.orientationScale, cameraControl_.reverseDirection);
    } else {
      cameraControl_.velocityOutput = mcx::math::Twist{}; // clear camera control output
    }
    hostInToolDeltaPos = mcx::math::Vector3D{cameraControl_.velocityOutput[0] + hostInToolVelocity_[0],
                                             cameraControl_.velocityOutput[1] + hostInToolVelocity_[1],
                                             cameraControl_.velocityOutput[2] + hostInToolVelocity_[2]} *
                         getDtSec();
    // update orientation input from GUI
    // pqr from the gui are mapped to rz, ry, rx of the robot
    hostInToolDeltaRot = mcx::math::Vector3D{cameraControl_.velocityOutput[3] + hostInToolVelocity_[3],
                                             hostInToolVelocity_[4], hostInToolVelocity_[5]} *
                         getDtSec();
  } else {
    // update position input from GUI
    hostInToolDeltaPos = mcx::math::Vector3D{hostInToolVelocity_[0] + joystickInAdditiveToolVelocity_[0],
                                             hostInToolVelocity_[1] + joystickInAdditiveToolVelocity_[1],
                                             hostInToolVelocity_[2] + joystickInAdditiveToolVelocity_[2]} *
                         getDtSec();

    // update orientation input from GUI
    // pqr from the gui are mapped to rz, ry, rx of the robot
    hostInToolDeltaRot = mcx::math::Vector3D{hostInToolVelocity_[3] + joystickInAdditiveToolVelocity_[3],
                                             hostInToolVelocity_[4] + joystickInAdditiveToolVelocity_[4],
                                             hostInToolVelocity_[5] + joystickInAdditiveToolVelocity_[5]} *
                         getDtSec();
  }
  mcx::math::Quaternion rotJog(hostInToolDeltaRot[0], hostInToolDeltaRot[1], hostInToolDeltaRot[2]);

  // update input from the hand controller
  mcx::math::Position hostInToolPositionReference;
  mcx::math::Quaternion hostInToolRotationReference;
  if (syncCart) {
    // Reset the hand controller position with transformed sync position
    hostInToolPositionReference = mcx::math::Position(rotInertial2Controller.rotateVector(
        mcx::math::Position{syncToolCoordinates[0], syncToolCoordinates[1], syncToolCoordinates[2]}));

    // Reset the hand controller orientation with transformed sync orientation
    hostInToolRotationReference =
        rotManipulatorSyncReference_.transpose() *
        mcx::math::Quaternion(syncToolCoordinates[3], syncToolCoordinates[4], syncToolCoordinates[5]);
  } else {
    // TO DO perhaps remove this part, so always reset to syncTool pose to take into account the joint PVA limiter
    // Reset the hand controller position with transformed sync position
    hostInToolPositionReference = mcx::math::Position(rotInertial2Controller.rotateVector(
        mcx::math::Position{instrumentToolPoseConstrainedTarget_[0], instrumentToolPoseConstrainedTarget_[1],
                            instrumentToolPoseConstrainedTarget_[2]}));

    // Reset the hand controller orientation with transformed sync orientation
    hostInToolRotationReference =
        rotManipulatorSyncReference_.transpose() * mcx::math::Quaternion(instrumentToolPoseConstrainedTarget_[3],
                                                                         instrumentToolPoseConstrainedTarget_[4],
                                                                         instrumentToolPoseConstrainedTarget_[5]);
  }

  mcx::math::Vector3D hostInToolEulerReference{};
  hostInToolRotationReference.getEuler(hostInToolEulerReference[0], hostInToolEulerReference[1],
                                       hostInToolEulerReference[2]);
  hostInToolPosition_.setReference(hostInToolPositionReference);
  hostInToolOrientation_.setReference(hostInToolEulerReference);

  // update the hand controller position
  hostInToolPosition_.setEnable(!syncCart);
  hostInToolPosition_.setReset(true);
  hostInToolPosition_.iterate(systemTime, userTime);

  // update the hand controller rotation
  if (syncInstrumentRotationToCamera_) {
    hostInToolOrientation_.setRzOrientationOffsetExternal(instrumentToCameraRzOffset_);
  } else {
    hostInToolOrientation_.setRzOrientationOffsetExternal(0.0);
  }
  hostInToolOrientation_.setEnable(!syncCart);
  hostInToolOrientation_.setReset(true);
  hostInToolOrientation_.iterate(systemTime, userTime);

  cartTransReferenceGenerator_.iterate(systemTime, userTime);

  // Transform the target from the hand controller's local frame into the inertial reference frame
  mcx::math::Position hostInToolPosition =
      rotController2Inertial_.getMatrix().dot(mcx::math::Position{hostInToolPosition_.getOutput()});
  mcx::math::Quaternion hostInToolOrientation =
      rotManipulatorSyncReference_ * hostInToolOrientation_.getOutputQuaternion();

  const auto cartTransManual = hostInToolPosition + mcx::math::Position{cartTransReferenceGenerator_.getOutput()} +
                               rotController2Inertial_.rotateVector(hostInToolDeltaPos);

  const auto cartRotManual = hostInToolOrientation * rotJog;

  // construct the new cartesian space target in the inertial reference frame
  mcx::math::Pose instrumentTipCartPose{cartTransManual, cartRotManual.getMatrix()};

  // limit the rate of target in cartesian space
  cartTransGainLookup_.setInput(
      instrument_.getInsertionDepth()); // insertion depth function of cart gain for Ratelimiter
  cartTransGainLookup_.iterate(systemTime, userTime);

  // auto manipulatorJointPositionActual = jointPvaActual_.pos[std::slice(0, numberOfManipulatorJoints_, 1)];
  double jointLimitScaling = 1.0;
  for (unsigned int cnt = 0; cnt < numberOfManipulatorJoints_; cnt++) {
    jointLimiterGainLookup_[cnt].setInput(jointPvaActual_.pos[cnt]);
    jointLimiterGainLookup_[cnt].iterate(systemTime, userTime);
    jointLimitScaling = std::min(jointLimiterGainLookup_[cnt].getOutput(), jointLimitScaling);
  }
  jointLimitsScalingFactor_ = jointLimitScaling;

  // limit rate based on fulcrum position error
  fulcrumPositionErrorLookup_.setInput(instrument_.getFulcrumPositionError());
  fulcrumPositionErrorLookup_.iterate(systemTime, userTime);
  fulcrumErrorScalingFactor_ = fulcrumPositionErrorLookup_.getOutput();

  // cartPose is rate limited as a function of the insertion dept * the manipulabilityGain
  cartPoseRateLimiter_.setTranslationalRate(cartTransGainLookup_.getOutput() *
                                            std::min(jointLimitsScalingFactor_, fulcrumErrorScalingFactor_));
  cartPoseRateLimiter_.setReference(instrumentTipCartPose);
  cartPoseRateLimiter_.setReset(syncCart); // upon reset the instrumentTipCartPose is taken.
  cartPoseRateLimiter_.setInput(instrumentTipCartPose);
  cartPoseRateLimiter_.iterate(systemTime, userTime);

  mcx::math::Pose instrumentTipCartPoseLimited = cartPoseRateLimiter_.getOutput();

  // Instrument reverse kinematics
  // after instrument reverse kinematics update, the instrument target will be corrected for the constraints
  instrumentToolPoseTarget_ = instrumentTipCartPoseLimited.getCartPose6();

  JointPositions ikOutInstrument(4);
  ikOutInstrument[0] = jointAutoPositionsTarget_[INSTRUMENT_PITCH_JOINT];
  ikOutInstrument[1] = jointAutoPositionsTarget_[INSTRUMENT_YAW_JOINT];
  ikOutInstrument[2] = jointAutoPositionsTarget_[INSTRUMENT_PINCH_JOINT];
  ikOutInstrument[3] = jointAutoPositionsTarget_[INSTRUMENT_ROLL_JOINT];

  mcx::math::CartPose6 instrumentToolPoseConstrainedTarget = instrumentToolPoseConstrainedTarget_;
  mcx::math::CartPose6 manipulatorToolPoseTarget = manipulatorToolPoseTarget_;

  instrument_.updateInverseKin(instrumentToolPoseTarget_, manipulatorToolPoseTarget, ikOutInstrument,
                               instrumentToolPoseConstrainedTarget);

  // Manipulator inverse kinematics
  JointPositions ikOutManipulator(numberOfManipulatorJoints_);
  mechanismModule_->cartToJoint(mcx::math::Pose{manipulatorToolPoseTarget},
                                syncJointPositions[std::slice(0, numberOfManipulatorJoints_, 1)], ikOutManipulator);

  JointPositions ikOut(features_.numberOfJoints);
  ikOut[std::slice(0, ikOutManipulator.size(), 1)] = ikOutManipulator;
  ikOut[INSTRUMENT_PITCH_JOINT] = ikOutInstrument[0];
  ikOut[INSTRUMENT_YAW_JOINT] = ikOutInstrument[1];
  ikOut[INSTRUMENT_ROLL_JOINT] = ikOutInstrument[3];

  // check the manipulator joint limits
  if (!isJointNearingLimits(ikOut, 1.0, 0, numberOfManipulatorJoints_)) {
    jointAutoPositionsTarget_ = ikOut;
    manipulatorToolPoseTarget_ = manipulatorToolPoseTarget;
    instrumentToolPoseConstrainedTarget_ = instrumentToolPoseConstrainedTarget;
  }

  // Instrument pinch
  if (instrumentPinchEnable_) {
    if (!instrumentPinchIsTracking_) {
      instrumentPinchIsTracking_ =
          fabs(instrumentPinchTarget_ - hostInInstrumentPinch_) < instrumentPinchStartTrackingTolerance_;
    } else {
      instrumentPinchTarget_ = hostInInstrumentPinch_;
    }
  } else {
    // disable tracking
    instrumentPinchIsTracking_ = false;
    instrumentPinchTarget_ = syncJointPositions[INSTRUMENT_PINCH_JOINT];
  }
  instrumentPinchTargetFilter_.setInput(instrumentPinchTarget_); // this should not limit to have sufficient pinch
  instrumentPinchTargetFilter_.iterate(systemTime, userTime);
  jointAutoPositionsTarget_[INSTRUMENT_PINCH_JOINT] = instrumentPinchTargetFilter_.getOutput()[0];

  merilMode_.out.cartesianIsLimiting = cartPoseRateLimiter_.isLimiting();
  merilMode_.out.instrumentConstraintIsLimiting = instrument_.isConstraintLimiting() && merilMode_.out.fulcrumIsValid;

  /// Haptic Force Feedback
  // value updates are only allowed when in surgical mode
  // fade in when the finger is placed in the handcontroller gripper
  bool handcontrollerEngage = handcontrollerEngageButton_ && mode_.out.jointModeSwitchOff;

  handcontrollerFadeIn_ =
      mcx::control3::rateLimit(static_cast<double>(handcontrollerEngage), handcontrollerFadeIn_,
                               handControllerEngageRateLimit_, handControllerEngageRateLimitOmega_, getDtSec());

  // instrument cart constraint limiter force (forcefeedback)
  // calculate cart constraint forces, force will be generated once the target position crosses the soft braking
  // boundary
  mcx::math::Vector3D instrumentCartConstraintForce = instrument_.getConstraintForce();
  mcx::math::Vector3D instrumentCartConstraintForceControllerFrame =
      rotInertial2Controller.rotateVector(instrumentCartConstraintForce) * handcontrollerFadeIn_;
  for (size_t i = 0; i < instrumentCartConstraintForce.size(); i++) {
    instrumentCartConstraintWrenchControllerFrame_[i] = instrumentCartConstraintForceControllerFrame[i];
  }

  // manipulator joint limiter force (forcefeedback) iterate()
  mcx::math::Vector6D manipulatorJointLimiterWrench = calculateJointLimitForce(systemTime, userTime);
  mcx::math::Vector3D manipulatorJointLimiterForceControllerFrame;
  mcx::math::Vector3D manipulatorJointLimiterTorqueControllerFrame;

  manipulatorJointLimiterForceControllerFrame =
      rotInertial2Controller.rotateVector(mcx::math::Vector3D{manipulatorJointLimiterWrench[WRENCH_FORCE_X],
                                                              manipulatorJointLimiterWrench[WRENCH_FORCE_Y],
                                                              manipulatorJointLimiterWrench[WRENCH_FORCE_Z]}) *
      handcontrollerFadeIn_;
  manipulatorJointLimiterTorqueControllerFrame =
      rotInertial2Controller.rotateVector(mcx::math::Vector3D{manipulatorJointLimiterWrench[WRENCH_TORQUE_RX],
                                                              manipulatorJointLimiterWrench[WRENCH_TORQUE_RY],
                                                              manipulatorJointLimiterWrench[WRENCH_TORQUE_RZ]}) *
      handcontrollerFadeIn_;

  for (size_t i = 0; i < 3; i++) {
    manipulatorJointLimiterWrenchControllerFrame_[i] = manipulatorJointLimiterForceControllerFrame[i];
    manipulatorJointLimiterWrenchControllerFrame_[i + 3] = manipulatorJointLimiterTorqueControllerFrame[i];
  }

  forceFeedbackWrenchControllerFrame_ =
      instrumentCartConstraintWrenchControllerFrame_ + manipulatorJointLimiterWrenchControllerFrame_;

  if (syncCart) {
    jointAutoPositionsTarget_ = syncJointPositions;
    manipulatorToolPoseTarget_ = syncManipulatorCoordinates;
  }

  return jointAutoPositionsTarget_;
}

/**
 * Updates the manipulators' joint positions in manual joint mode.
 * This function processes user inputs and calculated constraints to determine
 * the desired joint target positions. It manages compliance control, adherence
 * to joint limits, jogging behavior, and safety constraints. Additionally, it
 * incorporates symbolic positions and specific conditions related to manipulator
 * modes and tasks (e.g., instrument manipulation).
 *
 * @param systemTime The reference system clock from the control system.
 * @param userTime A pointer to a user-modifiable time reference, synchronized with the `systemTime`.
 * @param syncJointPositions The joint positions at the last synchronized state.
 * @return The updated joint positions after processing manual joint mode logic.
 *
 * Functionality includes:
 * - Compliance Control:
 *   - Activates admittance or impedance control modes based on system inputs.
 *   - Sets virtual mass and torque values, enabling dynamic adjustments.
 *   - Applies positional, velocity, and dynamic constraints.
 * - Joint Jogging:
 *   - Updates jogging velocity and resets references based on mode changes.
 *   - Ensures smooth and bounded joint position updates.
 * - Synchronization:
 *   - Aligns integration of manual jog targets and reference generator outputs.
 *   - Accounts for transitions between manual linear and manual rotary control.
 * - Joint Target Verification:
 *   - Checks for joint limits and handles safety constraints, preventing overreach.
 *   - Discards potentially dangerous or limiting target positions.
 * - Specific Task Adjustments:
 *   - Manages fulcrum constraints for instrument manipulation.
 *   - Incorporates symbolic positioning for predefined tasks or joint corrections.
 *
 * Key Features:
 * - Maintains safety and reliability through joint limit enforcement.
 * - Adapts to changes in control mode and input sources dynamically.
 * - Handles manipulator-specific configurations efficiently.
 */
const JointPositions& ManipulatorControl::manualJointMode(const mcx::container::TaskTime& systemTime,
                                                          mcx::container::UserTime* userTime,
                                                          const JointPositions& syncJointPositions) {

  // Manual Joint Mode
  jointVelocityWatchdog_.check(hostInJointVelocityHandle_, hostInJointVelocity_);
  hostInJointVelocity_ *= hostInJointVelocityGain_;

  /// compliance control: admittance or impedance control.
  // if compliance mode is on the 2 positions from the admittance control and jointJogIntegrators_ will deviate from
  // each other. the admittance position is used as input for the impedance controller. getting the JOG position value
  // added there.

  const auto activeManipulatorAdmittanceJoint =
      manipulatorAdmittanceJointsMap_.at(merilMode_.in.manipulatorAdmittanceJoints);

  jointComplianceControllers_.setEnableAdmittanceController(activeManipulatorAdmittanceJoint);

  enableJointComplianceMode_ = false;
  if (merilMode_.in.gotoManipulatorManual && !merilMode_.in.gotoSymbolicMove) {
    enableJointComplianceMode_ = true;
  }

  // set calculated virtual mass of the joint (by mechanics) and set the actual measured torque (from motor or force
  // feedback)
  jointComplianceControllers_.setVirtualMass(idJointVirtualMassActual_);

  // Use Extended Kalman Filter outputs
  auto jointAdmittancePVAActual = jointPvaActual_;
  if (enableJointStateEstimationForAdmittance_ && merilMode_.out.fulcrumIsValid) {
    jointComplianceControllers_.setMeasuredTorques(jointPVTEstimate_.acc);
    jointAdmittancePVAActual.vel = jointPVTEstimate_.vel;
  } else {
    jointComplianceControllers_.setMeasuredTorques(jointTorquesActual_);
  }

  // Jogging
  jointCompliancePvaInput_.vel =
      merilMode_.in.gotoManipulatorManual ? hostInJointVelocity_ : hostInJointVelocity_ * 0.0;
  jointComplianceControllers_.setInputPVA(jointCompliancePvaInput_);
  jointComplianceControllers_.setActualPVA(jointAdmittancePVAActual);

  // set position limits
  if (merilMode_.in.updateJointLimits) {
    jointComplianceControllers_.setLimitPositions(jointPositionLimitSettings_["active"].jointPositionLowerLimits,
                                                  jointPositionLimitSettings_["active"].jointPositionUpperLimits,
                                                  JOINT_LIMITING_FACTOR);
  }

  // check target
  const bool manipulatorJointTargetLimitReached =
      isJointNearingLimits(jointPvaTarget_.pos, 1.0, 0, numberOfManipulatorJoints_ - 1);

  const bool syncLinearMove = enableManipulatorManualLinearPrevious_ != merilMode_.in.enableManipulatorManualLinear ||
                              gotoManipulatorManualPrevious_ != merilMode_.in.gotoManipulatorManual;

  if (merilMode_.in.gotoManipulatorManual || merilMode_.out.instrumentIsStraightening) {
    //    This makes sure the jog integrators are reset to the constrained joint targets
    jointComplianceControllers_.setReferencePositions(jointManualPositionsTarget_);
    if (merilMode_.out.instrumentInsertionDepthIsLimiting || syncLinearMove) {
      jointComplianceControllers_.setReferenceVelocities(hostInJointVelocity_);
    }
  } else {
    jointComplianceControllers_.setReferencePositions(syncJointPositions);
    jointComplianceControllers_.setReferenceVelocities(hostInJointVelocity_); // this can also be actual velocities
  }
  jointComplianceControllers_.setResetPosition(true);
  jointComplianceControllers_.setResetVelocity(merilMode_.out.instrumentInsertionDepthIsLimiting ||
                                               manipulatorJointTargetLimitReached || syncLinearMove);
  jointComplianceControllers_.setDisableDynamics(jointComplianceModeSwitch_.isOff() ||
                                                 isInSimulation_); // jogging only.
  jointComplianceControllers_.iterate(systemTime, userTime);

  if (!(mode_.out.jointModeSwitchOn && mode_.out.pauseModeSwitchOff)) {
    enableJointComplianceMode_ = false;
  }
  jointComplianceModeSwitch_.setToggle(enableJointComplianceMode_);
  jointComplianceModeSwitch_.setInput1(0.0);
  jointComplianceModeSwitch_.setInput2(1.0);
  jointComplianceModeSwitch_.iterate(systemTime, userTime);

  // Joint Reference Generator
  jointReferenceGenerator_.iterate(systemTime, userTime);

  // Sum of jointJogModeSwitch_ and jointReferenceGenerator_
  JointPositions jointManualPositionsTargetUpdate = jointManualPositionsTarget_;
  plus(jointManualPositionsTargetUpdate, jointComplianceControllers_.getOutputPositions(),
       jointReferenceGenerator_.getOutput());

  // update fulcrum information
  merilMode_.out.fulcrumIsStored = instrument_.getFulcrumIsStored();

  // retract & manual
  if (merilMode_.in.manipulatorAdmittanceJoints != logic::meril::NO_FULCRUM) {
    instrumentAdjust(jointManualPositionsTargetUpdate, syncJointPositions);
  }

  // only accept the new joint targets when the joint limits or insertion depth are not reached
  const JointPositions jointManualPositionsDiscard = jointManualPositionsTarget_;
  bool jointManualPositionIsNotLimiting = true;
  bool jointFourOrFiveLimit = false;
  unsigned int numberOfLimitsCount = 0;
  for (unsigned int i = 0; i < features_.numberOfJoints; i++) {
    // insertion depth not considered for camera
    jointManualPositionIsNotLimiting = !((isJointLimitsReached(jointManualPositionsTargetUpdate, i) ||
                                          merilMode_.out.instrumentInsertionDepthIsLimiting) &&
                                         (merilMode_.in.gotoManipulatorManual));

    if (jointManualPositionIsNotLimiting) {
      jointManualPositionsTarget_[i] = jointManualPositionsTargetUpdate[i];
    } else {
      jointFourOrFiveLimit = (i == 3) || (i == 4);
      numberOfLimitsCount++;
    }
  }
  // discard whgen more than 2 limits or joint 4 or 5 are limiting (needed for position correction)
  if (numberOfLimitsCount >= 2 || jointFourOrFiveLimit) {
    jointManualPositionsTarget_ = jointManualPositionsDiscard; // discard all
  }

  // Symbolic positions: (init moves, straightening, set joint axeslimiter positions.)
  symbolicJointMode(systemTime, userTime, jointManualPositionsTarget_, syncJointPositions);

  return jointManualPositionsTarget_;
}

/**
 * Computes and manages joint targets for a manipulator system and its associated instrument
 * in symbolic and manual modes. This method integrates symbolic manipulation, joint limitations,
 * symbolic moves, and calibration or straightening of the instrument in response to system states
 * and user inputs.
 *
 * @param systemTime The system time used to synchronize and update symbolic positions for the manipulator or
 * instrument.
 * @param userTime A pointer to user-specific time information, utilized during symbolic position iterations.
 * @param jointManualPositionsTarget The joint position targets provided manually by the user, modified
 * based on symbolic moves or calibration processes.
 * @param syncJointPositions The current synchronized joint positions of the manipulator and instrument.
 * This is used as a reference during computation for limit enforcement and symbolic position handling.
 */
void ManipulatorControl::symbolicJointMode(const mcx::container::TaskTime& systemTime,
                                           mcx::container::UserTime* userTime,
                                           JointPositions& jointManualPositionsTarget,
                                           const JointPositions& syncJointPositions) {
  JointPositions syncManipulatorJointPositions =
      syncJointPositions[std::slice(ROBOT_SYMBOLIC_POSITIONS_INDEX, ROBOT_SYMBOLIC_POSITIONS_SIZE, 1)];
  JointPositions manipulatorJointTargets{
      jointManualPositionsTarget[std::slice(ROBOT_SYMBOLIC_POSITIONS_INDEX, ROBOT_SYMBOLIC_POSITIONS_SIZE, 1)]};
  syncManipulatorJointPositions[5] = jointSymbolicPositions_.getOutputPosition(5);

  // straightening of the instrument joints or calibrate the instrument positions
  JointPositions syncInstrumentJointPositions =
      syncJointPositions[std::slice(INSTRUMENT_SYMBOLIC_POSITIONS_INDEX, INSTRUMENT_SYMBOLIC_POSITIONS_SIZE, 1)];
  JointPositions instrumentJointTargets{jointManualPositionsTarget[std::slice(INSTRUMENT_SYMBOLIC_POSITIONS_INDEX,
                                                                              INSTRUMENT_SYMBOLIC_POSITIONS_SIZE, 1)]};

  /// JOINT SYMBOLIC POSITIONS:
  jointSymbolicPositions_.syncJointPositions(syncManipulatorJointPositions);

  //
  const auto& selectedName = jointSymbolicPositions_.getSelectedMoveName();
  const bool jointSymbolicPositionsNotInIdle = selectedName != "idle";

  // set limiting (axesLimiters)
  if (merilMode_.in.updateJointLimits) {
    if (jointPositionLimitSettings_.contains(selectedName) && jointSymbolicPositionsNotInIdle) {
      jointPositionLimitSettings_["active"] = jointPositionLimitSettings_[selectedName];
    }
    if (!jointSymbolicPositionsNotInIdle) {
      jointPositionLimitSettings_["active"] = jointPositionLimitSettings_["moveToStartHigh"]; // default is High
      if (jointPvaActual_.pos[ROBOT_JOINT_3] > (150 * M_PI / 180)) { // limit for J3, in park it goes to 158 degrees
        jointPositionLimitSettings_["active"] = jointPositionLimitSettings_["moveToPark"];
      }
      if (std::fabs(jointPvaActual_.pos[ROBOT_JOINT_5]) < -1.0 * M_PI) {
        // low position
        jointPositionLimitSettings_["active"] = jointPositionLimitSettings_["moveToStartLow"];
      }
    }
    // set joint limits used for admittance control and limiting cartesian rate limiter translational gain on update of
    // the symbolic positions.
    for (unsigned int cnt = 0; cnt < numberOfManipulatorJoints_; cnt++) {
      setLimitPositions(jointPositionLimitSettings_["active"].jointPositionLowerLimits[cnt],
                        jointPositionLimitSettings_["active"].jointPositionUpperLimits[cnt], jointPositionLimitGain_,
                        jointPositionLimitOffset_, cnt);
    }
  }

  /// joint symbolic moves
  if (merilMode_.in.gotoSymbolicMove) {
    /// symbolic manipulator move
    if (!gotoSymbolicMovePrev_) {
      storedJointManipulatorSyncPosition_ = manipulatorJointTargets[5] - syncManipulatorJointPositions[5];
      jointSymbolicPositions_.setTimeScaleFactor(0.0);
    }
    manipulatorJointTargets = jointSymbolicPositions_.moveToSymbolicPositions(
        manipulatorJointTargets, syncManipulatorJointPositions, jointSymbolicPositionsNotInIdle);

    merilMode_.out.symbolicMoveIsDone =
        (jointSymbolicPositions_.checkAtSymbolicPosition(syncManipulatorJointPositions) &&
         jointSymbolicPositionsNotInIdle);

    merilMode_.out.symbolicMoveIsStarted = jointSymbolicPositions_.getIsStarted();
    merilMode_.out.isAtParkPosition = (selectedName == "moveToPark") && merilMode_.out.symbolicMoveIsDone;
    merilMode_.out.isAtLowPosition = (selectedName == "moveToStartLow") && merilMode_.out.symbolicMoveIsDone;
    instrumentJointTargets[0] = manipulatorJointTargets[5] +
                                storedJointManipulatorSyncPosition_; // copy J6 moves to instrument symbolic positions.
    syncInstrumentJointPositions[0] = syncManipulatorJointPositions[5] + storedJointManipulatorSyncPosition_;
  } else {
    merilMode_.out.symbolicMoveIsStarted = false;
    merilMode_.out.symbolicMoveIsDone = false;
    jointSymbolicPositions_.setSelectMove(SP_IDLE);
  }
  jointSymbolicPositions_.iterate(systemTime, userTime);
  gotoSymbolicMovePrev_ = merilMode_.in.gotoSymbolicMove;

  /// INSTRUMENT SYMBOLIC POSITIONS
  // correct instrument joint target with the manipulator symbolic position for J6:
  instrumentSymbolicPositions_.syncJointPositions(syncInstrumentJointPositions);

  // past the fulcrum the instrument shall rotate towards the camera.
  // default start position is J6 rotation from jointSymbolicPOsitions for the manipulator
  double straightRollAngle = syncManipulatorJointPositions[5];
  if (manipulatorRollStraighteningToCameraView_) {
    straightRollAngle +=
        mcx::math::normalizeAngle(jointPvaActual_.pos[ROBOT_JOINT_6] + manipulatorToCameraViewRotationAngle_, 0.0);
  } else {
    straightRollAngle += manipulatorRollStraighteningPosition_;
  }
  instrumentSymbolicPositions_.teachSymbolicPosition(straightRollAngle, SP_INSTRUMENT_STRAIGHTEN_ROLL,
                                                     SP_INSTRUMENT_ROBOT_ROLL_JOINT);

  unsigned int SP_STRAIGHTEN =
      merilMode_.in.gotoInstrumentStraightenRoll ? SP_INSTRUMENT_STRAIGHTEN_ROLL : SP_INSTRUMENT_STRAIGHTEN;

  // calibration & Straightening of the instrument
  merilMode_.out.instrumentAutoStraighten = false;  // reset auto-straightening boolean.
  if (!merilMode_.in.gotoInstrumentCalibration && !merilMode_.in.gotoSterileAdapterCalibration) {
    // straighten is disabled for camera (poseSyncInSurgicalEnable_ = false --> camera)
    if (!merilMode_.out.isCameraRobot) {
      if (merilMode_.in.gotoInstrumentStraighten) {
        // roll rotation (only in surgical)
        instrumentJointTargets = instrumentSymbolicPositions_.moveToSymbolicPositions(
            instrumentJointTargets, syncInstrumentJointPositions, SP_STRAIGHTEN,
            !merilMode_.out.instrumentStraightenDone);

      } else {
        // auto straighten on insert / retract instrument only in unlocked instrument mode!
        if (jointModeSwitch_.isOn()) {
          double autoStraightenInsertionDepth =
              std::max(jointAutoStraightenInsertionDepth_, MINIMUM_AUTO_STRAIGHTEN_INSERTION_DEPTH);
          if ((instrument_.getInsertionDepth() > instrument_.getMinimumInsertionDepth()) &&
              (instrument_.getInsertionDepth() <
               instrument_.getMinimumInsertionDepth() + autoStraightenInsertionDepth)) {
            merilMode_.out.instrumentAutoStraighten = true;   // straightening is a seperate mode.
          }
        }
      }
    }
    // check if instrument is straight or when it's a camera, SP is DONE
    instrumentSymbolicPositions_.checkAtSymbolicPosition(syncInstrumentJointPositions, SP_STRAIGHTEN);
    merilMode_.out.instrumentIsStraightening = instrumentSymbolicPositions_.getIsBusy(SP_STRAIGHTEN);
    merilMode_.out.instrumentStraightenDone =
        instrumentSymbolicPositions_.getAtSymbolicPosition(SP_STRAIGHTEN) || merilMode_.out.isCameraRobot;

    instrumentCalibrationChannel_ = 0; // reset calibration sequence

    merilMode_.out.instrumentCalibrationDone = false;
    merilMode_.out.sterileAdapterCalibrationDone = false;

  } else if (merilMode_.in.gotoInstrumentCalibration) {
    // instrument calibration sequence is run here:
    const auto channel = instrumentCalibrationSequence_[instrumentCalibrationChannel_];
    instrumentJointTargets = instrumentSymbolicPositions_.moveToSymbolicPositions(
        instrumentJointTargets, syncInstrumentJointPositions, channel, !merilMode_.out.instrumentCalibrationDone);
    if (instrumentSymbolicPositions_.checkAtSymbolicPosition(syncInstrumentJointPositions, channel)) {
      if (instrumentCalibrationChannel_ == (instrumentCalibrationSequence_.size() - 1)) {
        merilMode_.out.instrumentCalibrationDone = true;
      } else {
        instrumentCalibrationChannel_++;
      }
    }
  } else if (merilMode_.in.gotoSterileAdapterCalibration) {
    // instrument calibration sequence is run here:
    const auto channel = adapterCalibrationSequence_[instrumentCalibrationChannel_];
    instrumentJointTargets = instrumentSymbolicPositions_.moveToSymbolicPositions(
        instrumentJointTargets, syncInstrumentJointPositions, channel, !merilMode_.out.sterileAdapterCalibrationDone);
    if (instrumentSymbolicPositions_.checkAtSymbolicPosition(syncInstrumentJointPositions, channel)) {
      if (instrumentCalibrationChannel_ == (adapterCalibrationSequence_.size() - 1)) {
        merilMode_.out.sterileAdapterCalibrationDone = true;
      } else {
        instrumentCalibrationChannel_++;
      }
    }
  }

  // iterate symbolic positions for instrument and connect to target values
  instrumentSymbolicPositions_.iterate(systemTime, userTime);

  jointManualPositionsTarget[std::slice(ROBOT_SYMBOLIC_POSITIONS_INDEX, ROBOT_SYMBOLIC_POSITIONS_SIZE, 1)] =
      manipulatorJointTargets;
  jointManualPositionsTarget[std::slice(INSTRUMENT_SYMBOLIC_POSITIONS_INDEX, INSTRUMENT_SYMBOLIC_POSITIONS_SIZE, 1)] =
      instrumentJointTargets;
}

/**
 * Handles Cartesian pose-based planning mode for a manipulator, overriding the instrument's
 * Cartesian target positions with the stored planning targets when planning mode is enabled
 * and within a simulation context. This function ensures synchronization of the manipulator
 * and instrument's kinematics, joint limits enforcement, and forward kinematics updates.
 *
 * Key functional steps:
 * - Updates the instrument's Cartesian and inverse kinematics transformations using target joint positions.
 * - Computes the manipulator's joint positions from the Cartesian target, enforcing joint position limits.
 * - Updates manipulator and instrument forward kinematics based on resolved joint positions.
 * - Applies constraints to maintain valid joint positions within configured operational limits.
 * - Synchronizes manipulator and instrument pose and joint position data in planning mode.
 *
 * If planning mode is not enabled or outside a simulation context, falls back to using actual
 * (non-planning) pose and joint position data for the manipulator and instrument.
 */
void ManipulatorControl::planningCartMode() {
  // in case of planning mode, override the instrument cartesian target directly by the planning target
  if (planningMode_.enable && isInSimulation_) {
    if (planningMode_.setFulcrum) {
      instrument_.resetFulcrumPose(planningMode_.fulcrumPose);
      planningMode_.setFulcrum = false;
    }

    mcx::math::CartPose6 instrumentToolPoseConstrainedTarget = planningMode_.instrumentToolPoseActual;
    mcx::math::CartPose6 manipulatorToolPoseTarget = planningMode_.manipulatorToolPose;

    // update instrument inverse kinematics
    JointPositions ikOutInstrument(4);
    ikOutInstrument[0] = planningMode_.jointPositionsTarget[INSTRUMENT_PITCH_JOINT];
    ikOutInstrument[1] = planningMode_.jointPositionsTarget[INSTRUMENT_YAW_JOINT];
    ikOutInstrument[2] = planningMode_.jointPositionsTarget[INSTRUMENT_PINCH_JOINT];
    ikOutInstrument[3] = planningMode_.jointPositionsTarget[INSTRUMENT_ROLL_JOINT];

    instrument_.updateInverseKin(planningMode_.instrumentToolPoseTarget, manipulatorToolPoseTarget, ikOutInstrument,
                                 instrumentToolPoseConstrainedTarget);

    // update manipulator inverse kinematics
    JointPositions ikOutManipulator(numberOfManipulatorJoints_);
    mechanismModule_->cartToJoint(mcx::math::Pose{manipulatorToolPoseTarget},
                                  planningMode_.jointPositionsTarget[std::slice(0, numberOfManipulatorJoints_, 1)],
                                  ikOutManipulator);

    JointPositions ikOut(features_.numberOfJoints);
    ikOut[std::slice(0, ikOutManipulator.size(), 1)] = ikOutManipulator;
    ikOut[INSTRUMENT_PITCH_JOINT] = ikOutInstrument[0];
    ikOut[INSTRUMENT_YAW_JOINT] = ikOutInstrument[1];
    ikOut[INSTRUMENT_ROLL_JOINT] = ikOutInstrument[3];

    // check for manipulator joint position limits
    planningMode_.isLimiting = false;
    for (size_t i = 0; i < numberOfManipulatorJoints_; i++) {
      if (ikOut[i] > jointPositionLimitSettings_["active"].jointPositionUpperLimits[i]) {
        ikOut[i] = jointPositionLimitSettings_["active"].jointPositionUpperLimits[i];
        planningMode_.isLimiting = true;
      } else if (ikOut[i] < jointPositionLimitSettings_["active"].jointPositionLowerLimits[i]) {
        ikOut[i] = jointPositionLimitSettings_["active"].jointPositionLowerLimits[i];
        planningMode_.isLimiting = true;
      }
    }

    planningMode_.jointPositionsRawTarget = ikOut;
    // set the joint positions
    if (!planningMode_.isLimiting) {
      planningMode_.jointPositionsTarget = ikOut;
      // update manipulator forward kinematics
      CartPoseOut cartPoseOut;
      mechanismModule_->jointToCart(JointPVAIn{planningMode_.jointPositionsTarget},
                                    mcx::math::Pose{planningMode_.manipulatorToolPose}, cartPoseOut);
      planningMode_.manipulatorToolPose = cartPoseOut.toolCoord.getCartPose6();

      // instrument forward kinematics
      instrument_.updateForwardKin(
          planningMode_.manipulatorToolPose,
          mcx::control3::JointPositions{planningMode_.jointPositionsTarget[INSTRUMENT_PITCH_JOINT],
                                        planningMode_.jointPositionsTarget[INSTRUMENT_YAW_JOINT],
                                        planningMode_.jointPositionsTarget[INSTRUMENT_PINCH_JOINT],
                                        planningMode_.jointPositionsTarget[INSTRUMENT_ROLL_JOINT]},
          planningMode_.instrumentToolPoseActual);
    }
  } else {
    planningMode_.instrumentToolPoseActual = instrumentToolPoseActual_;
    planningMode_.manipulatorToolPose = manipulatorToolPoseActual_;
    planningMode_.jointPositionsTarget = jointPvaActual_.pos;
  }
}

/**
 * Adjusts the instrument's position and interaction settings based on the current state,
 * input commands, and constraints such as fulcrum penetration and motion thresholds.
 * Additionally, handles the transition logic for instrument exchange operations, manual
 * manipulation modes, and linear movement constraints.
 *
 * @param jointManualPositionsTarget A reference to the joint position targets for manual adjustments.
 * These positions are updated within the function to reflect changes based on the instrument's
 * manipulation and alignment.
 * @param syncJointPositions A reference to the synchronized joint positions used for ensuring
 * cohesion between manual adjustments and the manipulator's actual state.
 *
 * Function Details:
 * - Disables automatic straightening and predefined insertion depth limitations.
 * - Detects retraction and insertion states based on velocity thresholds for specific joints.
 * - Manages the state and activation of the instrument exchange process, ensuring safety
 *   by checking the instrument's position relative to the fulcrum's constraints before allowing
 *   operation.
 * - Implements mechanisms for resetting and returning the exchange setpoints.
 * - Adjusts the manipulator's tool position target for manual or linear movements while
 *   respecting the fulcrum constraints.
 * - Applies forward kinematics to update robot coordinates and manipulate the tool's Cartesian
 *   position.
 * - Differentiates between regular unlocked instrument mode (move in any direction) and linear
 *   straight-line movement mode for respecting fulcrum constraints, using calculations in both
 *   inertial and manipulator coordinate frames.
 * - Rotates and aligns the manipulator respecting fulcrum orientation while addressing the
 *   manipulator's position and additional rotation requirements.
 */
void ManipulatorControl::instrumentAdjust(JointPositions& jointManualPositionsTarget,
                                          const JointPositions& syncJointPositions) {
  merilMode_.out.instrumentInsertionDepthIsLimiting = false;

  // check if the instrument is outside the fulcrum
  merilMode_.out.instrumentIsOutsideFulcrum = instrument_.isOutsideFulcrumPort();

  // always reset the reference position for retract if it is not enabled
  if (!merilMode_.in.enableManipulatorManualLinear) {
    manipulatorManualLinearMovePoseReference_ = manipulatorToolPoseActual_;
  }

  // return to instrument retract position as set during exchange:
  if (merilMode_.in.storeInstrumentRetractPosition) {
    manipulatorRetractPose_ = manipulatorToolPoseActual_;
    storedInstrumentLink3LengthForRetract_ = instrument_.getLink3Length();
  }
  // adapt the retract pose if the newly installed instrument has a longer link3 length
  double manipulatorRetractPoseDeltaZ = instrument_.getLink3Length() - storedInstrumentLink3LengthForRetract_;
  if (manipulatorRetractPoseDeltaZ > std::numeric_limits<double>::epsilon()) {
    storedInstrumentLink3LengthForRetract_ = instrument_.getLink3Length();
    mcx::math::Quaternion rotRetractManipulatorToInertial(manipulatorRetractPose_[3], manipulatorRetractPose_[4],
                                                          manipulatorRetractPose_[5]);
    mcx::math::Position posRetractInertialFrame(manipulatorRetractPose_[0], manipulatorRetractPose_[1],
                                                manipulatorRetractPose_[2]);
    mcx::math::Position posRetractManipulatorFrame =
        rotRetractManipulatorToInertial.transpose().rotateVector(posRetractInertialFrame);
    posRetractManipulatorFrame[2] -= manipulatorRetractPoseDeltaZ;
    posRetractInertialFrame = rotRetractManipulatorToInertial.rotateVector(posRetractManipulatorFrame);
    for (size_t i = 0; i < posRetractInertialFrame.size(); i++) {
      manipulatorRetractPose_[i] = posRetractInertialFrame[i];
    }
  }

  /// Manual handguiding moves + linear moves
  if (merilMode_.in.gotoManipulatorManual) {

    // syncs the drives that are in torque control
    if (merilMode_.in.enableImpedanceInUnlockedInstrumentMode) {
      for (size_t cnt = 0; cnt < numberOfManipulatorJoints_; cnt++) {
        if (driveMode_[cnt] == mcx::drive::DRIVE_MODE_CYCLIC_SYNC_TORQUE) {
          jointManualPositionsTarget[cnt] = syncJointPositions[cnt]; // iff in torque mode
        }
      }
    }

    // update forward kinematics for estimated robot coordinates
    CartPoseOut cartPoseOut;
    mechanismModule_->jointToCart(JointPVAIn{jointManualPositionsTarget}, mcx::math::Pose{manipulatorToolPoseActual_},
                                  cartPoseOut);

    CartPose6 manipulatorCartPoseTarget = cartPoseOut.toolCoord.getCartPose6();
    mcx::math::Position manipulatorPosition(manipulatorCartPoseTarget[0], manipulatorCartPoseTarget[1],
                                            manipulatorCartPoseTarget[2]);

    // check distance to fulcrum
    CartPose6 fulcrumPose = instrument_.getFulcrumPose();
    mcx::math::Position fulcrumPosition(fulcrumPose[0], fulcrumPose[1], fulcrumPose[2]);
    mcx::math::Position manipulatorToFulcrum = fulcrumPosition - manipulatorPosition;

    // correct cart pose
    if (merilMode_.in.enableStopAtRetractPosition || merilMode_.in.enableManipulatorManualLinear) {
      // unlocked instrument mode using linear straight line moves only for moving the instrument respecting the
      // fulcrum:
      // default the manipulator retract position is taken.
      if (merilMode_.in.enableStopAtRetractPosition) { // this line has priority
        manipulatorManualLinearMovePoseReference_ = manipulatorRetractPose_;
      } else { // enableManipulatorManualLinear = true
        if (!enableManipulatorManualLinearPrevious_) {
          manipulatorManualLinearMovePoseReference_ = manipulatorToolPoseActual_;
        }
      }
      mcx::math::Position manipulatorLinearMoveReferencePosition = mcx::math::Position(
          manipulatorManualLinearMovePoseReference_[0], manipulatorManualLinearMovePoseReference_[1],
          manipulatorManualLinearMovePoseReference_[2]);

      // instrument retract calculation
      // limit the cart pose target on the Z axis of the robot body frame
      // mcx::math::Rotation rotManipulator2Inertial(manipulatorManualLinearMovePoseReference_);
      mcx::math::Rotation rotManipulator2Inertial(manipulatorManualLinearMovePoseReference_);
      mcx::math::Rotation rotInertial2Manipulator(rotManipulator2Inertial.getMatrix().transpose());

      mcx::math::Position positionReferenceManipulatorFrame =
          rotInertial2Manipulator.getMatrix().dot(manipulatorLinearMoveReferencePosition);
      mcx::math::Position positionTargetManipulatorFrame = rotInertial2Manipulator.getMatrix().dot(manipulatorPosition);

      mcx::math::Position positionLinearManipulatorFrame = positionReferenceManipulatorFrame;
      // stop at retract position during exchange (enableStopAtRetractPosition). freeze insertion depth if so.
      if (merilMode_.in.enableStopAtRetractPosition &&
          (positionTargetManipulatorFrame[2] > positionReferenceManipulatorFrame[2])) {
        positionLinearManipulatorFrame[2] = positionReferenceManipulatorFrame[2];
        merilMode_.out.instrumentIsAtRetractPosition = true;
      } else {
        positionLinearManipulatorFrame[2] = positionTargetManipulatorFrame[2];
        merilMode_.out.instrumentIsAtRetractPosition = false;
      }

      mcx::math::Position positionLinearInertialFrame =
          rotManipulator2Inertial.getMatrix().dot(positionLinearManipulatorFrame);

      manipulatorCartPoseTarget = manipulatorManualLinearMovePoseReference_;
      for (size_t i = 0; i < 3; i++) {
        manipulatorCartPoseTarget[i] = positionLinearInertialFrame[i];
      }
    } else {
      // regular unlocked instrument mode for moving the instrument respecting the fulcrum:
      // in this part the joint target positions are updated such that the fulcrum is respected.
      mcx::math::Quaternion rotManipulator2Inertial(manipulatorCartPoseTarget[3], manipulatorCartPoseTarget[4],
                                                    manipulatorCartPoseTarget[5]);
      mcx::math::Quaternion rotInertial2Manipulator(rotManipulator2Inertial.transpose());

      // correct the rotation
      mcx::math::Position currentOrientationVec(0, 0, 1);
      mcx::math::Position correctedOrientationVec = rotInertial2Manipulator.getMatrix().dot(manipulatorToFulcrum);
      correctedOrientationVec.normalize();

      mcx::math::Position diffOrientationVec = correctedOrientationVec - currentOrientationVec;

      double lA = currentOrientationVec.norm();
      double lB = correctedOrientationVec.norm();
      double lC = diffOrientationVec.norm();

      double rotationAngle = std::acos((lA * lA + lB * lB - lC * lC) / (2 * lA * lB));

      mcx::math::Position crossProductOrientation = currentOrientationVec.cross(correctedOrientationVec);
      crossProductOrientation.normalize();

      mcx::math::Quaternion additionalRotation = {};
      additionalRotation.setW(cos(rotationAngle / 2));
      additionalRotation.setX(sin(rotationAngle / 2) * crossProductOrientation[0]);
      additionalRotation.setY(sin(rotationAngle / 2) * crossProductOrientation[1]);
      additionalRotation.setZ(sin(rotationAngle / 2) * crossProductOrientation[2]);

      mcx::math::Quaternion correctedRotation = rotManipulator2Inertial * additionalRotation;

      // update euler angles in the manipulatorCartPoseTarget:
      correctedRotation.getEuler(manipulatorCartPoseTarget[3], manipulatorCartPoseTarget[4],
                                 manipulatorCartPoseTarget[5]);
    }

    if (instrument_.isMaximumInsertionDepthEnabled()) {
      // recalculate the manipulatorCartPoseTarget for the maximum insertion depth.
      mcx::math::Quaternion rotManipulatorToInertial(manipulatorCartPoseTarget[3], manipulatorCartPoseTarget[4],
                                                     manipulatorCartPoseTarget[5]);
      mcx::math::Position manipulatorPositionTarget =
          mcx::math::Position(manipulatorCartPoseTarget[0], manipulatorCartPoseTarget[1], manipulatorCartPoseTarget[2]);
      mcx::math::Position manipulatorPositionTargetManipulatorFrame =
          rotManipulatorToInertial.transpose().rotateVector(manipulatorPositionTarget);
      mcx::math::Position fulcrumPositionManipulatorFrame =
          rotManipulatorToInertial.transpose().rotateVector(fulcrumPosition);

      double instrumentLength;
      if (instrument_.pointOfInterestAtTip()) {
        instrumentLength = instrument_.getLink1Length() + instrument_.getLink2Length() + instrument_.getLink3Length();
      } else {
        instrumentLength = instrument_.getLink1Length() + instrument_.getLink2Length();
      }
      if (manipulatorPositionTargetManipulatorFrame[2] + instrumentLength - fulcrumPositionManipulatorFrame[2] >
          instrument_.getMaximumInsertionDepth()) {
        manipulatorPositionTargetManipulatorFrame[2] =
            instrument_.getMaximumInsertionDepth() + fulcrumPositionManipulatorFrame[2] - instrumentLength;
      }
      manipulatorPositionTarget = rotManipulatorToInertial.rotateVector(manipulatorPositionTargetManipulatorFrame);

      for (size_t i = 0; i < manipulatorPositionTarget.size(); i++) {
        manipulatorCartPoseTarget[i] = manipulatorPositionTarget[i];
      }
    }

    // Prevent the instrument to retract through the fulcrum whenever the instrument is not straight (not
    // considering roll axes)
    bool straight =
        instrumentSymbolicPositions_.getJointAtSymbolicPosition(SP_INSTRUMENT_STRAIGHTEN, SP_INSTRUMENT_PITCH_JOINT) &&
        instrumentSymbolicPositions_.getJointAtSymbolicPosition(SP_INSTRUMENT_STRAIGHTEN, SP_INSTRUMENT_YAW_JOINT) &&
        instrumentSymbolicPositions_.getJointAtSymbolicPosition(SP_INSTRUMENT_STRAIGHTEN, SP_INSTRUMENT_PINCH_JOINT);

    // stop at fulcrum for calibration during exchange procedure: inserting a new instrument
    merilMode_.out.instrumentIsAtCalibrationPosition = false;
    if (merilMode_.in.enableStopAtCalibrationPosition) {
      double calibrationDepth =
          mcx::control3::limit(instrumentCalibrationDepth_, -INSTRUMENT_CALIBRATION_MAXIMUM_POSITION,
                               INSTRUMENT_CALIBRATION_MAXIMUM_POSITION);
      if (instrument_.getInsertionDepth() > calibrationDepth) {
        merilMode_.out.instrumentIsAtCalibrationPosition = true;
      }
    }

    // check if the instrument robot is limiting. a camera is never limiting.
    merilMode_.out.instrumentInsertionDepthIsLimiting = ((instrument_.minInsertionDepthLimitReached() && !straight) ||
                                                         merilMode_.out.instrumentIsAtCalibrationPosition) &&
                                                        !merilMode_.out.isCameraRobot;

    // update inverse kinematics for the constrained joint targets
    // only set the first 5 joint position targets
    double lastJointPositionTarget = jointManualPositionsTarget[numberOfManipulatorJoints_ - 1];
    mechanismModule_->cartToJoint(mcx::math::Pose(manipulatorCartPoseTarget), jointPvaActual_.pos,
                                  jointManualPositionsTarget);
    jointManualPositionsTarget[numberOfManipulatorJoints_ - 1] = lastJointPositionTarget;
  }
  // update triggers
  gotoManipulatorManualPrevious_ = merilMode_.in.gotoManipulatorManual;
  enableManipulatorManualLinearPrevious_ = merilMode_.in.enableManipulatorManualLinear;
}

/** ManipulatorControl::isJointLimitsReached(...)
 * Checks whether the target joint position for a specific joint index has reached or
 * exceeded its defined limits. This method evaluates the joint's position against
 * predefined upper and lower limits, considering whether the joint is in open-loop mode.
 *
 * @param jointPositionsTarget A vector containing the target positions for all joints.
 * @param index The index of the joint to be evaluated for position limits.
 * @return A boolean value indicating whether the joint's target position exceeds its
 * defined limits. True if the upper or lower limit is exceeded; false otherwise.
 */
bool ManipulatorControl::isJointLimitsReached(const JointPositions& jointPositionsTarget, unsigned int index) {
  bool limitReached = false;
  if (!jointIsOpenLoop_[index]) {
    limitReached =
        ((jointPositionsTarget[index] >= jointPositionLimitSettings_["active"].jointPositionUpperLimits[index]) ||
         (jointPositionsTarget[index] <= jointPositionLimitSettings_["active"].jointPositionLowerLimits[index]));
  }
  //  }
  return limitReached;
}

/** ManipulatorControl::isJointNearingLimits(...)
 * Checks if any of the specified joints are nearing their position limits based on
 * the given percentage of the allowed range. This function evaluates a target position
 * against both upper and lower limits for each joint in the specified range of indices.
 *
 * @param jointPositionsTarget A reference to a container holding the target positions
 * of joints to be evaluated.
 * @param percentage A scaling factor representing the percentage of the joint's allowed
 * range that determines when the joint is nearing its limit.
 * @param startIndex The starting index of the joint range to evaluate.
 * @param stopIndex The stopping (exclusive) index of the joint range to evaluate.
 * @return A boolean value indicating whether any joint in the specified range is
 * nearing its limits (true if at least one joint meets the criteria; false otherwise).
 *
 * Additional Details:
 * - Only evaluates joints that are not configured to operate in open-loop mode.
 * - Utilizes limit settings from the "active" limit configuration to determine upper and
 *   lower bounds for joint positions.
 */
bool ManipulatorControl::isJointNearingLimits(const JointPositions& jointPositionsTarget, double percentage,
                                              unsigned int startIndex, unsigned int stopIndex) {
  bool limitReached = false;
  for (size_t i = startIndex; i < stopIndex; i++) {
    if (!jointIsOpenLoop_[i]) {
      // 95% of stroke reached
      limitReached |=
          ((jointPositionsTarget[i] >=
            percentage * jointPositionLimitSettings_["active"].jointPositionUpperLimits[i]) ||
           (jointPositionsTarget[i] <= percentage * jointPositionLimitSettings_["active"].jointPositionLowerLimits[i]));
    }
  }
  return limitReached;
}

/** ManipulatorControl::getManipulatorToCameraViewRotationAngle(...)
 * Calculates the rotation angle required to align the manipulator's frame with the
 * camera's view frame.
 *
 * This method computes the relative rotation between the manipulator orientation
 * in the inertial frame and the camera's view direction, taking into account
 * a specified angle offset. The result is the angular difference needed to align
 * the manipulator's frame with the camera's view plane.
 *
 * @param manipulatorToCameraViewAngleOffset The angle offset to account for relative
 * to the manipulator's actual orientation.
 * @return The rotation angle (in radians) required to align the manipulator
 * with the camera's view frame.
 *
 * Calculation Details:
 * - Constructs and combines quaternions representing the manipulator's orientation
 *   to compute the composite rotation.
 * - Determines the relative positions of the camera and the manipulator's tool
 *   to derive direction vectors.
 * - Computes the angular difference based on cross products and dot products
 *   of orientation vectors, ensuring proper handling of singularities.
 * - Adjusts the angle's sign based on the direction of the cross product in the
 *   manipulator's frame.
 */
double ManipulatorControl::getManipulatorToCameraViewRotationAngle() const {
  // const mcx::math::Quaternion rotManipulatorToInertial(
  //     manipulatorToolPoseActual_[3], manipulatorToolPoseActual_[4], manipulatorToolPoseActual_[5]);
  const mcx::math::Quaternion rotManipulatorToInertial(
      globalPoses_.manipulatorToolPose[3], globalPoses_.manipulatorToolPose[4], globalPoses_.manipulatorToolPose[5]);
  const mcx::math::Vector3D posCamera{cameraPose_[0], cameraPose_[1], cameraPose_[2]}; // global pose
  // const mcx::math::Vector3D posInstrument{instrumentToolPoseActual_[0], instrumentToolPoseActual_[1],
  //                                         instrumentToolPoseActual_[2]};
  const mcx::math::Vector3D posInstrument{globalPoses_.instrumentToolPose[0], globalPoses_.instrumentToolPose[1],
                                          globalPoses_.instrumentToolPose[2]};
  mcx::math::Vector3D nzCamera = posCamera - posInstrument;
  nzCamera.normalize();
  const mcx::math::Vector3D nzManipulator = rotManipulatorToInertial.rotateVector(mcx::math::Vector3D{0.0, 0.0, 1.0});
  const mcx::math::Vector3D nyManipulator = rotManipulatorToInertial.rotateVector(mcx::math::Vector3D{0.0, 1.0, 0.0});
  const mcx::math::Vector3D cp = nzManipulator.cross(nzCamera);
  double deltaAngle = 0.0;
  if (cp.norm() > std::numeric_limits<double>::epsilon()) {
    const double val = mcx::control3::limit(nyManipulator.dot(cp) / (nyManipulator.norm() * cp.norm()), -1.0, 1.0);
    deltaAngle = std::acos(val);
    if (const mcx::math::Vector3D cpManipulatorFrame = rotManipulatorToInertial.transpose().rotateVector(cp);
        cpManipulatorFrame[0] > 0) {
      deltaAngle *= -1;
    }
  }

  return deltaAngle;
}

/** ManipulatorControl::getInstrumentToCameraRotationRzOffset()
 * Computes the rotational offset angle around the Z-axis (Rz) between the
 * current instrument frame and the camera frame in the inertial reference system.
 * The method calculates this angle by analyzing the relationship between the
 * orientation vectors of the instrument and the camera, using quaternion rotations
 * to project and compare these orientations.
 *
 * @return The computed Z-axis rotational offset angle in radians. The sign and
 * magnitude of the angle describe the rotation required to align the instrument's
 * and camera's coordinate frames in the inertial system.
 */
double ManipulatorControl::getInstrumentToCameraRotationRzOffset() const {
  const mcx::math::Quaternion rotCameraToInertial(cameraPose_[3], cameraPose_[4], cameraPose_[5]);
  // const mcx::math::Quaternion rotInstrumentToInertial(instrumentToolPoseActual_[3], instrumentToolPoseActual_[4],
  //                                                     instrumentToolPoseActual_[5]);
  const mcx::math::Quaternion rotInstrumentToInertial(
      globalPoses_.instrumentToolPose[3], globalPoses_.instrumentToolPose[4], globalPoses_.instrumentToolPose[5]);

  const mcx::math::Vector3D nyCamera = rotCameraToInertial.rotateVector(mcx::math::Vector3D{0.0, 1.0, 0.0});
  const mcx::math::Vector3D nzCamera = rotCameraToInertial.rotateVector(mcx::math::Vector3D{0.0, 0.0, 1.0});
  const mcx::math::Vector3D nyInstrument = rotInstrumentToInertial.rotateVector(mcx::math::Vector3D{0.0, 1.0, 0.0});
  const mcx::math::Vector3D nzInstrument = rotInstrumentToInertial.rotateVector(mcx::math::Vector3D{0.0, 0.0, 1.0});
  const mcx::math::Vector3D cp = nzInstrument.cross(nyCamera);
  double deltaAngle = 0.0;
  if (cp.norm() > std::numeric_limits<double>::epsilon()) {
    const double val = mcx::control3::limit(nyInstrument.dot(cp) / (nyInstrument.norm() * cp.norm()), -1.0, 1.0);
    deltaAngle = std::acos(val);
    if (const mcx::math::Vector3D cpInstrumentFrame = rotInstrumentToInertial.transpose().rotateVector(cp);
        cpInstrumentFrame[0] > 0) {
      deltaAngle *= -1;
    }
    deltaAngle = M_PI - deltaAngle;
  } else {
    if (const mcx::math::Vector3D cpNz = nzInstrument.cross(nzCamera);
        cpNz.norm() > std::numeric_limits<double>::epsilon()) {
      const double val = mcx::control3::limit(nyInstrument.dot(cpNz) / (nyInstrument.norm() * cpNz.norm()), -1.0, 1.0);
      deltaAngle = std::acos(val);
      if (const mcx::math::Vector3D cpInstrumentFrame = rotInstrumentToInertial.transpose().rotateVector(cpNz);
          cpInstrumentFrame[0] > 0) {
        deltaAngle *= -1;
      }
    }
  }
  return deltaAngle;
}

/** ManipulatorControl::calculateJointLimitForce()
 * Calculates the joint limit force to ensure the manipulator's joints operate
 * within predefined position and velocity constraints. This method accounts
 * for surgical mode and uses a limiter component to apply position-based
 * constraints on the joint forces.
 *
 * @param systemTime The system time parameter used to update and synchronize
 * calculations for the limiter component.
 * @param userTime A user-defined timestamp object for additional time-based
 * processing. It can be utilized internally by the limiter for control updates.
 *
 * @return A 6-dimensional vector representing the wrench (force and torque)
 * computed after applying the joint position limiting constraints.
 *
 * Functionality Details:
 * - Retrieves the manipulator's Jacobian matrix and target joint positions/velocities.
 * - Activates or disables the joint limiter calculations based on the surgical mode's state.
 * - Sets current joint positions and velocities in the limiter based on the active target values.
 * - Configures the upper and lower limits for joint positions using preloaded settings.
 * - Provides the Jacobian matrix and invokes an iterative computation step in the limiter.
 * - Outputs the wrench force resulting from joint limiting constraints.
 */
mcx::math::Vector6D ManipulatorControl::calculateJointLimitForce(const mcx::container::TaskTime& systemTime,
                                                                 mcx::container::UserTime* userTime) {
  const mcx::math::Matrix<6, NR_JOINT_FORCE_FEEDBACK> jac = getJacobian();
  mcx::math::Vector<NR_JOINT_FORCE_FEEDBACK> jointPositions{};
  mcx::math::Vector<NR_JOINT_FORCE_FEEDBACK> jointVelocities{};
  for (size_t cnt = 0; cnt < NR_JOINT_FORCE_FEEDBACK; cnt++) {
    jointPositions[cnt] = jointPvaTarget_.pos[cnt];
    jointVelocities[cnt] = jointPvaTarget_.vel[cnt];
  }

  // enable calculations whenever surgical is allowed.
  jointPositionLimiterForce_->setEnable(merilMode_.out.surgicalModeIsAllowed);

  jointPositionLimiterForce_->setJointPositions(jointPositions);
  jointPositionLimiterForce_->setJointVelocities(jointVelocities);

  mcx::math::Vector<NR_JOINT_FORCE_FEEDBACK> jointPositionsUpperLimit;
  mcx::math::Vector<NR_JOINT_FORCE_FEEDBACK> jointPositionsLowerLimit;
  for (size_t cnt = 0; cnt < NR_JOINT_FORCE_FEEDBACK; cnt++) {
    jointPositionsUpperLimit[cnt] = jointPositionLimitSettings_["active"].jointPositionUpperLimits[cnt];
    jointPositionsLowerLimit[cnt] = jointPositionLimitSettings_["active"].jointPositionLowerLimits[cnt];
  }

  jointPositionLimiterForce_->setJointPositionsLimit(jointPositionsUpperLimit, jointPositionsLowerLimit);

  jointPositionLimiterForce_->setJacobian(jac);
  jointPositionLimiterForce_->iterate(systemTime, userTime);
  return jointPositionLimiterForce_->getWrench();
}

/** ManipulatorControl::getJacobian()
 * Computes the Jacobian matrix for the manipulator, considering both the manipulator
 * joints and integrated instrument geometry. This includes transformations across
 * different coordinate frames, as well as the coupling between manipulator motions
 * and instrument tool dynamics.
 *
 * The method performs the following:
 * - Calculates the manipulator's Jacobian matrix based on its current joint kinematics.
 * - Computes the instrument's Jacobian matrix using the tool pose and relevant joint states.
 * - Constructs the manipulator-to-instrument Jacobian, incorporating the relationship
 *   between manipulator twist and tool joint velocities.
 * - Extends both the manipulator and manipulator-to-instrument Jacobians to include
 *   additional constraints by augmenting with appropriate zero and identity terms.
 * - Multiplies the resulting matrices together to produce the full manipulator-instrument
 *   joint Jacobian.
 *
 * @return The 6x8 Jacobian matrix that describes the relationship between the manipulator's
 * and instrument's joint velocities and tool motion in the operational workspace.
 *
 * Implementation Details:
 * - Rotational transformations are used to switch between inertial and manipulator frames.
 * - Zero-padding and diagonal extensions are applied to extend the matrices appropriately.
 * - Matrix operations (dot products) are used to combine partial Jacobians into a final result.
 */
mcx::math::Matrix<6, 8> ManipulatorControl::getJacobian() const {
  // get the manipulator jacTotal
  auto jacManipulator = mechanismModule_->getJacobian(jointPvaTarget_);
  // get the instrument jacTotal
  const mcx::math::Matrix6x6 jacInstrument =
      instrument_.getJacobian(manipulatorToolPoseTarget_, JointPositions{jointPvaTarget_.pos[INSTRUMENT_PITCH_JOINT],
                                                                         jointPvaTarget_.pos[INSTRUMENT_YAW_JOINT]});

  // get the jacTotal from manipulator twist to instrument joints velocities
  mcx::math::Matrix<4, 6> jacM2I{};

  const mcx::math::Rotation rotManipulator2Inertial(manipulatorToolPoseTarget_);
  const mcx::math::Rotation rotInertial2Manipulator(rotManipulator2Inertial.getMatrix().transpose());

  mcx::math::Matrix3x3 matInertial2Manipulator = rotInertial2Manipulator.getMatrix();
  const mcx::math::Vector3D nz{0, 0, 1};

  auto nzt = nz.transpose().dot(matInertial2Manipulator);
  for (size_t row = 0; row < 3; row++) {
    for (size_t col = 0; col < 3; col++) {
      jacM2I(row, col) = 0;
      jacM2I(row, col + 3) = matInertial2Manipulator(row, col);
    }
  }
  for (size_t col = 0; col < 3; col++) {
    jacM2I(3, col) = nzt[col];
    jacM2I(3, col + 3) = 0;
  }

  // extend the manipulator jacTotal with proper zeros and ones
  mcx::math::Matrix<8, 8> jacManipulatorExtended = 0;
  for (size_t row = 0; row < 6; row++) {
    for (size_t col = 0; col < 6; col++) {
      jacManipulatorExtended(row, col) = jacManipulator(row, col);
    }
  }
  jacManipulatorExtended(6, 6) = 1;
  jacManipulatorExtended(7, 7) = 1;

  // extend the m2i jacTotal with proper zeros and ones
  mcx::math::Matrix<6, 8> jacM2IExtended{};
  for (size_t row = 0; row < 4; row++) {
    for (size_t col = 0; col < 6; col++) {
      jacM2IExtended(row, col) = jacM2I(row, col);
    }
  }
  jacM2IExtended(4, 6) = 1;
  jacM2IExtended(5, 7) = 1;

  return jacInstrument.dot(jacM2IExtended.dot(jacManipulatorExtended));
}

/**
 * Adjusts the camera's movement velocity by modifying the rate at which the camera moves
 * in response to user input or automated controls.
 *
 * @param speed The desired speed for the camera's movement, typically specified as a
 * numerical value representing units per second.
 * @param directionVector A vector defining the direction of the camera's movement. This
 * indicates the axis or plane in which the camera should move.
 * @return A boolean value indicating whether the velocity adjustment was successfully
 * applied. Returns true if the adjustment was made; otherwise, false if an error occurred or
 * the parameters were invalid.
 *
 * Additional Details:
 * - The velocity adjustment considers any existing limits or constraints on the*/
mcx::math::Twist ManipulatorControl::cameraMoveVelocity(
    const mcx::math::Vector6D& inputVelocities, // Input velocities for translation of Left[0,1,2] and right[3,4,5].
    double deadband,                            // Deadband threshold for input offsets.
    mcx::math::Vector4D gain,                   // Gain for scaling resulting velocities.
    double translationScale,                    // Scaling factor for translation velocities.
    double orientationScale,                    // Scaling factor for orientation velocities.
    bool flip                                   // flips the camera view and movement direction x,y
) const {

  bool input1, input2;
  mcx::math::Vector3D magnitude;
  mcx::math::Vector4D direction{};
  mcx::math::Twist result;

  for (unsigned int cnt = 0; cnt < 3; cnt++) {
    magnitude[cnt] = std::min(std::fabs(inputVelocities[cnt]), std::fabs(inputVelocities[cnt + 3])) - deadband;
    if (std::abs(inputVelocities[cnt]) >= deadband && std::abs(inputVelocities[cnt + 3]) >= deadband) {
      input1 = (inputVelocities[cnt] > 0.0);
      input2 = (inputVelocities[cnt + 3] > 0.0);
      direction[cnt] = (input1 == input2) ? ((input1) ? 1.0 : -1.0) : 0.0;
      result[cnt] = magnitude[cnt] * direction[cnt] * translationScale * gain[cnt];
      if (cnt == 2) { // handcontroller up/down movement also creates the roll rotation.
        direction[3] = (input1 == input2) ? 0.0 : ((input1 && !input2) ? 1.0 : -1.0);
        result[3] = magnitude[2] * direction[3] * orientationScale * gain[3];
      }
    }
  }
  if (flip) {
    result[2] *= -1.0;
    result[1] *= -1.0;
  }
  return result;
}

/**
 * Configures the position limit parameters for a specific joint in the manipulator's control system.
 * This function adjusts the joint limiter gain lookup table for the specified joint index, taking
 * into account both lower and upper position limits as well as a limit factor to control the joint's
 * behavior within its constrained operating range.
 *
 * @param lowerPositionLimit The lower boundary of the joint's allowable position range.
 * @param upperPositionLimit The upper boundary of the joint's allowable position range.
 * @param limitFactor A bounded scaling factor (clamped between 0.01 and 1.0) regulating the gain
 *        applied to the joint for compliance with the position limits.
 * @param index An unsigned integer identifying the joint for which the limits and gain values are specified.
 *
 * Implementation Details:
 * - The `limitFactor` is constrained between 0.01 and 1.0 to ensure valid operation.
 * - Sets up the lookup table with four control points for both the `X` (position) and `Y` (gain) components.
 * - Adjusts the position limits relative to predefined scaling factors to produce effective limit
 *   thresholds for the joint's operation.
 * - Applies the specified `limitFactor` to the gain at boundary control points, ensuring smooth
 *   gain transitions between setpoints.
 */
void ManipulatorControl::setLimitPositions(double lowerPositionLimit, double upperPositionLimit, double limitFactor,
                                           double positionLimitOffset, unsigned int index) {

  limitFactor = mcx::control3::limit(limitFactor, 0.001, 1.0);
  jointLimiterGainLookup_[index].setNumPoints(4);
  jointLimiterGainLookup_[index].setX(lowerPositionLimit, 0);

  // lowerPositionLimit > 0 ? jointLimiterGainLookup_[index].setX(lowerPositionLimit / JOINT_LIMITING_FACTOR, 1)
  //                        : jointLimiterGainLookup_[index].setX(lowerPositionLimit * JOINT_LIMITING_FACTOR, 1);
  // upperPositionLimit < 0 ? jointLimiterGainLookup_[index].setX(upperPositionLimit / JOINT_LIMITING_FACTOR, 2)
  //                        : jointLimiterGainLookup_[index].setX(upperPositionLimit * JOINT_LIMITING_FACTOR, 2);

  // use fixed values
  jointLimiterGainLookup_[index].setX(lowerPositionLimit + positionLimitOffset, 1); // default 15 deg
  jointLimiterGainLookup_[index].setX(upperPositionLimit - positionLimitOffset, 2);

  jointLimiterGainLookup_[index].setX(upperPositionLimit, 3);

  jointLimiterGainLookup_[index].setY(limitFactor, 0);
  jointLimiterGainLookup_[index].setY(1.0, 1);
  jointLimiterGainLookup_[index].setY(1.0, 2);
  jointLimiterGainLookup_[index].setY(limitFactor, 3);
};

mcx::utils::span<const double> ManipulatorControl::semiAutoMode(const mcx::container::TaskTime& systemTime,
                                                                mcx::container::UserTime* userTime,
                                                                const JointPositions& syncJointPositions) {

  activateSemiAutoWatchdog_.timeoutSec = jointVelocityWatchdog_.timeoutSec;
  activateSemiAutoWatchdog_.check(activateSemiAutoHandle_, activateSemiAuto_);

  semiAutoTimeScaleSwitch_.setInput1(0.0);
  semiAutoTimeScaleSwitch_.setInput2(1.0);
  semiAutoTimeScaleSwitch_.setToggle(activateSemiAuto_);
  semiAutoTimeScaleSwitch_.iterate(systemTime, userTime);

  semiAutoMotionGenerator_.setRunSpeedFactor(semiAutoTimeScaleSwitch_.getOutput()[0]);
  semiAutoMotionGenerator_.setActualJointPositions(syncJointPositions);
  semiAutoMotionGenerator_.setEngaged(semiAutoModeSwitch_.isOn());
  semiAutoMotionGenerator_.iterate(systemTime, userTime);

  return semiAutoMotionGenerator_.getJointSetpoints();
}

/**
 * Converts a Cartesian pose from local to global coordinates.
 *
 * This function takes a pose defined in the local coordinate frame and transforms it into the global
 * coordinate frame by applying a position offset and a rotation offset. The transformation involves
 * rotating and translating the position values and applying a quaternion rotation to the orientation.
 *
 * @param[in] poseLocal The Cartesian pose in the local coordinate frame. It consists of position
 *                      (x, y, z) and orientation (roll, pitch, yaw).
 * @param[in] positionOffset The position offset to be subtracted before applying the rotation.
 * @param[in] rotationOffset The quaternion rotation offset to transform the local pose to the global pose.
 * @param[out] poseGlobal The transformed Cartesian pose in the global coordinate frame.
 *                        It consists of position (x, y, z) and orientation (roll, pitch, yaw).
 */
void ManipulatorControl::convertLocalToGlobalPoses(const CartPose6& poseLocal,
                                                   const mcx::math::Position& positionOffset,
                                                   const mcx::math::Quaternion& rotationOffset, CartPose6& poseGlobal) {
  mcx::math::Position positionGlobal =
      rotationOffset.rotateVector(mcx::math::Position{poseLocal[0], poseLocal[1], poseLocal[2]} - positionOffset);
  for (unsigned int cnt = 0; cnt < positionGlobal.size(); cnt++) {
    poseGlobal[cnt] = positionGlobal[cnt];
  }
  mcx::math::Quaternion rotationGlobal = rotationOffset * mcx::math::Quaternion{poseLocal[3], poseLocal[4], poseLocal[5]};
  rotationGlobal.getEuler(poseGlobal[3], poseGlobal[4], poseGlobal[5]);
}

/**
 * Converts a global Cartesian pose to a local Cartesian pose using specified positional
 * and rotational offsets.
 *
 * This function applies a rotational transformation using the transpose of the rotation
 * offset to the position components of the global pose. The transformed position is then
 * offset by the given positional offset to determine the local position. Similarly, the
 * orientation components of the global pose are transformed using the transpose of the
 * rotation offset, and the resulting quaternion is converted to Euler angles for the
 * local pose.
 *
 * @param[in] poseGlobal The global Cartesian pose, consisting of position (x, y, z) and
 *                       orientation (roll, pitch, yaw).
 * @param[in] positionOffset A positional offset to apply after converting the global pose's
 *                            position component to the local frame.
 * @param[in] rotationOffset A rotational offset (quaternion) used to transform the global
 *                            pose into the local reference frame.
 * @param[out] poseLocal The resulting local Cartesian pose after applying the positional and
 *                        rotational transformations, represented as position (x, y, z) and
 *                        orientation (roll, pitch, yaw).
 */
void ManipulatorControl::convertGlobalToLocalPoses(const CartPose6& poseGlobal,
                                                   const mcx::math::Position& positionOffset,
                                                   const mcx::math::Quaternion& rotationOffset, CartPose6& poseLocal) {
  mcx::math::Position positionLocal =
      rotationOffset.transpose().rotateVector(mcx::math::Position{poseGlobal[0], poseGlobal[1], poseGlobal[2]});
  for (unsigned int cnt = 0; cnt < 3; cnt++) {
    poseLocal[cnt] = positionLocal[cnt] + positionOffset[cnt];
  }
  mcx::math::Quaternion rotationLocal =
      rotationOffset.transpose() * mcx::math::Quaternion{poseGlobal[3], poseGlobal[4], poseGlobal[5]};
  rotationLocal.getEuler(poseLocal[3], poseLocal[4], poseLocal[5]);
}

/**
 * @brief Converts global poses to local poses and vice versa for various components such as the camera, screen,
 * instrument, hand controller, and manipulator.
 *
 * This function performs the following operations:
 * - Computes the orientation and position offsets needed for coordinate transformations.
 * - Converts the fulcrum port position using the computed orientation offset and notifies the instrument module.
 * - Converts global poses of components (camera, screen, hand controller) to local poses accounting for offsets.
 * - Converts local poses of components (manipulator base, tool, fulcrum) to global poses considering offsets.
 *
 * The function uses the base orientation offset and position offset, adjusting Z-axis translation if necessary,
 * to perform accurate transformations between global and local coordinate systems.
 *
 * Key transformations include:
 * - Conversion of the camera's global pose to a local pose.
 * - Direct assignment of the screen and hand controller global poses (no orientation changes required).
 * - Calculation of the manipulator base, manipulator tool, instrument tool, and fulcrum poses in the global frame
 *   from their respective local poses.
 */
void ManipulatorControl::convertGlobalAndLocalPoses() {
  // get the orientation offset
  mcx::math::Quaternion rotBaseOffset(globalPoses_.baseOrientationOffset[0], globalPoses_.baseOrientationOffset[1],
                                      globalPoses_.baseOrientationOffset[2]);

  // inform the instrument module about the fulcrum port position
  mcx::math::Position fulcrumPortPosition = rotBaseOffset.transpose().rotateVector(mcx::math::Position{
      globalPoses_.fulcrumPortPose[0], globalPoses_.fulcrumPortPose[1], globalPoses_.fulcrumPortPose[2]});
  instrument_.setFulcrumPortPosition(fulcrumPortPosition);

  // get the position offset, discard the offset in Z
  mcx::math::Position posBaseOffset = instrument_.getFulcrumPositionOffset();
  posBaseOffset[2] = 0;

  /** global to local conversion **/
  // camera local pose
  convertGlobalToLocalPoses(globalPoses_.cameraPose, posBaseOffset, rotBaseOffset, cameraPose_);

  /** local to global conversion **/
  // base global pose
  CartPose6 basePose = mechanismModule_->mechanism().baseOffset();
  convertLocalToGlobalPoses(basePose, posBaseOffset, rotBaseOffset, globalPoses_.manipulatorBasePose);

  // manipulator global pose
  convertLocalToGlobalPoses(manipulatorToolPoseActual_, posBaseOffset, rotBaseOffset, globalPoses_.manipulatorToolPose);

  // instrument global pose
  convertLocalToGlobalPoses(instrumentToolPoseActual_, posBaseOffset, rotBaseOffset, globalPoses_.instrumentToolPose);

  // fulcrum global pose
  CartPose6 fulcrumPose = instrument_.getFulcrumPose();
  convertLocalToGlobalPoses(fulcrumPose, posBaseOffset, rotBaseOffset, globalPoses_.fulcrumPose);
}

/**
 * @brief Calculates the global camera view transformation based on the position and orientation
 *        of the instrument tool and the camera's relative offset in the local frame.
 *
 * This method updates the global camera position and orientation (`cameraPoseView_`)
 * by calculating the transformed position and rotation of the camera in the global inertial frame,
 * using the instrument tool's pose and orientation as a reference.
 *
 * Steps:
 * - Converts the offset of the camera from the local tool frame to the inertial (global) frame
 *   using the instrument tool's orientation.
 * - Adjusts the camera's global position, adding the transformed offset to the tool's position.
 * - Computes the global camera rotation by combining the tool's rotation with the local camera's
 *   orientation, then extracts Euler angles.
 */
void ManipulatorControl::calculateGlobalCameraView() {
  const mcx::math::Quaternion rotInstrumentTool2Inertial(
      globalPoses_.instrumentToolPose[3], globalPoses_.instrumentToolPose[4], globalPoses_.instrumentToolPose[5]);

  // get the camera in the local frame. The cameraPoseView is received in the global frame:
  auto cameraPoseViewOffset = cameraPoseViewOffset_;

  // adjusted tool position
  mcx::math::Position cameraViewOffsetPositionInertialFrame = rotInstrumentTool2Inertial.rotateVector(
      mcx::math::Position(cameraPoseViewOffset[0], cameraPoseViewOffset[1], cameraPoseViewOffset[2]));
  for (size_t i = 0; i < cameraViewOffsetPositionInertialFrame.size(); i++) {
    cameraPoseView_[i] = globalPoses_.instrumentToolPose[i] + cameraViewOffsetPositionInertialFrame[i]; // global frame
  }
  //  adjusted tool rotation
  const mcx::math::Quaternion rotCameraPoseView2Inertial =
      rotInstrumentTool2Inertial *
      mcx::math::Quaternion(cameraPoseViewOffset[3], cameraPoseViewOffset[4], cameraPoseViewOffset[5]);
  rotCameraPoseView2Inertial.getEuler(cameraPoseView_[3], cameraPoseView_[4], cameraPoseView_[5]);
}

/**
 * Initializes and configures Kalman observers for joint state estimation.
 * The joint Kalman observers are utilized to provide real-time estimation
 * of joint states such as position, velocity, or acceleration, improving
 * the accuracy and reliability of the system by filtering sensor noise
 * and handling potential measurement inaccuracies.
 *
 * @param observerConfigurations A data structure that contains configuration
 * parameters for each joint observer, including noise covariance matrices and
 * observer settings tailored to specific joint dynamics.
 * @param jointCount The number of joints in the system for which Kalman observers
 * need to be initialized and operated.
 * @return A collection of Kalman observers, each configured for their respective
 * joint, ready to process input measurements and provide estimated states.
 *
 * Additional Details:
 * - Initializes observer parameters based on predefined configurations.
 * - Allocates computation resources for real-time state estimation.
 * - Supports adaptive updates for time-varying systems or changing noise profiles.
 * - Ensures robust performance under varying operational conditions through
 * reinitialization and error correction mechanisms.
 */
void ManipulatorControl::jointKalmanObservers(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) {
  // get torque and position estimates using an extended kalman observer:
  mcx::math::Matrix<3, 1> initVector;
  for (unsigned int cnt = 0; cnt < numberOfJoints_; cnt++) {
    initVector(0, 0) = jointPvaActual_.pos[cnt];
    initVector(1, 0) = jointPvaActual_.vel[cnt];
    initVector(2, 0) = jointTorquesActual_[cnt];
    if (pauseModeSwitch_.isOn()) {
      jointKalmanObservers_[cnt].setInitialState(initVector);
    }
    jointKalmanObservers_[cnt].setDisable(pauseModeSwitch_.isOn());

    // measurementVector(0, 0) = jointPvaActual_.pos[cnt];
    // measurementVector(1, 0) = jointPvaActual_.vel[cnt];
    // measurementVector(2, 0) = jointTorquesActual_[cnt];
    if (cnt < numberOfManipulatorJoints_) { // not for instrument joints
      jointKalmanObservers_[cnt].setModelInertia(idJointVirtualMassActual_[cnt]);
    }
    jointKalmanObservers_[cnt].setInput(idJointTorqueReference_.torque[cnt]);
    jointKalmanObservers_[cnt].setMeasurementVector(initVector);
    jointKalmanObservers_[cnt].iterate(systemTime, userTime);
    jointPVTEstimate_.pos[cnt] = jointKalmanObservers_[cnt].getStateVector()[0];
    jointPVTEstimate_.vel[cnt] = jointKalmanObservers_[cnt].getStateVector()[1];
    jointPVTEstimate_.acc[cnt] = jointKalmanObservers_[cnt].getStateVector()[2];
  }
}

/**
 * Executes one iteration of the manipulator's operation cycle. This method updates the manipulator
 * and instrument states, including forward and inverse kinematics, joint torques, manipulability,
 * user inputs, and system parameters. It ensures synchronization between manipulator and instrument
 * components while monitoring limits, validity conditions, and operational modes.
 *
 * @param systemTime The system's task time encapsulated in a container for synchronization
 * between components.
 * @param userTime A pointer to the user-specific time container, where results or intermediate
 * operations requiring user-specific timing can be updated.
 * @return Returns true if the operation cycle completes successfully, indicating all required
 * updates and evaluations were performed within the iteration.
 *
 * Key Operations:
 * - Iterates the mechanism module for the manipulator, updating dynamics and kinematics.
 * - Computes forward kinematics to derive the operational tool pose and update it within the
 *   instrument module.
 * - Calculates manipulability metrics and updates its state in relevant components.
 * - Evaluates operational limits for both manipulator and instrument joints.
 * - Updates the instrument's forward kinematics with the latest tool and joint configuration.
 * - Computes adjustments for camera view offsets when the manipulator serves a camera robot function.
 * - Executes inverse dynamics to calculate joint torques and masses required.
 * - Processes control modes like manual and synchronization for manipulator and instrument operations.
 * - Monitors the validity and status of the fulcrum point with a watchdog mechanism.
 * - Evaluates whether manipulator and instrument joints are approaching operational limits.
 * - Ensures operational mode validity, such as surgical mode, based on the fulcrum and input conditions.
 */
bool ManipulatorControl::iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) {

  // calculate joint estimates (position, velocity & torque actual)
  jointKalmanObservers(systemTime, userTime);

  // Iterate mechanism
  mechanismModule_->iterate(systemTime, userTime);

  /// MECHANISM: Forward Kinematics for actual robot coordinates
  CartPoseOut cartPoseOut;
  mechanismModule_->jointToCart(jointPvaActual_, mcx::math::Pose{manipulatorToolPoseActual_}, cartPoseOut);

  /* calculate manipulability of the manipulator.   */
  manipulabilityActual_ = mechanismModule_->jointToManipulability(jointPvaActual_);

  manipulatorToolPoseActual_ = cartPoseOut.toolCoord.getCartPose6();

  // update the actual manipulator pose stored in the instrument module
  instrument_.setManipulatorToolPose(manipulatorToolPoseActual_);

  // update the joint positions stored in the instrument module
  instrument_.setJointPositions(mcx::control3::JointPositions{
      jointPvaTarget_.pos[INSTRUMENT_PITCH_JOINT], jointPvaTarget_.pos[INSTRUMENT_YAW_JOINT],
      jointPvaTarget_.pos[INSTRUMENT_PINCH_JOINT], jointPvaTarget_.pos[INSTRUMENT_ROLL_JOINT]});

  instrument_.setJointPositionsLimit(
      mcx::control3::JointPositions{
          jointPositionLimitSettings_["active"].jointPositionUpperLimits[INSTRUMENT_PITCH_JOINT],
          jointPositionLimitSettings_["active"].jointPositionUpperLimits[INSTRUMENT_YAW_JOINT],
          jointPositionLimitSettings_["active"].jointPositionUpperLimits[INSTRUMENT_PINCH_JOINT],
          jointPositionLimitSettings_["active"].jointPositionUpperLimits[INSTRUMENT_ROLL_JOINT]},
      mcx::control3::JointPositions{
          jointPositionLimitSettings_["active"].jointPositionLowerLimits[INSTRUMENT_PITCH_JOINT],
          jointPositionLimitSettings_["active"].jointPositionLowerLimits[INSTRUMENT_YAW_JOINT],
          jointPositionLimitSettings_["active"].jointPositionLowerLimits[INSTRUMENT_PINCH_JOINT],
          jointPositionLimitSettings_["active"].jointPositionLowerLimits[INSTRUMENT_ROLL_JOINT]});

  // teach fulcrum
  instrument_.setTeachFulcrum(merilMode_.in.gotoTeachFulcrum);

  // Iterate instrument
  instrument_.iterate(systemTime, userTime);

  // instrument forward kinematics
  instrument_.updateForwardKin(manipulatorToolPoseActual_,
                               mcx::control3::JointPositions{jointPvaActual_.pos[INSTRUMENT_PITCH_JOINT],
                                                             jointPvaActual_.pos[INSTRUMENT_YAW_JOINT],
                                                             jointPvaActual_.pos[INSTRUMENT_PINCH_JOINT],
                                                             jointPvaActual_.pos[INSTRUMENT_ROLL_JOINT]},
                               instrumentToolPoseActual_);

  // pose transformation between the multi-robot global frame and individual robot local frame
  convertGlobalAndLocalPoses();

  // calculate the camera view in the multi-robot global frame
  calculateGlobalCameraView();

  // calculate the angle between the instrument facade and camera view
  manipulatorToCameraViewRotationAngle_ = getManipulatorToCameraViewRotationAngle();

  /// MECHANISM: Inverse Dynamics
  mechanismModule_->calcJointTorques(jointPvaActual_, {}, idJointTorqueActual_);
  mechanismModule_->calcJointTorques(jointPvaLimitedReference_, {}, idJointTorqueReference_);
  mechanismModule_->calcJointMasses(jointPvaActual_, idJointVirtualMassActual_);

  // Manipulability Window Detector
  manipulabilityDetector_.setInput(manipulabilityActual_);
  if (state_.in.resetErrors) {
    manipulabilityDetector_.reset();
  }

  const auto manualJointIsActive = manualModeSwitch_.isOn() && jointModeSwitch_.isOn();
  manipulabilityDetector_.setDisable(manualJointIsActive);
  manipulabilityDetector_.iterate(systemTime, userTime);

  // Manipulability Gain Lookup
  manipulabilityGainLookup_.setInput(manipulabilityActual_);
  manipulabilityGainLookup_.iterate(systemTime, userTime);
  const auto manipulabilityGain = manipulabilityGainLookup_.getOutput();

  /** check if the robot is used for the camera **/
  merilMode_.out.isCameraRobot = !poseSyncInSurgicalEnable_;

  /** I am Syncing, I am Syncing **/
  auto syncJointPositions = getSyncJointOutput(jointIsOpenLoop_, state_.in.syncControlLoop, jointPvaActual_.pos, jointPvaTarget_.pos);
  const auto syncToolCoordinates = getSyncToolCartOutput(isAnyPositionLoopOpen(jointIsOpenLoop_));
  const auto syncManipulatorCoordinates = getSyncManipulatorCartOutput(isAnyPositionLoopOpen(jointIsOpenLoop_));

  // Run Cartesian Manual Mode
  auto& toolManualCoords = manualCartMode(systemTime, userTime, manipulabilityGain, syncJointPositions,
                                          syncToolCoordinates, syncManipulatorCoordinates);

  // Run Joint Manual Mode
  JointPositions jointManualCoords = manualJointMode(systemTime, userTime, syncJointPositions);

  /**
   * monitor fulcrum status with a watchdog
   * fulcrum is valid can be shortly interupted which should not cause surgical mode to exit.
   * watchdog stime can be set after which the manipulator will go to pauseMode (locked).
   * fulcrum valid watchdog, if fulcrum not valid > t[sec] error/warning occurs.
   **/
  merilMode_.out.fulcrumIsValid = instrument_.isFulcrumValid();
  if (!merilMode_.out.fulcrumIsValid &&
      merilMode_.out.fulcrumIsStored) {
    fulcrum_.watchdog.timerSec += getDtSec();
    if (fulcrum_.watchdog.timerSec >= fulcrum_.watchdog.timeoutSec) {
      fulcrum_.watchdog.active = true;
      fulcrum_.watchdog.timerSec = fulcrum_.watchdog.timeoutSec;
    }
  } else {
    fulcrum_.watchdog.timerSec = 0.0;
    fulcrum_.watchdog.active = false;
  }

  /** check if the manipulator joints, excluding joint 6, are limiting. **/
  merilMode_.out.jointManipulatorIsLimiting =
      isJointNearingLimits(jointPvaActual_.pos, 1.0, 0, numberOfManipulatorJoints_ - 1);

  /** check if the instrument joints are limiting. **/
  merilMode_.out.jointInstrumentIsLimiting =
      isJointNearingLimits(jointPvaActual_.pos, 1.0, numberOfManipulatorJoints_ - 1, features_.numberOfJoints);

  // surgical mode allowed?
  if (merilMode_.out.surgicalModeIsAllowed) {
    // if surgical mode is allowed it can only be broken by the fulcrum. upon losing the fulcrum it will go to locked.
    //    merilMode_.out.surgicalModeIsAllowed = merilMode_.out.fulcrumIsValid;   // disable and let this be governed
    //    by the watchdog instead. --> move to logic
    merilMode_.out.surgicalModeIsAllowed = !fulcrum_.watchdog.active && !instrument_.minInsertionDepthLimitReached() &&
                                           !instrument_.isOutsideFulcrumPort();
  } else {
    // condition upon entering surgical mode. the instrument should not be constrained + fulcrum is valid.
    merilMode_.out.surgicalModeIsAllowed =
        merilMode_.out.fulcrumIsValid && (!instrument_.isConstraintViolated()) &&
        (!instrument_.minInsertionDepthLimitReached()) && (!instrument_.isOutsideFulcrumPort()) &&
        (!merilMode_.out.jointManipulatorIsLimiting) && (!merilMode_.out.jointInstrumentIsLimiting);
  }

  // reset fulcrum by holding teach button outside fulcrum or maintenance mode active
  if (merilMode_.in.gotoResetFulcrum) {
    instrument_.resetFulcrumPose();
    fulcrum_.reset = false;
  }

  if (pauseModeSwitch_.isOn()) {
    // Prevent going to cart mode when fulcrum is not valid
    jointModeSwitch_.setToggle(mode_.in.gotoJointMode);
  }
  /** joint switch switches between cartesian mode and joint mode. **/
  jointModeSwitch_.setInput1(toolManualCoords);  // off
  jointModeSwitch_.setInput2(jointManualCoords); // on
  jointModeSwitch_.iterate(systemTime, userTime);
  mode_.out.jointModeSwitchOn = jointModeSwitch_.isOn();
  mode_.out.jointModeSwitchOff = jointModeSwitch_.isOff();

  if (mode_.out.jointModeSwitchOff) {
    // disable retract so it has to be taught again. this prevents a jump when switching jointMode
    merilMode_.in.enableManipulatorManualLinear = false;
    merilMode_.in.gotoManipulatorManual = false;
  }

  // Semi-auto Mode Switch
  semiAutoModeSwitch_.setInput1(jointModeSwitch_.getOutput());                           // off
  semiAutoModeSwitch_.setInput2(semiAutoMode(systemTime, userTime, syncJointPositions)); // on
  if (pauseModeSwitch_.isOn()) {
    semiAutoModeSwitch_.setToggle(mode_.in.gotoSemiautoMode);
  }
  semiAutoModeSwitch_.iterate(systemTime, userTime);
  mode_.out.semiautoModeSwitchOn = semiAutoModeSwitch_.isOn();
  mode_.out.semiautoModeSwitchOff = semiAutoModeSwitch_.isOff();

  // Manual Mode Switch
  if (pauseModeSwitch_.isOn()) {
    manualModeSwitch_.setToggle(mode_.in.gotoManualMode);
  }
  manualModeSwitch_.setInput1(semiAutoModeSwitch_.getOutput()); // off
  manualModeSwitch_.setInput2(jointModeSwitch_.getOutput());    // on
  manualModeSwitch_.iterate(systemTime, userTime);
  mode_.out.manualModeSwitchOn = manualModeSwitch_.isOn();
  mode_.out.manualModeSwitchOff = manualModeSwitch_.isOff();

  mcx::utils::span<double>{manualModeSwitchPositions_} = manualModeSwitch_.getOutput();

  // syncs the drives that are in torque control
  for (size_t cnt = 0; cnt < numberOfManipulatorJoints_; cnt++) {
    if (driveMode_[cnt] == mcx::drive::DRIVE_MODE_CYCLIC_SYNC_TORQUE) {
      manualModeSwitchPositions_[cnt] = syncJointPositions[cnt];
    }
  }

  // Pause Mode Switch
  pauseModeSwitch_.setToggle(mode_.in.gotoPauseMode);
  pauseModeSwitch_.setInput1(manualModeSwitchPositions_); // off
  pauseModeSwitch_.setInput2(syncJointPositions);         // on
  pauseModeSwitch_.iterate(systemTime, userTime);
  mode_.out.pauseModeSwitchOn = pauseModeSwitch_.isOn();
  mode_.out.pauseModeSwitchOff = pauseModeSwitch_.isOff();

  jointPVATargetFilter_.setInput(pauseModeSwitch_.getOutput());
  jointPVATargetFilter_.iterate(systemTime, userTime);
  mcx::utils::span<double>{jointPvaTarget_.pos} = jointPVATargetFilter_.getOutput();
  mcx::utils::span<double>{jointPvaTarget_.vel} = jointPVATargetFilter_.getOutputDot();
  mcx::utils::span<double>{jointPvaTarget_.acc} = jointPVATargetFilter_.getOutputDDot();

  // pinch force controller
  double pinchForceError;
  (jointPvaTarget_.pos[INSTRUMENT_PINCH_JOINT] < 0.01)
      ? pinchForceError = pinchControlTargetForce_ - jointPVTEstimate_.acc[INSTRUMENT_PINCH_JOINT]
      : pinchForceError = 0.0;
  pinchController_.setError(pinchForceError);
  pinchController_.iterate(systemTime, userTime);
  jointPvaTarget_.pos[INSTRUMENT_PINCH_JOINT] += (mcx::control3::limit(pinchController_.getOutput(), -0.1, 0.1));

  // Forward Kinematics for Limited Reference coordinates
  mechanismModule_->jointToCart(jointPvaLimitedReference_, mcx::math::Pose{manipulatorToolPoseLimitedReference_},
                                cartPoseOut);
  manipulatorToolPoseLimitedReference_ = cartPoseOut.toolCoord.getCartPose6();

  instrument_.updateForwardKin(manipulatorToolPoseLimitedReference_,
                               mcx::control3::JointPositions{jointPvaLimitedReference_.pos[INSTRUMENT_PITCH_JOINT],
                                                             jointPvaLimitedReference_.pos[INSTRUMENT_YAW_JOINT],
                                                             jointPvaLimitedReference_.pos[INSTRUMENT_PINCH_JOINT],
                                                             jointPvaLimitedReference_.pos[INSTRUMENT_ROLL_JOINT]},
                               instrumentToolPoseLimitedReference_);

  // update planning mode
  planningCartMode();

  return true;
}

} // namespace control
