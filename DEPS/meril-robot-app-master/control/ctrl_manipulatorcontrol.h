/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef CTRL_MANIPULATORCONTROL_H
#define CTRL_MANIPULATORCONTROL_H

#include "app/app_feature.h"
#include "ctrl_cartposeratelimiter.h"
#include "ctrl_compliancecontrol.h"
#include "ctrl_hostin.h"
#include "ctrl_instrumentcontrol.h"
#include "ctrl_jointlimitforcefeedback.h"
#include "ctrl_semiauto.h"
#include "ctrl_symbolicpositionmodule.h"
#include "ctrl_observerkalmanfilter.h"
#include "logic/lgc_robot_modes_data.h"
#include "logic/meril/lgc_meril_modes_data.h"
#include <mcx/control3.h>
#include <mcx/control3/ctrl_states.h>
#include <mcx/math.h>
#include <mcx/mechanics.h>
#include <mcx/motion.h>
#include <vector>

namespace control {
class ManipulatorControl final : public mcx::container::Module {
  using lookUpTables = std::vector<mcx::control3::Lookup>;
  using observerKalmanFilters = std::vector<ObserverKalmanFilter<3,1,3>>;

  enum ComplianceSetting { J123_ADMITTANCE = 0, J345_ADMITTANCE = 1, ALL_ADMITTANCE = 2 };

  /// Position limit sets for symbolic position
  struct PositionLimitSettings {
    mcx::control3::JointPositions jointPositionUpperLimits;
    mcx::control3::JointPositions jointPositionLowerLimits;
  };

  /// Pinch is connected to which joint
  static constexpr uint ROBOT_JOINT_1 = 0;
  static constexpr uint ROBOT_JOINT_2 = 1;
  static constexpr uint ROBOT_JOINT_3 = 2;
  static constexpr uint ROBOT_JOINT_4 = 3;
  static constexpr uint ROBOT_JOINT_5 = 4;
  static constexpr uint ROBOT_JOINT_6 = 5;
  static constexpr uint INSTRUMENT_PITCH_JOINT = 6;
  static constexpr uint INSTRUMENT_YAW_JOINT = 7;
  static constexpr uint INSTRUMENT_PINCH_JOINT = 8;
  static constexpr uint INSTRUMENT_ROLL_JOINT = 9;

  /// Instrument Symbolic Joint Numbers
  static constexpr uint SP_INSTRUMENT_ROBOT_ROLL_JOINT = 0;
  static constexpr uint SP_INSTRUMENT_PITCH_JOINT = 1;
  static constexpr uint SP_INSTRUMENT_YAW_JOINT = 2;
  static constexpr uint SP_INSTRUMENT_PINCH_JOINT = 3;
  static constexpr uint SP_INSTRUMENT_ROLL_JOINT = 4;

  static constexpr uint SP_INSTRUMENT_STRAIGHTEN = 7;
  static constexpr uint SP_INSTRUMENT_STRAIGHTEN_ROLL = 8;
  static constexpr uint SP_ADAPTER_CALIBRATE_NEGATIVE = 9;
  static constexpr uint SP_ADAPTER_CALIBRATE_POSITIVE = 10;

  /// Symbolic positions
  static constexpr uint SP_IDLE = 0;
  static constexpr uint SP_START_HIGH = 1;
  static constexpr uint SP_START_LOW = 2;
  static constexpr uint SP_PARK = 3;
  static constexpr uint SP_EXCHANGE = 4;

  /// Straighten threshold
  static constexpr double JOINT_STRAIGHTEN_THRESHOLD = 1e-3;

  /// force feedback CONSTANTS
  static constexpr uint NR_JOINT_FORCE_FEEDBACK = 8;
  static constexpr uint WRENCH_FORCE_X = 0;
  static constexpr uint WRENCH_FORCE_Y = 1;
  static constexpr uint WRENCH_FORCE_Z = 2;
  static constexpr uint WRENCH_TORQUE_RX = 3;
  static constexpr uint WRENCH_TORQUE_RY = 4;
  static constexpr uint WRENCH_TORQUE_RZ = 5;

  /// calibration insertion depth
  static constexpr double JOINT_LIMITING_FACTOR =
      0.90; // [starting at 95% of joint limits the cartTransGain starts to reduce]

  /// calibration insertion depth
  static constexpr double INSTRUMENT_CALIBRATION_POSITION = 0.01;         // [m] @ 1 cm below fulcrum
  static constexpr double INSTRUMENT_CALIBRATION_MAXIMUM_POSITION = 0.02; // [m] @ 1 cm below fulcrum

  /// auto straightening
  static constexpr double MINIMUM_AUTO_STRAIGHTEN_INSERTION_DEPTH = 0.01; // [m] @ 0.1 cm below minimum insertion depth

  /// default camera handcontroller velocity deadband
  static constexpr double DEFAULT_CAMERA_HANDCONTROLLER_VELOCITY_DEADBAND =
      0.01; // [m] @ 0.1 cm below minimum insertion depth

public:
  ManipulatorControl() = delete;

  ManipulatorControl(std::unique_ptr<mcx::mechanics::MechanismModule> mechanismModule, feature::Manipulator features);

private:
  /**
   * The primary purpose of this function is to check if Cartesian jog mode is synchronized with the mode integrator.
   * It determines activation based on the state of two mode switches (pauseModeSwitch_ and jointModeSwitch_). This
   * is used in scenarios where motion control or mode management depends on these switches being in specific states.
   */
  [[nodiscard]] bool syncCartJogModeIntegrator() const { return pauseModeSwitch_.isOn() || jointModeSwitch_.isOn(); }

  /**
   *
   */
  [[nodiscard]] bool syncJointJogModeIntegrator() const { return pauseModeSwitch_.isOn() || jointModeSwitch_.isOff(); }

  [[nodiscard]] const CartPose6& getSyncToolCartOutput(bool isPositionLoopOpen) const {
    return isPositionLoopOpen ? instrumentToolPoseActual_ : instrumentToolPoseLimitedReference_;
  }

  [[nodiscard]] const CartPose6& getSyncManipulatorCartOutput(bool isPositionLoopOpen) const {
    return isPositionLoopOpen ? manipulatorToolPoseActual_ : manipulatorToolPoseLimitedReference_;
  }

  [[nodiscard]] double getManipulatorToCameraViewRotationAngle() const;

  [[nodiscard]] double getInstrumentToCameraRotationRzOffset() const;

  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override;

  bool initPhase1_() override;

  bool initPhase2_() override { return true; }

  bool startOp_() override;

  mcx::math::Quaternion handcontrollerToInertialFrame(bool syncCart);

  const JointPositions& manualCartMode(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime,
                                       double manipulabilityGain, const JointPositions& syncJointPositions,
                                       const CartPose6& syncToolCoordinates,
                                       const CartPose6& syncManipulatorCoordinates);

  const JointPositions& manualJointMode(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime,
                                        const JointPositions& syncJointPositions);

  mcx::utils::span<const double> semiAutoMode(const mcx::container::TaskTime& systemTime,
                                              mcx::container::UserTime* userTime,
                                              const JointPositions& syncJointPositions);

  void symbolicJointMode(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime,
                         JointPositions& jointManualPositionsTarget, const JointPositions& syncJointPositions);

  void planningCartMode();

  void instrumentAdjust(JointPositions& jointManualPositionsTarget, const JointPositions& syncJointPositions);

  void jointKalmanObservers(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime);

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override;

  bool stopOp_() override { return true; }

  bool isJointLimitsReached(const JointPositions& jointPositionsTarget, unsigned int index);

  bool isJointNearingLimits(const JointPositions& jointPositionsTarget, double percentage, unsigned int startIndex,
                            unsigned int stopIndex);

  mcx::math::Vector6D calculateJointLimitForce(const mcx::container::TaskTime& systemTime,
                                               mcx::container::UserTime* userTime);

  mcx::math::Matrix<6, 8> getJacobian() const;

  // Function to compute the sign with threshold
  mcx::math::Twist cameraMoveVelocity(const mcx::math::Vector6D& inputVelocities, double deadband,
                                      mcx::math::Vector4D gain, double translationScale, double orientationScale,
                                      bool flip) const;

  void setLimitPositions(double lowerPositionLimit, double upperPositionLimit, double limitFactor,
                         double positionLimitOffset, unsigned int index);

  void convertLocalToGlobalPoses(const CartPose6& poseLocal, const mcx::math::Position& positionOffset,
                                 const mcx::math::Quaternion& rotationOffset, CartPose6& poseGlobal);

  void convertGlobalToLocalPoses(const CartPose6& poseGlobal, const mcx::math::Position& positionOffset,
                                 const mcx::math::Quaternion& rotationOffset, CartPose6& poseLocal);

  void convertGlobalAndLocalPoses();

  void calculateGlobalCameraView();

private:
  // From/To other tasks

  const std::map<logic::meril::ManipulatorAdmittanceJoints, std::array<bool, 6>> manipulatorAdmittanceJointsMap_ = {
    {logic::meril::ManipulatorAdmittanceJoints::NO_FULCRUM, {true, true, true, true, true, true}},
    {logic::meril::ManipulatorAdmittanceJoints::FULCRUM, {true, true, true, false, false, true}}};

  struct {
    mcx::control3::StateToControl in;
  } state_{};

  struct {
    mcx::control3::ModeToControl in;
    mcx::control3::ControlToMode out;
  } mode_{};

  // TODO: want a struct to communicate all IO to logic
  struct {
    logic::meril::MerilModeToControl in;
    logic::meril::ControlToMerilMode out;
  } merilMode_{};

  struct {
    mcx::motion::InterpreterToControl in{};
    mcx::motion::ControlToInterpreter out{};
    mcx::motion::InterpreterToControlHandles inHandles;
  } interpreter_{};

  // mechanism
  std::unique_ptr<mcx::mechanics::MechanismModule> mechanismModule_;
  struct {
    CartPose6 manipulatorBasePose{};
    CartPose6 manipulatorToolPose{};
    CartPose6 instrumentToolPose{};
    CartPose6 fulcrumPose{};
    CartPose6 fulcrumPortPose{};
    CartPose6 cameraPose{};
    CartPose6 handControllerPose{};
    CartPose6 screenPose{};
    mcx::math::Vector3D baseOrientationOffset{};
  } globalPoses_{};

  feature::Manipulator features_;

  std::vector<mcx::drive::DriveMode> driveMode_{};

  // system mode to distinguish simulation mode
  bool isInSimulation_ = false;

  // Manual Joint Mode
  // Host Joint Velocity
  JointVelocities hostInJointVelocity_; // jog velocity
  mcx::parameter_server::ParamHandle hostInJointVelocityHandle_;
  mcx::control3::Watchdog<JointVelocities> jointVelocityWatchdog_;
  double hostInJointVelocityGain_{1.0};
  JointPositions jointManualPositionsTarget_;

  // axes limiter settings
  // bool enableStiffAxesLimiter_{false};

  // Joint Reference Generator
  mcx::control3::SignalGenerators jointReferenceGenerator_;

  mcx::control3::Lookup fulcrumPositionErrorLookup_;
  lookUpTables jointLimiterGainLookup_; // insertion depth function of cart gain for Ratelimiter
  double fulcrumErrorScalingFactor_;
  double jointLimitsScalingFactor_;

  // Manual Cart Mode
  // Host Tool Velocity
  Twist hostInToolVelocity_;
  Twist joystickInAdditiveToolVelocity_;
  mcx::parameter_server::ParamHandle hostInToolVelocityHandle_;
  mcx::control3::Watchdog<Twist> toolVelocityWatchdog_;
  mcx::control3::SignalGenerators cartTransReferenceGenerator_{3};
  JointPositions jointAutoPositionsTarget_;
  double hostInInstrumentPinch_{};
  double instrumentPinchTarget_{};
  double instrumentPinchJog_{};
  double instrumentPinchStartTrackingTolerance_{1e-2};
  bool instrumentPinchEnable_{false};
  bool instrumentPinchIsTracking_{false};

  mcx::control3::Lookup cartTransGainLookup_; // insertion depth function of cart gain for Ratelimiter

  // Host Tool Trajectory
  HostInModule<HostInPosition> hostInToolPosition_{3};
  HostInModule<HostInOrientation> hostInToolOrientation_;

  // Semiauto Mode
  mcx::control3::SimpleSwitch semiAutoModeSwitch_;
  mcx::parameter_server::ParamHandle activateSemiAutoHandle_;
  mcx::control3::Watchdog<bool> activateSemiAutoWatchdog_;
  SemiautoGenerator semiAutoMotionGenerator_;
  mcx::control3::SimpleSwitch semiAutoTimeScaleSwitch_;
  bool activateSemiAuto_{};

  mcx::control3::SimpleSwitch jointModeSwitch_;  // Joint Mode Switch
  mcx::control3::SimpleSwitch manualModeSwitch_; // Manual Mode Switch
  JointPositions manualModeSwitchPositions_;

  mcx::control3::SimpleSwitch pauseModeSwitch_;  // Pause Mode Switch
  mcx::control3::LowPass2 jointPVATargetFilter_;

  // Actuator Control Loops
  JointPVAIn jointPvaActual_;
  JointPVAIn jointPvaLimitedReference_;
  JointTorques jointTorquesActual_;
  std::valarray<bool> jointIsOpenLoop_;

  // InverseDynamics
  JointTorqueOut idJointTorqueActual_;
  JointTorqueOut idJointTorqueReference_;
  JointTorques idJointVirtualMassActual_;

  // Compliance
  ComplianceControl jointComplianceControllers_;
  JointPVAIn jointCompliancePvaInput_;
  // unsigned int jointComplianceSetting_{ComplianceSetting::J123_ADMITTANCE};
  bool enableJointComplianceMode_{};
  mcx::control3::SimpleSwitch jointComplianceModeSwitch_;

  // Forward Kinematics
  JointPVAOut jointPvaTarget_;
  CartPose6 manipulatorToolPoseActual_;
  CartPose6 instrumentToolPoseActual_;
  CartPose6 manipulatorToolPoseTarget_;
  CartPose6 instrumentToolPoseTarget_;
  CartPose6 instrumentToolPoseConstrainedTarget_;
  CartPose6 manipulatorToolPoseLimitedReference_;
  CartPose6 instrumentToolPoseLimitedReference_;

  // instrument retract
  CartPose6 manipulatorRetractPose_{};
  double storedInstrumentLink3LengthForRetract_{};

  // Manipulability
  mcx::control3::Lookup manipulabilityGainLookup_;
  mcx::control3::WindowDetector manipulabilityDetector_;
  double manipulabilityActual_{};

  // Fulcrum/instrument kinematics
  InstrumentControl instrument_;
  double instrumentCalibrationDepth_{INSTRUMENT_CALIBRATION_POSITION};

  unsigned int numberOfJoints_{};
  unsigned int numberOfManipulatorJoints_{};

  struct {
    bool valid{false};

    struct {
      bool active{};
      double timerSec{};
      double timeoutSec{0.1};
    } watchdog;

    bool reset{false};
  } fulcrum_;

  mcx::control3::LowPass2 instrumentPinchTargetFilter_;

  // instrument calibration
  unsigned int instrumentCalibrationChannel_{};
  std::vector<unsigned int> instrumentCalibrationSequence_{SP_INSTRUMENT_STRAIGHTEN, 0, SP_INSTRUMENT_STRAIGHTEN, 1,
                                                           SP_INSTRUMENT_STRAIGHTEN, 2, SP_INSTRUMENT_STRAIGHTEN, 3,
                                                           SP_INSTRUMENT_STRAIGHTEN, 4, SP_INSTRUMENT_STRAIGHTEN, 5,
                                                           SP_INSTRUMENT_STRAIGHTEN, 6, SP_INSTRUMENT_STRAIGHTEN};
  std::vector<unsigned int> adapterCalibrationSequence_{SP_INSTRUMENT_STRAIGHTEN, SP_ADAPTER_CALIBRATE_NEGATIVE,
                                                        SP_INSTRUMENT_STRAIGHTEN, SP_ADAPTER_CALIBRATE_POSITIVE,
                                                        SP_INSTRUMENT_STRAIGHTEN};
  // instrument linear move / retract
  bool gotoManipulatorManualPrevious_{false};
  bool enableManipulatorManualLinearPrevious_{false};

  CartPose6 manipulatorManualLinearMovePoseReference_;

  // instrument exchange enabled flag
  double jointAutoStraightenInsertionDepth_ = MINIMUM_AUTO_STRAIGHTEN_INSERTION_DEPTH;
  bool enableStraightenInstrumentRollDuringRetract_ = false;

  JointPositions instrumentExchangeInitialJointPositions_{};

  mcx::math::Quaternion rotController2Inertial_{1.0, 0.0, 0.0, 0.0};
  double poseSyncReferencePsi_{};
  double poseSyncReferenceTheta_{};
  double poseSyncReferencePhi_{};
  bool poseSyncReferenceChanged_{false};

  bool manipulatorRollStraighteningToCameraView_{false};
  double manipulatorToCameraViewRotationAngle_{};
  double manipulatorRollStraighteningPosition_{}; // directly add offset to cameraView angle calculation
  double instrumentToCameraRzOffset_{};
  bool syncInstrumentRotationToCamera_{false};

  // cart pose rate limiter
  CartPoseRateLimiter cartPoseRateLimiter_;

  // Joint symbolic positions
  std::vector<std::string> symbolicPositionNames_ = {"idle", "moveToStartHigh", "moveToStartLow", "moveToPark",
                                                     "moveToExchange"};
  SymbolicPositionModule jointSymbolicPositions_{1, {}};
  bool gotoSymbolicMovePrev_{}; // to set timescale factor

  std::vector<std::string> instrumentPositionNames_ = {
      "moveToPitchPositive",  "moveToPitchNegative",      "moveToYawPositive",       "moveToYawNegative",
      "moveToPinchOpen",      "moveToRollPositive",       "moveToRollNegative",      "moveToStraighten",
      "moveToStraightenRoll", "calibrateAdapterNegative", "calibrateAdapterPositive"};
  SymbolicPositionModule instrumentSymbolicPositions_{1, {}};
  double storedJointManipulatorSyncPosition_;
  bool straightenDone_;

  // joint limits are set according to the symbolic (default) position the robot is placed in.
  std::map<std::string, PositionLimitSettings> jointPositionLimitSettings_{
      {"active", {}}, {"moveToStartHigh", {}}, {"moveToStartLow", {}}, {"moveToPark", {}}};
  double jointPositionLimitGain_{0.1};
  double jointPositionLimitOffset_{0.25}; // 15 degrees

  // force feedback
  mcx::math::Vector6D forceFeedbackWrenchControllerFrame_{};

  // joint limit force
  mcx::math::Vector6D manipulatorJointLimiterWrenchControllerFrame_{};
  std::unique_ptr<JointPositionLimiterForce<NR_JOINT_FORCE_FEEDBACK>> jointPositionLimiterForce_;

  // cartesian constraint force
  mcx::math::Vector6D instrumentCartConstraintWrenchControllerFrame_{};

  bool handcontrollerEngageButton_{};
  double handControllerEngageRateLimit_{0.01};
  double handControllerEngageRateLimitOmega_{M_PI * 40};
  double handcontrollerFadeIn_{};

  mcx::math::Quaternion rotManipulatorSyncReference_{};

  // camera dual handcontroller element:
  struct {
    bool enable{false};
    bool reverseDirection{false};
    mcx::math::Vector6D velocityInput{};
    mcx::math::Vector4D velocityGain{}; // (1.0, 1.0, 1.0, 1.0);
    double translationScale{0.5};
    double orientationScale{1.0};
    double velocityDeadband{DEFAULT_CAMERA_HANDCONTROLLER_VELOCITY_DEADBAND};
    mcx::math::Twist velocityOutput{};
  } cameraControl_;

  // reference coordinates poses
  CartPose6 cameraPose_{};
  CartPose6 cameraPoseViewOffset_{};
  CartPose6 cameraPoseView_{};

  // Camera synchronization
  bool poseSyncInSurgicalEnable_{false};
  double poseSyncAngleThreshold_{1e-3};

  observerKalmanFilters jointKalmanObservers_;
  bool enableJointStateEstimationForAdmittance_;
  JointPVAIn jointPVTEstimate_;

  double pinchControlTargetForce_{-0.8};
  mcx::control3::PID pinchController_{};

  // planning mode
  struct {
    bool enable{false};
    bool isLimiting{false};
    bool setFulcrum{false};
    CartPose6 instrumentToolPoseTarget;
    CartPose6 instrumentToolPoseActual;
    CartPose6 manipulatorToolPose;
    CartPose6 fulcrumPose;
    JointPositions jointPositionsTarget;
    JointPositions jointPositionsRawTarget;
  } planningMode_;
};
} // namespace control

#endif /* CTRL_MANIPULATORCONTROL_H */
