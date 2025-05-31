/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_motion.h"
#include "constants/const_errorcodes.h"
#include "lgc_robot_modes_transitions.h"
#include "lgc_robot_states.h"
#include "lgc_robot_states_data.h"
#include "lgc_robot_states_es.h"
#include "lgc_robot_states_fd.h"
#include "meril/lgc_meril_modes_adapter_calibration.h"
#include "meril/lgc_meril_modes_instrument_calibration.h"
#include "meril/lgc_meril_modes_instrument_exchange.h"
#include "meril/lgc_meril_modes_instrument_straighten.h"
#include "meril/lgc_meril_modes_locked.h"
#include "meril/lgc_meril_modes_reset_fulcrum.h"
#include "meril/lgc_meril_modes_sleep.h"
#include "meril/lgc_meril_modes_surgical.h"
#include "meril/lgc_meril_modes_symbolic_move.h"
#include "meril/lgc_meril_modes_unlocked.h"

namespace logic {

MotionLogic::MotionLogic(std::string saveConfigFilePath, const bool instrumentEnabled, const unsigned int numJoints,
                         const mcx::utils::SystemMode systemMode)
    : configFilePath_(std::move(saveConfigFilePath)), instrumentEnabled_(instrumentEnabled), numJoints_(numJoints),
      errors_(numJoints) {

  merilFsmData_.instrument.rfid.in.instrumentConnectionStatus = meril::NOT_CONNECTED;
  stateFsmData_.bus.in.ecat_bus_error = false;
  for (bool& cnt : stateFsmData_.bus.in.ecat_domain_error) {
    cnt = false;
  }
  stateFsmData_.bus.in.estop_not_active = true;
  stateFsmData_.bus.out.run_safety_plc = true;
  runMode_ = systemMode;
}

void MotionLogic::create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) {

  // Initializing mode array
  driveModeOut_ = std::make_unique<mcx::drive::DriveMode[]>(numJoints_);
  drivesWithTorqueMode_ = std::make_unique<bool[]>(numJoints_);
  drivesWithTorqueModeInUnlockedInstrument_ = std::make_unique<bool[]>(numJoints_);
  for (unsigned int i = 0; i < numJoints_; i++) {
    driveModeOut_[i] = mcx::drive::DriveMode::DRIVE_MODE_CYCLIC_SYNC_POSITION;
    drivesWithTorqueMode_[i] = false;
    drivesWithTorqueModeInUnlockedInstrument_[i] = false;
  }

  // Initializing meril modes machine
  merilModesFsm_.setName("Meril Mode");
  merilModesFsm_.createState(meril::SleepMode(merilFsmData_));
  merilModesFsm_.createState(meril::LockedMode(merilFsmData_));
  merilModesFsm_.createState(meril::UnlockedMode(merilFsmData_));
  merilModesFsm_.createState(meril::TransitionToLockedMode(merilFsmData_));
  merilModesFsm_.createState(meril::UnlockedInstrumentMode(merilFsmData_));
  merilModesFsm_.createState(meril::SurgicalMode(merilFsmData_));
  merilModesFsm_.createState(meril::ResetFulcrum(merilFsmData_));
  merilModesFsm_.createState(meril::SymbolicMoveMode(merilFsmData_));

  merilModesFsm_.createState(meril::InstrumentExchangeMode(merilFsmData_));
  merilModesFsm_.createState(meril::StraightenInstrumentBeforeStore(merilFsmData_));
  merilModesFsm_.createState(meril::RetractInstrument(merilFsmData_));
  merilModesFsm_.createState(meril::WaitingForInstrumentExchange(merilFsmData_));
  merilModesFsm_.createState(meril::InstrumentInsertTillFulcrum(merilFsmData_));
  merilModesFsm_.createState(meril::InstrumentInsertAndTeachFulcrum(merilFsmData_));
  merilModesFsm_.createState(meril::InstrumentInsert(merilFsmData_));

  merilModesFsm_.createState(meril::InstrumentStraightenMode(merilFsmData_));
  merilModesFsm_.createState(meril::ToSleepModeTransition(merilFsmData_));
  merilModesFsm_.createState(meril::InstrumentCalibrateMode(merilFsmData_));
  merilModesFsm_.createState(meril::AdapterCalibrateMode(merilFsmData_));
  merilModesFsm_.createState(meril::ResetFulcrumUnlockedMode(merilFsmData_));

  // Initializing state machines
  motionModesFsm_.setName("Motion Mode");
  motionModesFsm_.createState(InitMode(modeFsmData_));
  motionModesFsm_.createState(PauseMode(modeFsmData_));
  motionModesFsm_.createState(PauseToManualJointModeTransition(modeFsmData_));
  motionModesFsm_.createState(PauseToManualCartModeTransition(modeFsmData_));
  motionModesFsm_.createState(PauseToSemiAutoModeTransition(modeFsmData_));
  motionModesFsm_.createState(PauseToTorqueModeTransition(modeFsmData_));
  motionModesFsm_.createState(ToPauseModeTransition(modeFsmData_));
  motionModesFsm_.createState(ManualCartMode(modeFsmData_));
  motionModesFsm_.createState(ManualJointMode(modeFsmData_));
  motionModesFsm_.createState(SemiAutoMode(modeFsmData_));
  motionModesFsm_.createState(TorqueMode(modeFsmData_));

  robotStatesFsm_.setName("Robot State");
  robotStatesFsm_.createState(InitState(stateFsmData_));
  robotStatesFsm_.createState(ReferencingState(stateFsmData_));
  robotStatesFsm_.createState(SaveConfigurationState(stateFsmData_));
  robotStatesFsm_.createState(OffState(stateFsmData_));
  robotStatesFsm_.createState(OffToIdleState(stateFsmData_));
  robotStatesFsm_.createState(IdleToOffState(stateFsmData_));
  robotStatesFsm_.createState(IdleState(stateFsmData_));
  robotStatesFsm_.createState(IdleToEngagedState(stateFsmData_));
  robotStatesFsm_.createState(PausedToIdleState(stateFsmData_));
  robotStatesFsm_.createState(EngagedToPausedState(stateFsmData_));
  robotStatesFsm_.createState(EngagedState(stateFsmData_));

  robotStatesFsm_.createState(EStopResetState(stateFsmData_));
  robotStatesFsm_.createState(EStopOffState(stateFsmData_));
  robotStatesFsm_.createState(EStopOpenCircState(stateFsmData_));

  robotStatesFsm_.createState(ToForcedIdleState(stateFsmData_));
  robotStatesFsm_.createState(ForcedIdleState(stateFsmData_));
  robotStatesFsm_.createState(ForcedIdleResetState(stateFsmData_));
}

bool MotionLogic::initPhase1_() {

  using ParamType = mcx::parameter_server::ParameterType;

  /// Parameters
  addParameter("setNoEstopSec", ParamType::PARAMETER, &stateFsmData_.timings.setNoEstopSec);
  addParameter("resetEstopSec", ParamType::PARAMETER, &stateFsmData_.timings.resetEstopSec);
  addParameter("ecatRecoverSec", ParamType::PARAMETER, &stateFsmData_.timings.ecatRecoverTimeoutSec);
  addParameter("resetSlavesSec", ParamType::PARAMETER, &stateFsmData_.timings.resetSlavesSec);
  addParameter("gotoIdleDelaySec", ParamType::PARAMETER, &stateFsmData_.timings.gotoIdleDelaySec);
  addParameter("gotoEngageDelaySec", ParamType::PARAMETER, &stateFsmData_.timings.gotoEngageDelaySec);
  addParameter("stateTransitionTimeout", ParamType::PARAMETER, &stateTransitionTimeout_);
  addParameter("modeTransitionTimeout", ParamType::PARAMETER, &modeTransitionTimeout_);
  addParameter("drivesWithTorqueMode", ParamType::PARAMETER, drivesWithTorqueMode_.get(), numJoints_);
  addParameter("drivesWithTorqueModeInUnlockedInstrument", ParamType::PARAMETER,
               drivesWithTorqueModeInUnlockedInstrument_.get(), numJoints_);

  // errors & window detectors

  addParameter("guiFaultAcknowledge", mcx::parameter_server::ParameterType::INPUT, &guiFaultAcknowledge_);

  addParameter("driveErrorCode", ParamType::INPUT, errors_.slaveErrorCode.get(), numJoints_);
  addParameter("driveI2tActive", ParamType::INPUT, errors_.driveI2tActive.get(), numJoints_);
  addParameter("driveHasNoError", ParamType::INPUT, errors_.slaveHasNoError.get(), numJoints_);

  addParameter("rtViolationWindowDetector", ParamType::INPUT, &errors_.rtErrDetector);
  addParameter("actualManipulabilityDetector", ParamType::INPUT, &errors_.manipulabilityErrDetector);

  addParameter("xDetector", ParamType::INPUT, &errors_.xErrDetector);
  addParameter("yDetector", ParamType::INPUT, &errors_.yErrDetector);
  addParameter("zDetector", ParamType::INPUT, &errors_.zErrDetector);
  addParameter("actuatorLimitersActive", ParamType::INPUT, errors_.actuatorLimitersActive.get(), numJoints_);
  addParameter("positionErrorDetectors", ParamType::INPUT, errors_.posErrDetector.get(), numJoints_);
  addParameter("jumpDetectors", ParamType::INPUT, errors_.jumpDetector.get(), numJoints_);
  addParameter("positionDetectors", ParamType::INPUT, errors_.posDetector.get(), numJoints_);
  addParameter("velocityDetectors", ParamType::INPUT, errors_.velDetector.get(), numJoints_);
  addParameter("velocityDetectorsNotActive", ParamType::INPUT, &merilFsmData_.ctrl.velocityDetectorsNotActive);
  addParameter("accelerationDetectors", ParamType::INPUT, errors_.accDetector.get(), numJoints_);
  addParameter("torqueDetectors", ParamType::INPUT, errors_.torqueDetector.get(), numJoints_);
  addParameter("torqueSensorDetectors", ParamType::INPUT, errors_.torqueSensorDetector.get(), numJoints_);
  addParameter("collisionDetectors", ParamType::INPUT, errors_.collisionDetector.get(), numJoints_);
  addParameter("outputLimiterIsActive", ParamType::INPUT, errors_.outputLimiterIsActive.get(), numJoints_);

  addParameter("gotoSurgicalButton", ParamType::INPUT, &gotoSurgicalButton_);
  addParameter("gotoSurgicalPedal", ParamType::INPUT, &gotoSurgicalPedal_);
  addParameter("gotoSurgicalGuiButton", ParamType::INPUT, &gotoSurgicalGuiButton_);
  addParameter("gotoSurgicalMode", ParamType::OUTPUT, &gotoSurgicalMode_);
  addParameter("gotoTeachButton", ParamType::INPUT, &gotoTeachButton_);
  addParameter("gotoTeachGuiButton", ParamType::INPUT, &gotoTeachGuiButton_);
  addParameter("gotoUnlockButton", ParamType::INPUT, &gotoUnlockButton_);
  addParameter("gotoUnlockGuiButton", ParamType::INPUT, &gotoUnlockGuiButton_);
  addParameter("gotoInstrumentExchangeButton", ParamType::INPUT, &gotoInstrumentExchangeButton_);
  addParameter("gotoInstrumentExchangeGuiButton", ParamType::INPUT, &gotoInstrumentExchangeGuiButton_);
  addParameter("gotoInstrumentStraightenButton", ParamType::INPUT, &gotoInstrumentStraightenButton_);
  addParameter("gotoInstrumentStraightenGuiButton", ParamType::INPUT, &gotoInstrumentStraightenGuiButton_);
  addParameter("gotoInstrumentStraightenUdp", ParamType::INPUT, &gotoInstrumentStraightenUdp_);
  addParameter("gotoSymbolicPositionsButton", ParamType::INPUT, &gotoSymbolicPositionsButton_);
  addParameter("gotoSymbolicPositionsGuiButton", ParamType::INPUT, &gotoSymbolicPositionsGuiButton_);
  addParameter("gotoLockedDirectGuiButton", ParamType::INPUT, &gotoLockedDirectGuiButton_);
  addParameter("gotoInstrumentCalibrateGuiButton", ParamType::INPUT, &gotoInstrumentCalibrateGuiButton_);
  addParameter("gotoCameraReverseGuiButton", ParamType::INPUT, &gotoCameraReverseGuiButton_);

  addParameter("instrumentControl/gotoManualConnectGuiButton", ParamType::INPUT,
               &merilFsmData_.instrument.control.gotoManualConnectGuiButton);
  addParameter("instrumentControl/manualInstrumentType", ParamType::INPUT, &merilFsmData_.instrument.control.type);
  addParameter("instrumentControl/sterileAdapterConnect", ParamType::OUTPUT,
               &merilFsmData_.instrument.control.sterileAdapterConnect);
  addParameter("instrumentControl/sterileAdapterIsConnected", ParamType::OUTPUT,
               &merilFsmData_.instrument.control.sterileAdapterIsConnected);
  addParameter("instrumentControl/enableInstrumentTactileSwitch", ParamType::PARAMETER,
               &merilFsmData_.instrument.control.enableInstrumentTactileSwitch);
  addParameter("instrumentControl/enableSterileAdapterTactileSwitch", ParamType::PARAMETER,
               &merilFsmData_.instrument.control.enableSterileAdapterTactileSwitch);
  addParameter("instrumentControl/enableRFIDConnect", ParamType::PARAMETER,
               &merilFsmData_.instrument.control.enableRFIDConnect);
  addParameter("instrumentControl/timing/commandCycleSec", ParamType::PARAMETER,
               &merilFsmData_.instrument.rfid.state.commandCycleSec);
  addParameter("instrumentControl/timing/commandRetrySec", ParamType::PARAMETER,
               &merilFsmData_.instrument.rfid.state.commandRetrySec);

  addParameter("instrumentExchange/enableStopAtOutsideFulcrum", ParamType::PARAMETER,
               &merilFsmData_.ctrl.instrumentExchange.enableExchangeStopAtOutsideFulcrum);

  addParameter("cartIsDocked", ParamType::INPUT, &merilFsmData_.ctrl.cartIsDocked);
  addParameter("zActuatorIsDisabled", ParamType::INPUT, &merilFsmData_.ctrl.zActuator.isDisabled);
  addParameter("zActuatorDisable", ParamType::OUTPUT, &merilFsmData_.ctrl.zActuator.disable);
  addParameter("fulcrumWatchdogActive", ParamType::INPUT, &merilFsmData_.ctrl.fulcrumWatchdog.active);

  /// PARAMETERS
  addParameter("settings/enableMaintenanceMode", ParamType::INPUT, &merilFsmData_.ctrl.settings.enableMaintenanceMode);
  addParameter("settings/enableAutoStraighten", ParamType::PARAMETER, &enableAutoStraighten_);
  addParameter("settings/enableStraightenRollOutsideFulcrum", ParamType::PARAMETER,
               &merilFsmData_.ctrl.out.enableStraightenRollOutsideFulcrum);
  addParameter("settings/handguiding/enableSingleClickMode", ParamType::PARAMETER,
               &merilFsmData_.ctrl.settings.enableHandguidingSingleClickMode);
  addParameter("settings/handguiding/transitionTimeoutSec", ParamType::PARAMETER, &merilFsmData_.transitionTimeoutSec);

  // addParameter("settings/enableImpedanceInUnlockedInstrumentMode", ParamType::PARAMETER,
  //              &merilFsmData_.ctrl.settings.enableImpedanceInUnlockedInstrumentMode);

  addParameter(":timeOut/instrumentStraighteningTimeoutSec", ParamType::PARAMETER,
               &merilFsmData_.instrumentStraighteningTimeoutSec);
  addParameter(":timeOut/instrumentCalibrationTimeoutSec", ParamType::PARAMETER,
               &merilFsmData_.instrumentCalibrationTimeoutSec);

  // addParameter("enableAdmittanceUnlock", ParamType::PARAMETER, &enableAdmittanceUnlock_);

  addParameter("merilMode", ParamType::OUTPUT, &merilFsmData_.currentFsmModeOut);
  addParameter("activeInstrumentId", ParamType::OUTPUT, &merilFsmData_.instrument.activeInstrumentId);
  addParameter("exchangeInstrumentAllowed", ParamType::OUTPUT, &merilFsmData_.instrument.exchangeInstrumentAllowed);

  addParameter("torqueStateLeaveConditions/positionError", ParamType::OUTPUT,
               &modeFsmData_.torqueStateLeaveConditions.positionError);
  addParameter("torqueStateLeaveConditions/pvaLimitActive", ParamType::OUTPUT,
               &modeFsmData_.torqueStateLeaveConditions.pvaLimitActive);

  // adding active errors
  auto& robotStatesErr = robotStatesFsm_.getErrorHandle();
  robotStatesErr.addParameters("activeErrors", getLocalBranch());

  // incoming events mode
  externalModeEventInHandle_ = addParameter("modeCommand", ParamType::INPUT, &modeFsmData_.externalModeEventIn);

  // incoming events state
  stateEventHandle_ = addParameter("stateCommand", ParamType::INPUT, &stateFsmData_.actualEvent);

  /// Outputs
  addParameter("runMode", ParamType::OUTPUT, &runMode_);
  addParameter("state", ParamType::OUTPUT, &stateFsmData_.actualState);
  addParameter("mode", ParamType::OUTPUT, &modeFsmData_.currentFsmModeOut);

  addParameter("driveMode", ParamType::OUTPUT, driveModeOut_.get(), numJoints_);
  addParameter("driveCommand", ParamType::OUTPUT, &driveCommandOut_);

  addParameter("busHasError", ParamType::OUTPUT, &stateFsmData_.bus.busHasError);

  // persistence output (handle)
  stateFsmData_.persistenceCommandOutHandle =
      addParameter("persistenceCommand", ParamType::OUTPUT, &stateFsmData_.persistenceCommandOut);

  addParameter("referencing/:stateToCtrl", ParamType::OUTPUT, &stateFsmData_.referencing.out);

  // Folder links
  addParameter(":ctrlToMerilMode", ParamType::INPUT, &merilFsmData_.ctrl.in);
  addParameter(":merilModeToCtrl", ParamType::OUTPUT, &merilFsmData_.ctrl.out);

  addParameter(":rfidToMerilMode", ParamType::INPUT, &merilFsmData_.instrument.rfid.in);
  addParameter(":merilModeToRfid", ParamType::OUTPUT, &merilFsmData_.instrument.rfid.out);

  addParameter(":instrumentAxesCtrlToState", ParamType::INPUT, &instrumentAxesCtrlToState_);
  addParameter(":manipulatorAxesCtrlToState", ParamType::INPUT, &manipulatorAxesCtrlToState_);

  addParameter(":busToState", ParamType::INPUT, &stateFsmData_.bus.in);

  addParameter(":ctrlToMode", ParamType::INPUT, &modeFsmData_.ctrl.in);
  auto interpToModeGroupHandle = addParameter(":interpToMode", ParamType::INPUT, &modeFsmData_.interpreter.in);

  addParameter(":stateToCtrl", ParamType::OUTPUT, &stateFsmData_.ctrl.out);
  addParameter(":stateToBus", ParamType::OUTPUT, &stateFsmData_.bus.out);

  auto modeToCtrlGroupHandle = addParameter(":modeToCtrl", ParamType::OUTPUT, &modeFsmData_.ctrl.out);
  // handles to control (from mode)
  modeFsmData_.ctrl.outHandles.resetCollisionDetectors = modeToCtrlGroupHandle.getHandle("resetCollisionDetectors");
  modeFsmData_.ctrl.outHandles.resetCollisionDetectors.updateOutput(false);

  return true;
}

bool MotionLogic::initPhase2_() {

  auto& robotStateErr = robotStatesFsm_.getErrorHandle();
  robotStateErr.addEmergencyStop(ES_ESTOP_INPUT_OPENED, &stateFsmData_.bus.in.estop_not_active, false);

  robotStateErr.addEmergencyStop({ES_CIRCUIT_BREAKER_TRIPPED, 1}, &stateFsmData_.bus.in.circuitbreaker_us_tripped,
                                 true);

  robotStateErr.addEmergencyStop({ES_CIRCUIT_BREAKER_TRIPPED, 2}, &stateFsmData_.bus.in.circuitbreaker_up_tripped,
                                 true);

  robotStateErr.addEmergencyStop(ES_BUS_ERROR, &stateFsmData_.bus.in.ecat_bus_error, true);
  for (unsigned int cnt = 0; cnt < MAXDOMAINS; cnt++) {
    robotStateErr.addEmergencyStop({ES_BUS_ERROR, cnt + 1}, &stateFsmData_.bus.in.ecat_domain_error[cnt], true);
  }
  robotStateErr.disableErrors<InitState>(mcx::state_machine::Error(ES_ESTOP_INPUT_OPENED));
  robotStateErr.disableErrors<InitState>(mcx::state_machine::Error(ES_BUS_ERROR));

  robotStateErr.addWarning({WA_REALTIME_VIOLATION, 0}, &errors_.rtErrDetector.isHigh, true);

  robotStateErr.addWarning({WA_REALTIME_VIOLATION, 1}, &errors_.rtErrDetector.isTooHigh, true);

  robotStateErr.disableErrors<InitState, OffState, EStopOffState, EStopResetState, EStopOpenCircState>(
      mcx::state_machine::Error(WA_REALTIME_VIOLATION));

  mcx::control3::WindowDetectorStateData& xErr = errors_.xErrDetector;
  addWindowDetectorFd(WA_POSITION_WINDOW_EXCEEDED, FD_CART_POSITION_WINDOW_EXCEEDED, 101, &xErr);

  mcx::control3::WindowDetectorStateData& yErr = errors_.yErrDetector;
  addWindowDetectorFd(WA_POSITION_WINDOW_EXCEEDED, FD_CART_POSITION_WINDOW_EXCEEDED, 102, &yErr);

  mcx::control3::WindowDetectorStateData& zErr = errors_.zErrDetector;
  addWindowDetectorFd(WA_POSITION_WINDOW_EXCEEDED, FD_CART_POSITION_WINDOW_EXCEEDED, 103, &zErr);

  for (uint32_t i = 0; i < numJoints_; i++) {

    bool& outputLimiterIsActive = errors_.outputLimiterIsActive[i];
    robotStateErr.addWarning({WA_JOG_JOINT_JOYSTICK_OUTSIDE_LIMITS_ERROR, i + 1}, &outputLimiterIsActive, true);
    robotStateErr.disableErrors<InitState, OffState, ReferencingState, SaveConfigurationState, EStopOffState,
                                EStopResetState, EStopOpenCircState>(
        {WA_JOG_JOINT_JOYSTICK_OUTSIDE_LIMITS_ERROR, i + 1});

    // position limit reached
    auto& pvaErr = errors_.actuatorLimitersActive[i];
    robotStateErr.addWarning({WA_PVA_LIMITER_ACTIVE, i + 1}, &pvaErr, true);

    // Disable for init and error states
    robotStateErr.disableErrors<InitState, OffState, ReferencingState, SaveConfigurationState, EStopOffState,
                                EStopResetState, EStopOpenCircState>({WA_PVA_LIMITER_ACTIVE, i + 1});

    // position window detector
    mcx::control3::WindowDetectorStateData& posErr = errors_.posDetector[i];
    addWindowDetectorError(WA_POSITION_WINDOW_EXCEEDED, ES_POSITION_WINDOW_EXCEEDED, i + 1, &posErr);

    // a jump window detector
    mcx::control3::WindowDetectorStateData& jumpErr = errors_.jumpDetector[i];
    addWindowDetectorFd(WA_SETPOINT_JUMP, FD_SETPOINT_JUMP, i + 1, &jumpErr);

    // position tracking window detector
    mcx::control3::WindowDetectorStateData& trackingErr = errors_.posErrDetector[i];
    addWindowDetectorError(WA_TRACKING_ERROR_WINDOW_EXCEEDED, ES_TRACKING_ERROR_WINDOW_EXCEEDED, i + 1, &trackingErr);

    // force window detector
    mcx::control3::WindowDetectorStateData& forceErr = errors_.torqueDetector[i];
    addWindowDetectorError(WA_FORCE_WINDOW_EXCEEDED, ES_FORCE_WINDOW_EXCEEDED, i + 1, &forceErr);

    // velocity window detector
    // mcx::control3::WindowDetectorStateData& velocityErr = errors_.velDetector[i];
    // addWindowDetectorError(WA_VELOCIY_WINDOW_EXCEEDED, ES_VELOCIY_WINDOW_EXCEEDED, i + 1, &velocityErr);

    // acceleration window detector
    mcx::control3::WindowDetectorStateData& accelerationErr = errors_.accDetector[i];
    addWindowDetectorError(WA_ACCELERATION_WINDOW_EXCEEDED, ES_ACCELERATION_WINDOW_EXCEEDED, i + 1, &accelerationErr);

    // force sensor window detector
    mcx::control3::WindowDetectorStateData& forcesensorErr = errors_.torqueSensorDetector[i];
    addWindowDetectorError(WA_FORCE_WINDOW_EXCEEDED, ES_FORCE_WINDOW_EXCEEDED, i + 1, &forcesensorErr);

    // collision window detector
    mcx::control3::WindowDetectorStateData& collisionErr = errors_.collisionDetector[i];
    addWindowDetectorError(WA_COLLISION_DETECTED, ES_COLLISION_DETECTED, i + 1, &collisionErr);

    // collision detectors switched off
    // robotStateErr.addWarning({WA_COLLISION_DETECTION_DISABLED}, &collisionErr.isEnabled, false);

    // slave error
    robotStateErr.addEmergencyStop({ES_SLAVE_ERROR, i + 1, &errors_.slaveErrorCode[i]}, &errors_.slaveHasNoError[i],
                                   false);

    // i2t active
    robotStateErr.addEmergencyStop({ES_SLAVE_I2T_ACTIVE, i + 1}, &errors_.driveI2tActive[i], true);
    robotStateErr.disableErrors<InitState, OffState, ReferencingState, SaveConfigurationState, EStopOffState,
                                EStopResetState, EStopOpenCircState>({ES_SLAVE_I2T_ACTIVE, i + 1});
  }

  // close to singularity check
  robotStateErr.addWarning(WA_CLOSE_TO_SINGULARITY, &errors_.manipulabilityErrDetector.isLow, true);

  robotStateErr.addEmergencyStop(ES_TOO_CLOSE_TO_SINGULARITY, &errors_.manipulabilityErrDetector.isTooLow, true);

  // enable errors depending on the mode, check iterate() function
  robotStateErr.disableErrors(mcx::signal_monitor::SignalIdGroup{WA_CLOSE_TO_SINGULARITY, ES_TOO_CLOSE_TO_SINGULARITY});

  // Fulcrum lost

  robotStateErr.addForceDisengage(FD_INSTRUMENT_FULCRUM_INVALID_TIMEOUT, &merilFsmData_.ctrl.fulcrumWatchdog.active,
                                  true);

  bool cartIsNotDockWhileFulcrumValid = !merilFsmData_.ctrl.cartIsDocked && merilFsmData_.ctrl.in.fulcrumIsValid;
  bool zActuatorEnabledWhileFulcrumValid = !merilFsmData_.ctrl.zActuator.isDisabled && merilFsmData_.ctrl.in.fulcrumIsValid;

  robotStateErr.addWarning(WA_INSTRUMENT_INSERTION_DEPTH_LIMIT,
                           &merilFsmData_.ctrl.in.instrumentInsertionDepthIsLimiting, true);
  robotStateErr.addWarning(WA_INSTRUMENT_FULCRUM_INVALID, &merilFsmData_.ctrl.in.fulcrumIsValid, false);
  robotStateErr.addWarning(WA_SURGICAL_MODE_NOT_ALLOWED, &merilFsmData_.ctrl.in.surgicalModeIsAllowed, false);
  robotStateErr.addWarning(WA_INSTRUMENT_NOT_STRAIGHTENED, &merilFsmData_.ctrl.in.instrumentStraightenDone, false);
  robotStateErr.addWarning({WA_CART_UNDOCKED_WARNING, 0}, &cartIsNotDockWhileFulcrumValid, false);
  robotStateErr.addWarning({WA_Z_ACTUATOR_ACTIVE_WARNING, 0}, &zActuatorEnabledWhileFulcrumValid, false);
  robotStateErr.addWarning({WA_MAINTENANCE_MODE_ENABLED, 0}, &merilFsmData_.ctrl.settings.enableMaintenanceMode, true);
  robotStateErr.disableErrors(mcx::signal_monitor::SignalIdGroup{
      WA_INSTRUMENT_FULCRUM_INVALID, WA_SURGICAL_MODE_NOT_ALLOWED, WA_INSTRUMENT_NOT_STRAIGHTENED,
      WA_CART_UNDOCKED_WARNING, WA_Z_ACTUATOR_ACTIVE_WARNING});

  configureButtons();

  return true;
}

bool MotionLogic::startOp_() {
  stateFsmData_.actualEvent = StateEvents::DO_NOTHING_E;
  modeFsmData_.externalModeEventIn = ModeEvents::GOTO_INIT_E;
  modeFsmData_.currentFsmModeOut = Modes::INIT_M;
  merilModesFsm_.setActiveState<meril::SleepMode>();
  motionModesFsm_.setActiveState<InitMode>();
  robotStatesFsm_.setActiveState<InitState>();

  return true;
}

bool MotionLogic::stopOp_() { return true; }

ModeEvents MotionLogic::toFsmModeEvent(mcx::motion::ModeEvents modeEvent) {
  switch (modeEvent) {
  case mcx::motion::ModeEvents::GOTO_PAUSE_E:
    return ModeEvents::GOTO_PAUSE_E;
  case mcx::motion::ModeEvents::GOTO_AUTO_RUN_E:
    return ModeEvents::GOTO_AUTO_RUN_E;
  case mcx::motion::ModeEvents::GOTO_MOVE_TO_START_E:
    return ModeEvents::GOTO_MOVE_TO_START_E;
  default:
    return ModeEvents::GOTO_NONE_E;
  }
}

void MotionLogic::initTorqueConditions() {
  // filling data for leaving torque mode condition;
  modeFsmData_.torqueStateLeaveConditions.positionError = false;
  modeFsmData_.torqueStateLeaveConditions.pvaLimitActive = false;
  for (size_t i = 0; i < numJoints_; i++) {
    if (drivesWithTorqueMode_[i] || drivesWithTorqueModeInUnlockedInstrument_[i]) {
      modeFsmData_.torqueStateLeaveConditions.positionError |= !errors_.posErrDetector[i].noWarning;
      modeFsmData_.torqueStateLeaveConditions.pvaLimitActive |= errors_.actuatorLimitersActive[i];
    }
  }
}

void MotionLogic::activateSignalMonitor() {

  // if robot is switched on and is in AutoRunMode or ManualCartMode,
  // activate CLOSE_TO_SINGULARITY checks
  auto& robotStateErr = robotStatesFsm_.getErrorHandle();
  robotStateErr.enableErrors<EngagedState>(
      mcx::signal_monitor::SignalIdGroup{WA_MAINTENANCE_MODE_ENABLED, WA_CART_UNDOCKED_WARNING});

  const auto signalMonitorPtr = robotStateErr.getSignalMonitor();
  if (robotStatesFsm_.isStateActive<EngagedState>()) {
    if (merilFsmData_.ctrl.in.fulcrumIsValid) {
      signalMonitorPtr->enable(
          {mcx::signal_monitor::SignalIdGroup{WA_MAINTENANCE_MODE_ENABLED, WA_CART_UNDOCKED_WARNING}});
    } else {
      signalMonitorPtr->disable(
          {mcx::signal_monitor::SignalIdGroup{WA_MAINTENANCE_MODE_ENABLED, WA_CART_UNDOCKED_WARNING}});
    }
    if (motionModesFsm_.isStateActive<ManualCartMode>()) {
      signalMonitorPtr->enable({mcx::signal_monitor::SignalIdGroup{
          WA_CLOSE_TO_SINGULARITY, ES_TOO_CLOSE_TO_SINGULARITY, WA_INSTRUMENT_FULCRUM_INVALID,
          WA_SURGICAL_MODE_NOT_ALLOWED, FD_INSTRUMENT_FULCRUM_INVALID_TIMEOUT}});
    } else {
      signalMonitorPtr->disable({mcx::signal_monitor::SignalIdGroup{
          WA_CLOSE_TO_SINGULARITY, ES_TOO_CLOSE_TO_SINGULARITY, WA_INSTRUMENT_FULCRUM_INVALID,
          WA_SURGICAL_MODE_NOT_ALLOWED, FD_INSTRUMENT_FULCRUM_INVALID_TIMEOUT}});
    }

    if (merilModesFsm_.isStateActive<meril::InstrumentExchangeMode>() || gotoInstrumentRetract_) {
      signalMonitorPtr->enable({mcx::signal_monitor::SignalIdGroup{WA_INSTRUMENT_NOT_STRAIGHTENED}});
    } else {
      signalMonitorPtr->disable({mcx::signal_monitor::SignalIdGroup{WA_INSTRUMENT_NOT_STRAIGHTENED}});
    }
  }
}

void MotionLogic::syncFsm(bool isInSimulationMode) {
  // force meril to sleep mode
  if (robotStatesFsm_.isStateActive<EStopOffState>()) {
    merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoSleepEstopActive);
  }
  // force meril to locked mode
  if (robotStatesFsm_.isStateActive<ForcedIdleState>()) {
    merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoLockedForceIdleActive);
  }

  if (merilModesFsm_.isStateActive<meril::ToSleepModeTransition>()) {
    robotStatesFsm_.executeEvent({&SuperState::gotoOff, stateTransitionTimeout_.gotoOff});
  }

  // Modes where Manual Joint Mode is active
  if (merilModesFsm_.isStateActive<meril::UnlockedInstrumentMode, meril::AdapterCalibrateMode,
                                   meril::InstrumentCalibrateMode, meril::SymbolicMoveMode,
                                   meril::InstrumentStraightenMode, meril::RetractInstrument, meril::InstrumentInsert,
                                   meril::InstrumentInsertTillFulcrum, meril::InstrumentInsertAndTeachFulcrum>()) {
    motionModesFsm_.executeEvent({&SuperMode::gotoManualJointModeEvent, modeTransitionTimeout_.gotoManualJoin});
  }

  // When Unlocked Mode is active in simulation or with enable admittance flag, select Manual Joint Mode,
  // otherwise select Torque Mode
  if (merilModesFsm_.isStateActive<meril::UnlockedMode>()) {
    if (isInSimulationMode /*|| enableAdmittanceUnlock_*/) {
      motionModesFsm_.executeEvent({&SuperMode::gotoManualJointModeEvent, modeTransitionTimeout_.gotoManualJoin});
    } else {
      motionModesFsm_.executeEvent({&SuperMode::gotoTorqueModeEvent, stateFsmData_.engagedStateTimer,
                                    modeTransitionTimeout_.gotoTorqueInitialDelay, modeTransitionTimeout_.gotoTorque});
    }
  }

  // In Locked Mode switch to Pause
  if (merilModesFsm_.isStateActive<meril::LockedMode, meril::ResetFulcrum, meril::InstrumentExchangeMode,
                                   meril::WaitingForInstrumentExchange>()) {
    motionModesFsm_.executeEvent({&SuperMode::gotoPauseModeEvent, modeTransitionTimeout_.gotoPause});
  }

  // In Surgical Mode switch to Manual Cart Mode
  if (merilModesFsm_.isStateActive<meril::SurgicalMode>()) {
    motionModesFsm_.executeEvent({&SuperMode::gotoManualCartModeEvent, modeTransitionTimeout_.gotoManualCart});
  }
}

bool MotionLogic::iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) {

  stateFsmData_.ctrl.in = instrumentEnabled_
                              ? stateFsmData_.ctrl.performAnd(instrumentAxesCtrlToState_, manipulatorAxesCtrlToState_)
                              : manipulatorAxesCtrlToState_;

  merilFsmData_.ctrl.out.gotoTeachFulcrum = false;
  merilFsmData_.ctrl.out.gotoResetFulcrum = false;
  stateFsmData_.ctrl.out.syncControlLoop = false;

  // check simulation mode:
  const bool isInSimulationMode = runMode_ == mcx::cmd_line::SystemMode::SIMULATION;

  // checking if pause mode is active
  stateFsmData_.ctrl.in.isAtPause = modeFsmData_.ctrl.in.pauseModeSwitchOn;
  // checkin if bus or any domain has an error
  stateFsmData_.bus.busHasError = stateFsmData_.bus.in.ecat_bus_error;
  for (const auto domainError : stateFsmData_.bus.in.ecat_domain_error) {
    stateFsmData_.bus.busHasError |= domainError;
  }

  {
    // update drape connection status (sterile adapter)
    const bool systemIsNotStartUp =
        !merilModesFsm_.isStateActive<meril::SleepMode>() && !merilModesFsm_.isStateActive<InitState>();

    merilFsmData_.instrument.control.sterileAdapterConnect =
        merilFsmData_.instrument.control.enableSterileAdapterTactileSwitch &&
        merilFsmData_.instrument.rfid.in.sterileAdapterConnectionStatus == meril::InstrumentConnectStatus::CONNECTED &&
        systemIsNotStartUp;
  }

  // update instrument
  // TODO: Connect based on manual instrument ID-input as alternative to reading out RFID signals...
  if (!merilFsmData_.ctrl.in.isCameraRobot) {
    merilFsmData_.instrument.control.instrumentConnectionDetected =
        merilFsmData_.instrument.control.enableInstrumentTactileSwitch
            ? merilFsmData_.instrument.rfid.in.instrumentConnectionStatus == meril::InstrumentConnectStatus::CONNECTED
            : merilFsmData_.instrument.rfid.in.statusWord >=
                  meril::MerilModesFSMData::Instrument::RFID::Reader::INSTRUMENT_CONNECTION_STATUS;

    if (enableAutoStraighten_ && merilFsmData_.ctrl.in.instrumentAutoStraighten) {
      merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoInstrumentStraighten);
    }
  }

  merilButtons_.iterate(getDtSec());

  initTorqueConditions();
  activateSignalMonitor();

  // zActuator status connected to the fulcrum status. as soon as the fulcrum is taught the z-actuator shall be
  // disabled.
  merilFsmData_.ctrl.zActuator.disable = merilFsmData_.ctrl.in.fulcrumIsValid;

  // Maintenance (advanced) overwrite whenever the application is stuck (can't reset fulcrum pose due to robot envelope
  // limitations or teach new symbolic positions...) unlocked mode is allowed to manually move robot in any position.
  merilFsmData_.ctrl.out.gotoMaintenanceMode =
      (merilFsmData_.ctrl.settings.enableMaintenanceMode &&
       merilModesFsm_.isStateActive<meril::LockedMode, meril::UnlockedMode, meril::SymbolicMoveMode>());

  syncFsm(isInSimulationMode);

  // update event handles:
  if (externalModeEventInHandle_.isUpdated()) {
    switch (modeFsmData_.externalModeEventIn) {
    case ModeEvents::GOTO_INIT_E: {
      motionModesFsm_.executeEvent({&SuperMode::gotoInitEvent, modeTransitionTimeout_.gotoInit});
      break;
    }
    case ModeEvents::GOTO_PAUSE_E: {
      if (merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoLocked) == mcx::state_machine::EVENT_DONE) {
        log_info("gotoLocked event via the mode command");
      }
      break;
    }
    case ModeEvents::GOTO_MANUAL_CART_MODE_E: {
      // motionModesFsm_.executeEvent({&SuperMode::gotoManualCartModeEvent, modeTransitionTimeout_.gotoManualCart});

      if (!merilModesFsm_.isStateActive<meril::SurgicalMode>() &&
          merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoSurgicalMode) == mcx::state_machine::EVENT_DONE) {
        log_info("gotoSurgicalMode event via the mode command");
      }
      break;
    }
    case ModeEvents::GOTO_MANUAL_JOINT_MODE_E: {
      motionModesFsm_.executeEvent({&SuperMode::gotoManualJointModeEvent, modeTransitionTimeout_.gotoManualJoin});
      break;
    }
    case ModeEvents::GOTO_SEMI_AUTO_E: {
      motionModesFsm_.executeEvent({&SuperMode::gotoSemiAutoModeEvent, modeTransitionTimeout_.gotoSemiAuto});
      break;
    }
    case ModeEvents::GOTO_TORQUE_MODE_E: {
      motionModesFsm_.executeEvent({&SuperMode::gotoTorqueModeEvent, stateFsmData_.engagedStateTimer,
                                    modeTransitionTimeout_.gotoTorqueInitialDelay, modeTransitionTimeout_.gotoTorque});
      break;
    }
    default: {
      break;
    }
    }
  }

  // Gui acknowledge
  if (guiFaultAcknowledge_) {
    guiFaultAcknowledge_ = false;
    robotStatesFsm_.executeEvent(&SuperState::acknowledgeErrors);
  }

  if (stateEventHandle_.isUpdated()) {
    switch (stateFsmData_.actualEvent) {
    case StateEvents::GOTO_OFF_E: {
      if (merilModesFsm_.isStateActive<meril::SleepMode, meril::LockedMode, meril::SymbolicMoveMode>()) {
        robotStatesFsm_.executeEvent({&SuperState::gotoOff, stateTransitionTimeout_.gotoOff});
        merilModesFsm_.executeEvent({&meril::SuperMerilModes::gotoSleep});
      }
      break;
    }
    case StateEvents::GOTO_IDLE_E: {
      if (merilModesFsm_.isStateActive<meril::SleepMode, meril::LockedMode, meril::SymbolicMoveMode>()) {
        robotStatesFsm_.executeEvent({&SuperState::gotoIdle, stateTransitionTimeout_.gotoIdle});
      }
      break;
    }
    case StateEvents::GOTO_ENGAGED_E: {
      robotStatesFsm_.executeEvent({&SuperState::gotoEngaged, stateTransitionTimeout_.gotoEngage});
      break;
    }
    case StateEvents::GOTO_REFERENCING_E: {
      robotStatesFsm_.executeEvent({&SuperState::gotoReferencing, stateTransitionTimeout_.gotoReference});
      break;
    }
    case StateEvents::EMERGENCY_STOP_E: {
      robotStatesFsm_.emergencyStop(ES_ESTOP_INPUT_OPENED);
      break;
    }
    case StateEvents::ACKNOWLEDGE_ERROR: {
      robotStatesFsm_.executeEvent(&SuperState::acknowledgeErrors);
      break;
    }
    case StateEvents::SAVE_CONFIGURATION: {
      robotStatesFsm_.executeEvent({&SuperState::gotoSaveConfiguration, configFilePath_.c_str(), getParameterRoot()});
      break;
    }
    default: {
      break;
    }
    }
  }

  const double dtSec = getDtSec();
  merilFsmData_.ctrl.velocityDetectorsNotActive =
      std::all_of(errors_.velDetector.get(), errors_.velDetector.get() + numJoints_,
                  [](const auto& velDetector) { return velDetector.noError; });

  merilModesFsm_.iterate(dtSec);
  motionModesFsm_.iterate(dtSec);
  robotStatesFsm_.iterate(dtSec);

  // mapping modes fsm out to the drive mode
  merilFsmData_.ctrl.out.enableImpedanceInUnlockedInstrumentMode =
      merilFsmData_.ctrl.settings.enableImpedanceInUnlockedInstrumentMode;
  for (unsigned int i = 0; i < numJoints_; i++) {
    if ((modeFsmData_.currentFsmModeOut == Modes::TORQUE_M) && !isInSimulationMode && drivesWithTorqueMode_[i] &&
        (merilModesFsm_.isStateActive<meril::UnlockedMode>() ||
         merilModesFsm_.isStateActive<meril::TransitionToLockedMode>())) {
      driveModeOut_[i] = mcx::drive::DriveMode::DRIVE_MODE_CYCLIC_SYNC_TORQUE;
    } else if (merilFsmData_.ctrl.out.enableImpedanceInUnlockedInstrumentMode &&
               drivesWithTorqueModeInUnlockedInstrument_[i] &&
               merilModesFsm_.isStateActive<meril::UnlockedInstrumentMode>()) {
      driveModeOut_[i] = mcx::drive::DriveMode::DRIVE_MODE_CYCLIC_SYNC_TORQUE;
    } else {
      driveModeOut_[i] = mcx::drive::DriveMode::DRIVE_MODE_CYCLIC_SYNC_POSITION;
    }
  }

  // mapping states fsm out to the drive command
  switch (stateFsmData_.actualState) {
    // In the following states the drives are commanded off
  case States::INIT_S:
  case States::OFF_S:
  case States::IDLE_TO_OFF_T:
  case States::ESTOP_OFF_S:
    driveCommandOut_ = mcx::drive::DriveCommand::DRIVE_CMD_OFF;
    break;
    // Drives shall be on whenever we leave OFF towards idle and engaged
  case States::TO_FORCEDIDLE_T:
  case States::FORCEDIDLE_S:
  case States::RESET_FORCEDIDLE_T:
  case States::OFF_TO_IDLE_T:
  case States::IDLE_S:
  case States::IDLE_TO_ENGAGED_T: // Allow delay when releasing brakes on drives (see lgc_robot_states.cpp)
  case States::ENGAGED_TO_PAUSED_T:
  case States::PAUSED_TO_IDLE_T:
  case States::ENGAGED_S:
    driveCommandOut_ = mcx::drive::DriveCommand::DRIVE_CMD_ENGAGE;
    break;
  case States::RESET_ESTOP_T:
    driveCommandOut_ = mcx::drive::DriveCommand::DRIVE_CMD_FAULT_ACK;
    break;
  default:
    break;
  }

  colorManager();

  merilFsmData_.actualRobotState = stateFsmData_.actualState;
  stateFsmData_.ctrl.out.disablePVALimiter =
      merilFsmData_.ctrl.disablePVALimiter || stateFsmData_.ctrl.disablePVALimiter;

  // Force pause mode when not in one of the following modes/states
  const bool forcePauseMode1 = !motionModesFsm_.isStateActive<PauseMode>() &&
                               !motionModesFsm_.isStateActive<ToPauseModeTransition>() &&
                               !robotStatesFsm_.isStateActive<EngagedState>();

  if (forcePauseMode1) {
    motionModesFsm_.executeEvent({&SuperMode::gotoPauseModeEvent, modeTransitionTimeout_.gotoPause});
  }

  return true;
}

void MotionLogic::addWindowDetectorError(uint32_t warningCode, uint32_t errorCode, uint32_t subsystem,
                                         mcx::control3::WindowDetectorStateData* wd) {

  auto& stateErr = robotStatesFsm_.getErrorHandle();
  stateErr.addWarning({warningCode, subsystem}, &wd->isHigh, true);
  stateErr.addWarning({warningCode, subsystem}, &wd->isLow, true);

  stateErr.addEmergencyStop({errorCode, subsystem}, &wd->isTooHigh, true);
  stateErr.addEmergencyStop({errorCode, subsystem}, &wd->isTooLow, true);

  // configuring signal monitor
  stateErr.disableErrors<InitState, OffState, ReferencingState, SaveConfigurationState, EStopOffState, EStopResetState,
                         EStopOpenCircState>({{errorCode, subsystem}, {warningCode, subsystem}});
}

void MotionLogic::addWindowDetectorFd(uint32_t warningCode, uint32_t errorCode, uint32_t subsystem,
                                      mcx::control3::WindowDetectorStateData* wd) {

  auto& stateErr = robotStatesFsm_.getErrorHandle();
  stateErr.addWarning({warningCode, subsystem}, &wd->isHigh, true);
  stateErr.addWarning({warningCode, subsystem}, &wd->isLow, true);

  stateErr.addForceDisengage({errorCode, subsystem}, &wd->isTooHigh, true);
  stateErr.addForceDisengage({errorCode, subsystem}, &wd->isTooLow, true);

  // configuring signal monitor
  stateErr.disableErrors<InitState, OffState, ForcedIdleState, ReferencingState, SaveConfigurationState, EStopOffState,
                         EStopResetState, EStopOpenCircState>({{errorCode, subsystem}, {warningCode, subsystem}});
}

} // namespace logic
