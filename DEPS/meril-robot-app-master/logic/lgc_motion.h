/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_MOTION_H
#define LGC_MOTION_H

#include "instrument/instrument_description.h"
#include "lgc_robot_modes.h"
#include "lgc_robot_modes_data.h"
#include "lgc_robot_states_base.h"
#include "lgc_robot_states_data.h"
#include "meril/lgc_meril_modes_base.h"
#include "meril/lgc_meril_modes_data.h"
#include <mcx/control3.h>
#include <mcx/core.h>
#include <mcx/motion.h>
#include <string>

namespace logic {

class MotionLogic final : public mcx::container::Module {

public:
  MotionLogic(std::string saveConfigFilePath, bool instrumentEnabled, unsigned int numJoints,
              mcx::utils::SystemMode systemMode);

  ~MotionLogic() override = default;

private:
  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override;

  bool initPhase1_() override;

  bool initPhase2_() override;

  bool startOp_() override;

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override;

  bool stopOp_() override;

  static ModeEvents toFsmModeEvent(mcx::motion::ModeEvents modeEvent);

  void addWindowDetectorError(uint32_t warningCode, uint32_t errorCode, uint32_t subsystem,
                              mcx::control3::WindowDetectorStateData* wd);

  void addWindowDetectorFd(uint32_t warningCode, uint32_t errorCode, uint32_t subsystem,
                           mcx::control3::WindowDetectorStateData* wd);

  // void setStatusLights(const LEDStatus statusLed) { stateFsmData_.bus.out.status_led = statusLed; }
  void setStatusLights(const RGBLight statusLight) {
    stateFsmData_.bus.out.status_light[0] = statusLight.value[0];
    stateFsmData_.bus.out.status_light[1] = statusLight.value[1];
    stateFsmData_.bus.out.status_light[2] = statusLight.value[2];
    stateFsmData_.bus.out.status_led = statusLight.led;
  }

  void configureButtons();
  void colorManager();
  void initTorqueConditions();
  void activateSignalMonitor();
  void syncFsm(bool isInSimulationMode);

  const std::string configFilePath_;

  mcx::parameter_server::ParamHandle externalModeEventInHandle_;
  mcx::state_machine::StateMachine<SuperMode> motionModesFsm_;

  mcx::parameter_server::ParamHandle stateEventHandle_;
  mcx::state_machine::StateMachine<SuperState> robotStatesFsm_;

  mcx::parameter_server::ParamHandle merilModesEventHandle_;
  mcx::state_machine::StateMachine<meril::SuperMerilModes> merilModesFsm_;
  mcx::signal_monitor::SignalMonitor<bool> merilButtons_;

  StateFSMData stateFsmData_{};
  ModeFSMData modeFsmData_{};
  meril::MerilModesFSMData merilFsmData_{};

  const bool instrumentEnabled_{};
  const unsigned int numJoints_;

  // StateEvents state_event_;

  // Error monitoring and handling
  bool guiFaultAcknowledge_{};

  bool gotoInstrumentExchangeGuiButton_{false};
  bool gotoInstrumentExchangeButton_{false};
  bool gotoInstrumentExchange_{false};

  bool gotoInstrumentStraightenUdp_{false};
  bool gotoInstrumentStraightenGuiButton_{false};
  bool gotoInstrumentStraightenButton_{false};
  bool gotoInstrumentStraighten_{false};

  bool guiRetract_{false};
  bool gotoInstrumentRetract_{false};

  bool gotoLockedDirectGuiButton_{false};

  bool gotoInstrumentCalibrateGuiButton_{false};
  bool gotoCameraReverseGuiButton_{false};

  bool gotoUnlockGuiButton_{false};
  bool gotoUnlockButton_{false};

  bool gotoResetInsertionDepth_{false};
  bool gotoTeachInsertionDepth_{false};

  bool gotoTeachGuiButton_{false};
  bool gotoTeachButton_{false};
  bool gotoResetFulcrum_{false};

  bool gotoSurgicalGuiButton_{false};
  bool gotoSurgicalButton_{false};
  bool gotoSurgicalPedal_{false};
  bool gotoSurgicalMode_{false};

  bool gotoSymbolicPositionsButton_{false};
  bool gotoSymbolicPositionsGuiButton_{false};
  bool gotoSymbolicPositionMode_{false};

  bool zActuatorDisable_{false};
  bool zActuatorIsDisabled_{false};

  unsigned int cycleCount_{};

  // setting parameters
  bool enableAutoStraighten_{false};

  // bool enableAdmittanceUnlock_{};

  struct Errors {

    Errors() = delete;

    explicit Errors(const unsigned int numJoints) {
      actuatorLimitersActive = std::make_unique<bool[]>(numJoints);
      jumpDetector = std::make_unique<mcx::control3::WindowDetectorStateData[]>(numJoints);
      posErrDetector = std::make_unique<mcx::control3::WindowDetectorStateData[]>(numJoints);
      posDetector = std::make_unique<mcx::control3::WindowDetectorStateData[]>(numJoints);
      velDetector = std::make_unique<mcx::control3::WindowDetectorStateData[]>(numJoints);
      accDetector = std::make_unique<mcx::control3::WindowDetectorStateData[]>(numJoints);
      torqueDetector = std::make_unique<mcx::control3::WindowDetectorStateData[]>(numJoints);
      torqueSensorDetector = std::make_unique<mcx::control3::WindowDetectorStateData[]>(numJoints);
      collisionDetector = std::make_unique<mcx::control3::WindowDetectorStateData[]>(numJoints);
      outputLimiterIsActive = std::make_unique<bool[]>(numJoints);
      slaveHasNoError = std::make_unique<bool[]>(numJoints);
      slaveErrorCode = std::make_unique<unsigned int[]>(numJoints);
      driveI2tActive = std::make_unique<bool[]>(numJoints);
    }

    ~Errors() = default;

    mcx::control3::WindowDetectorStateData rtErrDetector{};
    mcx::control3::WindowDetectorStateData manipulabilityErrDetector{};
    mcx::control3::WindowDetectorStateData xErrDetector{};
    mcx::control3::WindowDetectorStateData yErrDetector{};
    mcx::control3::WindowDetectorStateData zErrDetector{};

    std::unique_ptr<bool[]> actuatorLimitersActive;
    std::unique_ptr<mcx::control3::WindowDetectorStateData[]> jumpDetector;
    std::unique_ptr<mcx::control3::WindowDetectorStateData[]> posErrDetector;
    std::unique_ptr<mcx::control3::WindowDetectorStateData[]> posDetector;
    std::unique_ptr<mcx::control3::WindowDetectorStateData[]> velDetector;
    std::unique_ptr<mcx::control3::WindowDetectorStateData[]> accDetector;
    std::unique_ptr<mcx::control3::WindowDetectorStateData[]> torqueDetector;
    std::unique_ptr<mcx::control3::WindowDetectorStateData[]> torqueSensorDetector;
    std::unique_ptr<mcx::control3::WindowDetectorStateData[]> collisionDetector;
    std::unique_ptr<bool[]> outputLimiterIsActive;

    std::unique_ptr<bool[]> slaveHasNoError;
    std::unique_ptr<unsigned int[]> slaveErrorCode;
    std::unique_ptr<bool[]> driveI2tActive;

  } errors_;

  std::unique_ptr<mcx::drive::DriveMode[]> driveModeOut_;
  std::unique_ptr<bool[]> drivesWithTorqueMode_;
  std::unique_ptr<bool[]> drivesWithTorqueModeInUnlockedInstrument_;
  mcx::drive::DriveCommand driveCommandOut_{mcx::drive::DriveCommand::DRIVE_CMD_OFF};

  unsigned int runMode_{mcx::cmd_line::SystemMode::PRODUCTION};

  mcx::control3::ControlToState instrumentAxesCtrlToState_{};
  mcx::control3::ControlToState manipulatorAxesCtrlToState_{};

  struct StateTransitionTimeout {
    BEGIN_VISITABLES(StateTransitionTimeout);
    VISITABLE(double, gotoOff);
    VISITABLE(double, gotoIdle);
    VISITABLE(double, gotoReference);
    VISITABLE(double, gotoEngage);
    VISITABLE(double, gotoHoming);
    END_VISITABLES;
  } stateTransitionTimeout_{
      .gotoOff = 5.0, .gotoIdle = 5.0, .gotoReference = 2.0, .gotoEngage = 5.0, .gotoHoming = 5.0};

  struct ModeTransitionTimeout {
    BEGIN_VISITABLES(ModeTransitionTimeout);
    VISITABLE(double, gotoInit);
    VISITABLE(double, gotoPause);
    VISITABLE(double, gotoManualCart);
    VISITABLE(double, gotoManualJoin);
    VISITABLE(double, gotoSemiAuto);
    VISITABLE(double, gotoTorqueInitialDelay);
    VISITABLE(double, gotoTorque);
    END_VISITABLES;
  } modeTransitionTimeout_{.gotoInit = 15.0,
                           .gotoPause = 15.0,
                           .gotoManualCart = 15.0,
                           .gotoManualJoin = 15.0,
                           .gotoSemiAuto = 15.0,
                           .gotoTorqueInitialDelay = 5.0,
                           .gotoTorque = 15.0};
};

} // namespace logic

#endif /* LGC_MOTION_H */
