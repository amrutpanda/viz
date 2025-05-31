/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef MERIL_ROBOT_LGC_MERIL_MODES_DATA_H
#define MERIL_ROBOT_LGC_MERIL_MODES_DATA_H

#include "instrument/instrument_description.h"
#include "lgc_robot_states.h"
#include <mcx/core/visit_struct_intrusive.h>

namespace logic::meril {

enum MerilModes {
  INIT_M = 0,
  SLEEP_M = 10,
  MOVE_TO_SYMBOLIC_POSITION_M = 15,
  LOCKED_M = 20,
  UNLOCKED_M = 21,
  UNLOCKED_INSTRUMENT_M = 22,
  ADAPTER_CALIBRATION_M = 30,
  INSTRUMENT_STRAIGHTEN_M = 31,
  INSTRUMENT_EXCHANGE_M = 32,
  WAITING_FOR_INSTRUMENT_EXCHANGE_M = 33,
  RETRACT_INSTRUMENT_FROM_FULCRUM_M = 34,
  INSERT_INSTRUMENT_IN_FULCRUM_M = 35,
  INSERT_INSTRUMENT_AND_TEACH_FULCRUM_M = 36,
  INSTRUMENT_CALIBRATION_M = 37,
  SURGICAL_M = 40,
  INSERT_INSTRUMENT_TILL_RETRACT = 38,

  // Transitions between modes
  TO_SLEEP_T = 100,
  LOCKED_TO_SLEEP_T = 101,
  SLEEP_TO_LOCKED_T = 102,
  LOCKED_TO_UNLOCKED_T = 110,
  LOCKED_TO_UNLOCKED_INSTRUMENT_T = 111,
  LOCKED_TO_SURGICAL_T = 112,
  LOCKED_TO_INSTRUMENT_CALIBRATE_T = 113,
  INSTRUMENT_RETRACT_TO_EXCHANGE_T = 120,
  INSTRUMENT_EXCHANGE_TO_CALIBRATE_T = 130,
  INSTRUMENT_EXCHANGE_TO_RETRACT_T = 131,
  EXCHANGE_INSTRUMENT_TO_LOCKED_T = 222
};

enum MerilModeEvents {
  // Modes
  GOTO_NONE_E = 0
};

enum InstrumentConnectStatus : bool {
  // Modes
  NOT_CONNECTED = true,
  CONNECTED = false
};

enum InstrumentConnectHeader {
  // Modes
  IDLE = 0,
  CONNECT = 26,
  LOCKED = 25
};

enum ManipulatorAdmittanceJoints { NO_FULCRUM, FULCRUM };

struct ControlToMerilMode {
  BEGIN_VISITABLES(ControlToMerilMode);
  VISITABLE(bool, symbolicMoveIsDone);
  VISITABLE(bool, symbolicMoveIsStarted);
  VISITABLE(bool, isAtParkPosition);
  VISITABLE(bool, isAtLowPosition);
  VISITABLE(bool, surgicalModeIsAllowed);
  VISITABLE(bool, instrumentCalibrationDone);
  VISITABLE(bool, sterileAdapterCalibrationDone);
  VISITABLE(bool, instrumentStraightenDone);
  VISITABLE(bool, instrumentAutoStraighten);
  VISITABLE(bool, instrumentIsStraightening);
  VISITABLE(bool, instrumentInsertionDepthIsLimiting);
  VISITABLE(bool, instrumentIsOutsideFulcrum);
  VISITABLE(bool, instrumentIsAtCalibrationPosition);
  VISITABLE(bool, instrumentIsAtRetractPosition);
  VISITABLE(bool, fulcrumIsValid);
  VISITABLE(bool, fulcrumIsStored);
  VISITABLE(bool, exchangeIsDone);
  VISITABLE(bool, jointManipulatorIsLimiting);
  VISITABLE(bool, jointInstrumentIsLimiting);
  VISITABLE(bool, instrumentConstraintIsLimiting);
  VISITABLE(bool, cartesianIsLimiting);
  VISITABLE(bool, isCameraRobot);
  END_VISITABLES;
};

struct MerilModeToControl {
  BEGIN_VISITABLES(MerilModeToControl);
  VISITABLE(bool, updateJointLimits);
  VISITABLE(bool, gotoSymbolicMove);
  VISITABLE(bool, gotoManipulatorManual);
  VISITABLE(bool, enableManipulatorManualLinear); // linear move is used during Manual.
  VISITABLE(bool, gotoInstrumentStraighten);
  VISITABLE(bool, gotoInstrumentStraightenRoll);
  VISITABLE(bool, gotoSurgicalMode);
  VISITABLE(bool, gotoTeachFulcrum);
  VISITABLE(bool, gotoResetFulcrum);
  VISITABLE(bool, gotoTeachInsertionDepth);
  VISITABLE(bool, gotoResetInsertionDepth);
  VISITABLE(bool, gotoMaintenanceMode);
  VISITABLE(bool, storeInstrumentRetractPosition);
  VISITABLE(bool, enableStopAtRetractPosition);
  VISITABLE(bool, enableStopAtCalibrationPosition);
  VISITABLE(bool, enableStraightenRollOutsideFulcrum);
  VISITABLE(bool, enableImpedanceInUnlockedInstrumentMode); // enableImpedanceInUnlockedInstrumentMode);
  VISITABLE(bool, gotoLockedDirect);
  VISITABLE(bool, gotoInstrumentCalibration);
  VISITABLE(bool, gotoSterileAdapterCalibration);
  VISITABLE(uint16_t, instrumentCalibrationChannel);
  VISITABLE(bool, instrumentIsConnected);
  VISITABLE(bool, cameraDirectionIsReversed);
  VISITABLE(ManipulatorAdmittanceJoints, manipulatorAdmittanceJoints);
  END_VISITABLES;
};

struct MerilModeToRfid {
  BEGIN_VISITABLES(MerilModeToRfid);
  VISITABLE(uint16_t, controlWord);
  VISITABLE(uint16_t, header);
  END_VISITABLES;
};

struct RfidToMerilMode {
  BEGIN_VISITABLES(RfidToMerilMode);
  VISITABLE(InstrumentConnectStatus, sterileAdapterConnectionStatus);
  VISITABLE(InstrumentConnectStatus, instrumentConnectionStatus);
  VISITABLE(uint16_t, statusWord);
  VISITABLE(uint16_t, header);
  VISITABLE(uint16_t, type);
  VISITABLE(uint16_t, version);
  VISITABLE(uint16_t, numberOfUsages);
  VISITABLE(uint16_t, usageLimit);
  END_VISITABLES;
};

struct MerilModesFSMData {
  struct {
    ControlToMerilMode in{};
    MerilModeToControl out{};

    struct {
      bool active{};
      double timeout{5.0};
    } fulcrumWatchdog;
    bool cartIsDocked;

    struct {
      bool enableExchangeStopAtOutsideFulcrum{true};
      bool retractPositionIsStored{false};
    } instrumentExchange;

    bool disablePVALimiter{false};
    bool velocityDetectorsNotActive{true};

    struct {
      bool isDisabled{true};    // hardware input conforming z-actuator can't be actuated
      bool disable;             // disable via hardware output that z-actuator can't be actuated
    } zActuator;

    struct {
      bool enableImpedanceInUnlockedInstrumentMode{false};
      bool enableHandguidingSingleClickMode{false}; // enable robot manual button
      bool enableMaintenanceMode{false};
    } settings;
  } ctrl{};

  struct Instrument {
    struct RFID {
      RfidToMerilMode in{};
      MerilModeToRfid out{};
      struct Reader {
        static constexpr auto INSTRUMENT_CONNECTION_STATUS = 128;
        static constexpr std::array<int, 7> COMMANDS{0, 65, 66, 67, 72, 80, 4};
        unsigned int commandNumber{0};
        double commandCycleSec{0.5};
        double commandRetrySec{0};
      } state;
    } rfid;
    INSTRUMENT_ID_TYPE activeInstrumentId{NO_INSTRUMENT_ID};
    bool exchangeInstrumentAllowed{};
    struct {
      bool instrumentConnectionDetected{false};
      bool enableInstrumentTactileSwitch{false};
      bool enableSterileAdapterTactileSwitch{false};
      bool enableRFIDConnect{true};
      bool sterileAdapterConnect{false};
      bool sterileAdapterIsConnected{false};
      bool gotoManualConnectGuiButton{false};
      INSTRUMENT_ID_TYPE type{};
    } control;
  } instrument;

  MerilModes currentFsmModeOut{MerilModes::INIT_M};

  MerilModeEvents externalModeEventIn{MerilModeEvents::GOTO_NONE_E};

  States actualRobotState{};

  // timeout values
  double resetFulcrumTimoutSec{1.0};
  double instrumentRetractTimeoutSec{60.0};
  double instrumentExchangeTimeout{60.0};
  double instrumentInsertTillFulcrumTimeoutSec{60.0};
  double instrumentAndTeachFulcrumTimeoutSec{60.0};
  double instrumentStraighteningTimeoutSec{10.0};
  double instrumentCalibrationTimeoutSec{20.0};
  double sterileAdapterCalibrationTimeoutSec{20.0};
  double symbolicPositioningTimeoutSec{300.0};
  double transitionTimeoutSec{5.0};
};
} // namespace logic::meril

#endif // MERIL_ROBOT_LGC_MERIL_MODES_DATA_H
