/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_motion.h"

#include <meril/lgc_meril_modes_locked.h>
#include <meril/lgc_meril_modes_surgical.h>

namespace logic {

/**
 * @brief Configures button mappings and their associated actions for various control modes in the system.
 *
 * This method initializes and defines button configurations using `merilButtons_` with specific categories
 * and subcategories. Each button is associated with a callback action and a triggering mode. This includes
 * the setup for multiple buttons across operational states such as Teach Fulcrum, Surgical Mode, Instrument
 * Straighten, Instrument Exchange, Unlocked, Symbolic Positions, and Instrument Calibration.
 *
 * The method maps each button identifier to its respective operational logic via lambda functions. These
 * lambdas define actions taken when the button is pressed, and they trigger associated finite state machine
 * (FSM) events using `merilModesFsm_`. Buttons are also configured with debounce time parameters.
 *
 * Conditional settings may apply based on availability of certain features or system configurations
 * (e.g., presence of camera robot).
 *
 * Key button categories and their functions:
 * - **Teach Fulcrum**: Handles buttons and GUI actions for navigating to teaching or resetting Fulcrum.
 * - **Surgical Mode**: Sets up buttons for entering the Surgical Mode.
 * - **Instrument Straighten**: Configures buttons for navigating to the instrument straighten logic.
 * - **Instrument Exchange**: Handles buttons for manual linear movements or instrument exchange procedures.
 * - **Unlocked**: Configures buttons to trigger the system into an unlocked state.
 * - **Symbolic Positions**: Sets up buttons to move to symbolic positions within the system.
 * - **Instrument Calibration**: Establishes configuration for instrument calibration via GUI-specific buttons.
 */
void MotionLogic::configureButtons() {
  constexpr auto TEACH_FULCRUM = 1;
  // subgroups
  constexpr auto TEACH_FULCRUM_BUTTON = 1;
  constexpr auto TEACH_FULCRUM_GUI_BUTTON = 2;
  constexpr auto RESET_FULCRUM_BUTTON = 3;
  constexpr auto RESET_FULCRUM_GUI_BUTTON = 4;
  constexpr auto RESET_FULCRUM_WATCHDOG = 5;
  constexpr auto SEC_2 = 2;
  constexpr auto MSEC_50 = 0.050;
  // constexpr auto MSEC_2000 = 2.000;
  // constexpr auto MSEC_100 = 0.100;
  merilButtons_.add(
      {TEACH_FULCRUM, TEACH_FULCRUM_BUTTON}, gotoTeachButton_, true,
      [&] {
        log_debug("Teach Fulcrum button pressed");
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoTeachFulcrum);
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, MSEC_50);
  merilButtons_.add(
      {TEACH_FULCRUM, TEACH_FULCRUM_GUI_BUTTON}, gotoTeachGuiButton_, true,
      [&] {
        log_debug("Teach Fulcrum GUI button pressed");
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoTeachFulcrum);
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE);
  merilButtons_.add(
      {TEACH_FULCRUM, RESET_FULCRUM_BUTTON}, gotoTeachButton_, true,
      [&] {
        log_debug("Reset Fulcrum button pressed");
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoResetFulcrum);
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, SEC_2);
  merilButtons_.add(
      {TEACH_FULCRUM, RESET_FULCRUM_GUI_BUTTON}, gotoTeachGuiButton_, true,
      [&]() {
        log_debug("Reset Fulcrum GUI button pressed");
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoResetFulcrum);
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, SEC_2);
  merilButtons_.add(
      {TEACH_FULCRUM, RESET_FULCRUM_WATCHDOG}, merilFsmData_.ctrl.fulcrumWatchdog.active, true,
      [&]() {
        log_debug("Reset Fulcrum watchdog active");
        merilModesFsm_.setActiveState<meril::LockedMode>();
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoResetFulcrum);
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE);

  constexpr auto SURGICAL = 2;
  constexpr auto SURGICAL_BUTTON = 1;
  constexpr auto SURGICAL_PEDAL = 2;
  constexpr auto SURGICAL_GUI_BUTTON = 3;
  merilButtons_.add(
      {SURGICAL, SURGICAL_BUTTON}, gotoSurgicalButton_, true,
      [&] {
        log_debug("Surgical button pressed");
        if (merilModesFsm_.isStateActive<meril::SurgicalMode>()) {
          merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoLocked);
        } else {
          merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoSurgicalMode);
        }
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, MSEC_50);
  merilButtons_.add(
      {SURGICAL, SURGICAL_PEDAL}, gotoSurgicalPedal_, true,
      [&] {
        log_debug("Surgical Pedal Released, going to surgical mode");
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoSurgicalMode);
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE);
  merilButtons_.add(
      {SURGICAL, SURGICAL_GUI_BUTTON}, gotoSurgicalGuiButton_, true,
      [&] {
        log_debug("Surgical GUI button pressed");
        if (merilModesFsm_.isStateActive<meril::SurgicalMode>()) {
          merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoLocked);
        } else {
          merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoSurgicalMode);
        }
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE);

  constexpr auto INSTRUMENT_STRAIGHTEN = 3;
  constexpr auto INSTRUMENT_STRAIGHTEN_BUTTON = 1;
  merilButtons_.add(
      {INSTRUMENT_STRAIGHTEN, INSTRUMENT_STRAIGHTEN_BUTTON}, gotoInstrumentStraightenButton_, true,
      [&] {
        log_debug("Instrument straighten button pressed");
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoInstrumentStraighten);
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, MSEC_50);
  if (!merilFsmData_.ctrl.in.isCameraRobot) {
    constexpr auto INSTRUMENT_STRAIGHTEN_GUI_BUTTON = 2;
    constexpr auto INSTRUMENT_STRAIGHTEN_UDP = 3;
    merilButtons_.add(
        {INSTRUMENT_STRAIGHTEN, INSTRUMENT_STRAIGHTEN_GUI_BUTTON}, gotoInstrumentStraightenGuiButton_, true,
        [&] {
          log_debug("Instrument straighten GUI button pressed");
          merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoInstrumentStraighten);
          return true;
        },
        mcx::signal_monitor::TRIGGER_ONCE);
    merilButtons_.add(
        {INSTRUMENT_STRAIGHTEN, INSTRUMENT_STRAIGHTEN_UDP}, gotoInstrumentStraightenUdp_, true,
        [&] {
          log_debug("Instrument straighten from the UDP");
          merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoInstrumentStraighten);
          return true;
        },
        mcx::signal_monitor::TRIGGER_ONCE);
  }
  constexpr auto INSTRUMENT_EXCHANGE = 4;
  // constexpr auto INSTRUMENT_LINEARMOVE_BUTTON = 1;
  constexpr auto INSTRUMENT_EXCHANGE_BUTTON = 2;
  constexpr auto INSTRUMENT_EXCHANGE_GUI_BUTTON = 3;

  merilButtons_.add(
      {INSTRUMENT_EXCHANGE, INSTRUMENT_EXCHANGE_BUTTON}, gotoInstrumentExchangeButton_, true,
      [&] {
        log_debug("Instrument linear move button pressed");
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoInstrumentExchange);
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, SEC_2);
  merilButtons_.add(
      {INSTRUMENT_EXCHANGE, INSTRUMENT_EXCHANGE_GUI_BUTTON}, gotoInstrumentExchangeGuiButton_, true,
      [&] {
        log_debug("Instrument exchange GUI button pressed");
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoInstrumentExchange);
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE);

  constexpr auto UNLOCKED = 5;
  constexpr auto UNLOCKED_BUTTON_SINGLE_CLICK_MODE = 1;
  constexpr auto UNLOCKED_BUTTON_HOLD_MODE = 11;
  constexpr auto UNLOCKED_GUI_BUTTON = 2;
  merilButtons_.add(
      {UNLOCKED, UNLOCKED_BUTTON_SINGLE_CLICK_MODE}, gotoUnlockButton_, true,
      [&] {
        if (merilFsmData_.ctrl.settings.enableHandguidingSingleClickMode) {
          log_info("Unlocked button pressed");
          merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoUnlocked);
        }
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, MSEC_50);
  merilButtons_.add(
      {UNLOCKED, UNLOCKED_BUTTON_HOLD_MODE}, gotoUnlockButton_, true,
      [&] {
        if (!merilFsmData_.ctrl.settings.enableHandguidingSingleClickMode) {
          log_debug("Unlocked button hold");
          gotoUnlockButton_ ? merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoUnlocked)
                            : merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoLocked);
        }
        return true;
      },
      mcx::signal_monitor::TRIGGER_TOGGLE, MSEC_50);
  merilButtons_.add(
      {UNLOCKED, UNLOCKED_GUI_BUTTON}, gotoUnlockGuiButton_, true,
      [&] {
        log_debug("Unlocked GUI button pressed");
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoUnlocked);
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, MSEC_50);

  constexpr auto SYMBOLIC_POSITIONS = 6;
  constexpr auto SYMBOLIC_POSITIONS_BUTTON = 1;
  constexpr auto SYMBOLIC_POSITIONS_GUI_BUTTON = 2;
  merilButtons_.add(
      {SYMBOLIC_POSITIONS, SYMBOLIC_POSITIONS_BUTTON}, gotoSymbolicPositionsButton_, true,
      [&] {
        if (robotStatesFsm_.isStateActive<EngagedState>()) {
          log_debug("Symbolic positions button pressed");
          merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoMoveSymbolicPosition);
        }
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, MSEC_50);
  merilButtons_.add(
      {SYMBOLIC_POSITIONS, SYMBOLIC_POSITIONS_GUI_BUTTON}, gotoSymbolicPositionsGuiButton_, true,
      [&] {
        if (robotStatesFsm_.isStateActive<EngagedState>()) {
          log_debug("Symbolic positions GUI button pressed");
          merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoMoveSymbolicPosition);
        }
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE);

  constexpr auto INSTRUMENT_CALIBRATE = 7;
  constexpr auto INSTRUMENT_CALIBRATE_GUI_BUTTON = 1;
  merilButtons_.add(
      {INSTRUMENT_CALIBRATE, INSTRUMENT_CALIBRATE_GUI_BUTTON}, gotoInstrumentCalibrateGuiButton_, true,
      [&] {
        log_debug("Instrument calibrate GUI button pressed");
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoInstrumentCalibrate);
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE);

  constexpr auto INSTRUMENT_CONNECT = 8;
  constexpr auto INSTRUMENT_CONNECT_SWITCH = 1;
  constexpr auto INSTRUMENT_DISCONNECT_SWITCH = 2;
  constexpr auto INSTRUMENT_CONNECT_GUI_BUTTON = 3;
  constexpr auto INSTRUMENT_ENABLE_RFID_GUI_BUTTON = 4;
  auto& instrument = merilFsmData_.instrument;
  merilButtons_.add(
      {INSTRUMENT_CONNECT, INSTRUMENT_CONNECT_SWITCH}, instrument.control.instrumentConnectionDetected, true,
      [&] {
        if (instrument.control.enableRFIDConnect) {
          if (instrument.rfid.in.statusWord >=
              meril::MerilModesFSMData::Instrument::RFID::Reader::INSTRUMENT_CONNECTION_STATUS) {
            const auto done = merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoInstrumentConnect) ==
                              mcx::state_machine::EVENT_DONE;
            return done;
          }
          return false;
        }
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, MSEC_50);
  merilButtons_.add(
      {INSTRUMENT_CONNECT, INSTRUMENT_DISCONNECT_SWITCH}, instrument.control.instrumentConnectionDetected, false,
      [&] {
        if (instrument.control.enableRFIDConnect) {
          const auto done = merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoInstrumentDisconnect) ==
                            mcx::state_machine::EVENT_DONE;
          return done;
        }
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, MSEC_50);

  merilButtons_.add(
      {INSTRUMENT_CONNECT, INSTRUMENT_CONNECT_GUI_BUTTON}, instrument.control.gotoManualConnectGuiButton, true,
      [&] {
        if (!instrument.control.enableRFIDConnect) {
          const auto done = merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoManualInstrumentConnect) ==
                            mcx::state_machine::EVENT_DONE;
          return done;
        }
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, 0);

  merilButtons_.add(
      {INSTRUMENT_CONNECT, INSTRUMENT_ENABLE_RFID_GUI_BUTTON}, instrument.control.enableRFIDConnect,
      !instrument.control.enableRFIDConnect,
      [&] {
        const auto done = merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoInstrumentDisconnect) ==
                          mcx::state_machine::EVENT_DONE;
        if (instrument.control.enableRFIDConnect) {
          merilButtons_.reset(mcx::signal_monitor::SignalId{INSTRUMENT_CONNECT, INSTRUMENT_CONNECT_SWITCH});
        }
        return done;
      },
      mcx::signal_monitor::TRIGGER_TOGGLE, 0);

  constexpr auto ADAPTER_CONNECT = 9;
  constexpr auto ADAPTER_CONNECT_TACTILE_SWITCH = 1;
  constexpr auto ADAPTER_DISCONNECT_TACTILE_SWITCH = 2;
  merilButtons_.add(
      {ADAPTER_CONNECT, ADAPTER_CONNECT_TACTILE_SWITCH}, merilFsmData_.instrument.control.sterileAdapterConnect, true,
      [&] {
        log_info("Sterile adapter connect detected");
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoAdapterCalibrate);
        merilFsmData_.instrument.control.sterileAdapterIsConnected = true;
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, SEC_2);
  merilButtons_.add(
      {ADAPTER_CONNECT, ADAPTER_DISCONNECT_TACTILE_SWITCH}, merilFsmData_.instrument.control.sterileAdapterConnect,
      false,
      [&] {
        log_info("Sterile adapter disconnect detected");
        merilFsmData_.instrument.control.sterileAdapterIsConnected = false;
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, SEC_2);

  constexpr auto CAMERA_REVERSE_DIRECTION = 10;
  constexpr auto CAMERA_REVERSE_DIRECTION_TRIGGER = 1;
  constexpr auto CAMERA_REVERSE_DIRECTION_GUI_BUTTON = 2;

  merilButtons_.add(
      {CAMERA_REVERSE_DIRECTION, CAMERA_REVERSE_DIRECTION_TRIGGER}, gotoInstrumentStraightenUdp_, true,
      [&] {
        if (merilFsmData_.ctrl.in.isCameraRobot) {
          log_info("Camera reverse button pressed");
          merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoCameraReverseDirection);
        }
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, MSEC_50);
  merilButtons_.add(
      {CAMERA_REVERSE_DIRECTION, CAMERA_REVERSE_DIRECTION_GUI_BUTTON}, gotoCameraReverseGuiButton_, true,
      [&] {
        if (merilFsmData_.ctrl.in.isCameraRobot) {
          log_info("Camera reverse GUI button pressed");
          merilModesFsm_.executeEvent(&meril::SuperMerilModes::gotoCameraReverseDirection);
        }
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, MSEC_50);

  constexpr auto EXCHANGE_ABORT = 11;
  constexpr auto ROBOT_BUTTON = 1;
  merilButtons_.add(
      {EXCHANGE_ABORT, ROBOT_BUTTON}, gotoUnlockButton_, true,
      [&] {
        merilModesFsm_.executeEvent(&meril::SuperMerilModes::exchangeAbort);
        return true;
      },
      mcx::signal_monitor::TRIGGER_ONCE, MSEC_50);
}
} // namespace logic