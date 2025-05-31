/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_meril_modes_instrument_exchange.h"
#include "lgc_meril_modes_common.h"
#include "lgc_meril_modes_instrument_calibration.h"
#include "lgc_meril_modes_locked.h"

namespace logic::meril {
/**
 * Instrument Exchange mode.
 */
void InstrumentExchangeMode::enter() { fsmdata_.currentFsmModeOut = MerilModes::INSTRUMENT_EXCHANGE_M; }

/**
 * Updates the state of the InstrumentExchangeMode by resetting control and
 * instrument exchange settings.
 */
void InstrumentExchangeMode::leave() { fsmdata_.ctrl.out.storeInstrumentRetractPosition = false; }

/**
 * Sets flags to enable the storage of the instrument's retract position
 * and the stop at retract position for manual insertion.
 */
mcx::state_machine::EventStatus InstrumentExchangeMode::storeRetractPosition() {
  if (!fsmdata_.ctrl.in.instrumentConstraintIsLimiting && !fsmdata_.ctrl.in.instrumentInsertionDepthIsLimiting) {
    fsmdata_.ctrl.instrumentExchange.retractPositionIsStored = true;
    fsmdata_.ctrl.out.storeInstrumentRetractPosition = true;
  }
  addEvent({&SuperMerilModes::delayEvent}, 0.01);
  addEvent({&SuperMerilModes::alignToFulcrum});
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

/**
 * Aligns the instrument to the fulcrum and switches to RetractInstrument state.
 */
mcx::state_machine::EventStatus InstrumentExchangeMode::alignToFulcrum() {
  fsmdata_.ctrl.out.storeInstrumentRetractPosition = false;
  setActiveState<RetractInstrument>();
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

/**
 * Switches to WaitingForInstrumentExchange state for the instrument exchange process.
 */
mcx::state_machine::EventStatus InstrumentExchangeMode::waitExchangeInstrument() {
  // trying to straighten and switch to WaitingForInstrumentExchange or directly go to WaitingForInstrumentExchange
  if (common::gotoInstrumentStraighten(*this, fsmdata_, LOGIC_STATE_ID(WaitingForInstrumentExchange)) ==
      mcx::state_machine::EventStatus::EVENT_NONE) {
    setActiveState<WaitingForInstrumentExchange>();
  }
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

/**
 * Updates the instrument settings as part of the instrument exchange process.
 */
mcx::state_machine::EventStatus InstrumentExchangeMode::updateInstrumentSettings() {
  if (fsmdata_.instrument.activeInstrumentId == NO_INSTRUMENT_ID) {
    log_warning("Instrument is not connected, aborting exchange procedure, switching to Locked mode");
    addEvent({&SuperMerilModes::exchangeDone});
  } else {
    addEvent({&SuperMerilModes::lockedEngageExchange});
  }
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

/**
 * Handles the insertion transition depending on the validity of the fulcrum.
 */
mcx::state_machine::EventStatus InstrumentExchangeMode::lockedEngageExchange() {
  if (fsmdata_.ctrl.in.fulcrumIsValid) {
    // trying to straighten and switch to InstrumentInsertTillFulcrum or directly go to InstrumentInsertTillFulcrum
    if (common::gotoInstrumentStraighten(*this, fsmdata_, LOGIC_STATE_ID(InstrumentInsertTillFulcrum)) ==
        mcx::state_machine::EventStatus::EVENT_NONE) {
      setActiveState<InstrumentInsertTillFulcrum>();
    }
  } else {
    setActiveState<InstrumentInsertAndTeachFulcrum>();
  }
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

/**
 * Calibrating the instrument with the return state InstrumentInsert.
 */
mcx::state_machine::EventStatus InstrumentExchangeMode::calibrateInstrument() {
  const auto statePtr = setActiveState<InstrumentCalibrateMode>();
  statePtr->setReturnState(LOGIC_STATE_ID(InstrumentInsert));
  addEvent({&SuperMerilModes::calibrateInstrument}, fsmdata_.instrumentCalibrationTimeoutSec);
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

/**
 * Initiates the process to home the roll axis of the instrument.
 */
mcx::state_machine::EventStatus InstrumentExchangeMode::homeInstrumentRoll() {
  // addEvent({&SuperMerilModes::cameraAlignmentAndStraighten});
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

/**
 * Initiates the process to align the camera.
 */
mcx::state_machine::EventStatus InstrumentExchangeMode::cameraAlignmentAndStraighten() {
  // addEvent({&SuperMerilModes::instrumentInsert});
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

/**
 * The end of the exchange process, switching back to Locked mode.
 */
mcx::state_machine::EventStatus InstrumentExchangeMode::exchangeDone() {
  setActiveState<LockedMode>();
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

// StraightenInstrumentBeforeStore
void StraightenInstrumentBeforeStore::enter() {
  setActiveState<InstrumentExchangeMode>();
  addEvent({&SuperMerilModes::storeRetractPosition});
}

// RetractInstrument routine

void RetractInstrument::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::RETRACT_INSTRUMENT_FROM_FULCRUM_M;
  timer_ = 0;
  fsmdata_.ctrl.out.gotoManipulatorManual = false;
  fsmdata_.ctrl.out.enableManipulatorManualLinear = fsmdata_.ctrl.instrumentExchange.retractPositionIsStored;
  fsmdata_.ctrl.out.enableStopAtRetractPosition =
      fsmdata_.ctrl.instrumentExchange.retractPositionIsStored; // enable stop at retract for manual insert
}

void RetractInstrument::leave() {
  fsmdata_.ctrl.out.gotoManipulatorManual = false;
  fsmdata_.ctrl.out.enableManipulatorManualLinear = false;
  fsmdata_.ctrl.out.enableStopAtRetractPosition = false; // enable stop at retract for manual insert
  fsmdata_.ctrl.out.gotoInstrumentStraighten = false;
}

void RetractInstrument::iterate(double dtSec) {
  if (timer_ >= fsmdata_.instrumentRetractTimeoutSec) {
    log_error("RetractInstrument timeout, instrument exchange procedure is aborted, switching to Locked");
    setActiveState<LockedMode>();
  }
  if (fsmdata_.ctrl.in.instrumentIsOutsideFulcrum) {
    setActiveState<InstrumentExchangeMode>();
    addEvent({&SuperMerilModes::waitExchangeInstrument});
  }

  // todo: abort case to be added

  // if (fsmdata_.ctrl.out.gotoManipulatorManual) {
  timer_ += dtSec;
  // }
  // else {
  // timer_ = 0;
  // }
}

mcx::state_machine::EventStatus RetractInstrument::gotoInstrumentStraighten() {
  const auto alreadyEnabled = fsmdata_.ctrl.out.gotoInstrumentStraighten;
  fsmdata_.ctrl.out.gotoInstrumentStraighten = true;
  return !alreadyEnabled ? mcx::state_machine::EventStatus::EVENT_DONE : mcx::state_machine::EventStatus::EVENT_NONE;
}

mcx::state_machine::EventStatus RetractInstrument::gotoInstrumentExchange() {
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

// toggled by handguiding button
mcx::state_machine::EventStatus RetractInstrument::gotoUnlocked() {
  fsmdata_.ctrl.out.gotoManipulatorManual = !fsmdata_.ctrl.out.gotoManipulatorManual;
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

//

void WaitingForInstrumentExchange::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::WAITING_FOR_INSTRUMENT_EXCHANGE_M;
  timer_ = 0;
  instrumentConnectTimerSec_ = 0;
  // enable rfid reader
  fsmdata_.instrument.rfid.out.header = InstrumentConnectHeader::CONNECT;
}
void WaitingForInstrumentExchange::leave() {
  // disable rfid reader
  fsmdata_.instrument.rfid.out.header = InstrumentConnectHeader::LOCKED;
}

mcx::state_machine::EventStatus WaitingForInstrumentExchange::gotoManualInstrumentConnect() {
  common::gotoManualInstrumentConnect(fsmdata_);
  setActiveState<InstrumentExchangeMode>();
  addEvent({&SuperMerilModes::updateInstrumentSettings});
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus WaitingForInstrumentExchange::gotoInstrumentConnect() {
  const auto status = common::gotoInstrumentConnect(fsmdata_, instrumentConnectTimerSec_, getDtSec());
  if (status == mcx::state_machine::EVENT_DONE) {
    setActiveState<InstrumentExchangeMode>();
    addEvent({&SuperMerilModes::updateInstrumentSettings});
  }
  return status;
}

mcx::state_machine::EventStatus WaitingForInstrumentExchange::gotoInstrumentDisconnect() {
  return common::gotoInstrumentDisconnect(fsmdata_, instrumentConnectTimerSec_);
}

mcx::state_machine::EventStatus WaitingForInstrumentExchange::exchangeAbort() {
  setActiveState<LockedMode>();
  if (!fsmdata_.ctrl.out.instrumentIsConnected) {
    addEvent({&SuperMerilModes::gotoResetFulcrum});
  }
  return mcx::state_machine::EVENT_DONE;
}

void WaitingForInstrumentExchange::iterate(double dtSec) {
  if (timer_ >= fsmdata_.instrumentExchangeTimeout) {
    log_error("WaitingForInstrumentExchange timeout, instrument exchange procedure is aborted, switching to Locked");
    if (!fsmdata_.ctrl.out.instrumentIsConnected) {
      setActiveState<LockedMode>();
      addEvent({&SuperMerilModes::gotoResetFulcrum});
    }
  }
  timer_ += dtSec;
}

// InstrumentInsertTillFulcrum

void InstrumentInsertTillFulcrum::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::INSERT_INSTRUMENT_IN_FULCRUM_M;
  fsmdata_.ctrl.out.enableManipulatorManualLinear = true;
  fsmdata_.ctrl.out.enableStopAtCalibrationPosition = true;
  timer_ = 0;
}
void InstrumentInsertTillFulcrum::leave() {
  fsmdata_.ctrl.out.gotoManipulatorManual = false;
  fsmdata_.ctrl.out.enableManipulatorManualLinear = false;
  fsmdata_.ctrl.out.enableStopAtCalibrationPosition = false;
}
void InstrumentInsertTillFulcrum::iterate(double dtSec) {
  if (timer_ >= fsmdata_.instrumentInsertTillFulcrumTimeoutSec) {
    log_error("InstrumentInsertTillFulcrum timeout, instrument exchange procedure is aborted, switching to Locked");
    setActiveState<LockedMode>();
  }
  if (fsmdata_.ctrl.in.instrumentIsAtCalibrationPosition) {
    setActiveState<InstrumentExchangeMode>();
    addEvent({&SuperMerilModes::calibrateInstrument});
  }
  timer_ += dtSec;
}

mcx::state_machine::EventStatus InstrumentInsertTillFulcrum::gotoLocked() {
  fsmdata_.ctrl.out.gotoManipulatorManual = false;
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus InstrumentInsertTillFulcrum::gotoUnlocked() {
  fsmdata_.ctrl.out.gotoManipulatorManual = !fsmdata_.ctrl.out.gotoManipulatorManual;
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

// InstrumentInsertAndTeachFulcrum

void InstrumentInsertAndTeachFulcrum::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::INSERT_INSTRUMENT_AND_TEACH_FULCRUM_M;
  timer_ = 0;
}

void InstrumentInsertAndTeachFulcrum::leave() {}

void InstrumentInsertAndTeachFulcrum::iterate(double dtSec) {
  if (timer_ >= fsmdata_.instrumentAndTeachFulcrumTimeoutSec) {
    log_error("InstrumentInsertAndTeachFulcrum timeout, instrument exchange procedure is aborted, switching to Locked");
    setActiveState<LockedMode>();
  }
  if (fsmdata_.ctrl.in.fulcrumIsValid) {
    setActiveState<InstrumentExchangeMode>();
    addEvent({&SuperMerilModes::calibrateInstrument});
  }
  timer_ += dtSec;
}

mcx::state_machine::EventStatus InstrumentInsertAndTeachFulcrum::gotoTeachFulcrum() {
  return common::teachFulcrum(fsmdata_) ? mcx::state_machine::EventStatus::EVENT_DONE
                                        : mcx::state_machine::EventStatus::EVENT_NONE;
}

mcx::state_machine::EventStatus InstrumentInsertAndTeachFulcrum::gotoUnlocked() {
  return common::gotoUnlocked(*this, fsmdata_, id());
}

// Instrument insert
void InstrumentInsert::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::INSERT_INSTRUMENT_TILL_RETRACT;
  if (fsmdata_.ctrl.instrumentExchange.retractPositionIsStored) {
    log_info("Retract position is stored, enabling instrument insert till retract");
    fsmdata_.ctrl.out.enableStopAtRetractPosition = true;
    fsmdata_.ctrl.out.enableManipulatorManualLinear = true;
  } else {
    log_info("Retract position is not stored, enabling instrument insert");
  }
  if (fsmdata_.ctrl.settings.enableHandguidingSingleClickMode) {
    fsmdata_.ctrl.out.gotoManipulatorManual = true;
  }
}

void InstrumentInsert::leave() {
  fsmdata_.ctrl.out.enableStopAtRetractPosition = false;
  fsmdata_.ctrl.out.enableManipulatorManualLinear = false;
  fsmdata_.ctrl.out.gotoManipulatorManual = false;
}

void InstrumentInsert::iterate(double dtSec) {
  if (fsmdata_.ctrl.instrumentExchange.retractPositionIsStored) {
    if (fsmdata_.ctrl.in.instrumentIsAtRetractPosition) {
      setActiveState<InstrumentExchangeMode>();
      addEvent({&SuperMerilModes::exchangeDone});
    }
  }
}

mcx::state_machine::EventStatus InstrumentInsert::gotoLocked() {
  setActiveState<InstrumentExchangeMode>();
  addEvent({&SuperMerilModes::exchangeDone});
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus InstrumentInsert::gotoUnlocked() {
  fsmdata_.ctrl.out.gotoManipulatorManual = true;
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

} // namespace logic::meril