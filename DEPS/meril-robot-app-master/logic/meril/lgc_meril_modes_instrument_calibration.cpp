/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_meril_modes_instrument_calibration.h"
#include "lgc_meril_modes_locked.h"

namespace logic::meril {

void InstrumentCalibrateMode::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::INSTRUMENT_CALIBRATION_M;
  fsmdata_.ctrl.out.gotoInstrumentCalibration = true;
  returnStateId_ = -1;
}

void InstrumentCalibrateMode::leave() { fsmdata_.ctrl.out.gotoInstrumentCalibration = false; }

mcx::state_machine::EventStatus InstrumentCalibrateMode::calibrateInstrument() {
  if (fsmdata_.ctrl.in.instrumentCalibrationDone) {
    log_info("Instrument Calibration is Done, leaving Instrument Calibration mode");
    fsmdata_.ctrl.out.gotoInstrumentCalibration = false;
    if (returnStateId_ < 0) {
      log_error("InstrumentCalibrateMode: Return state is not set, switching to LockedMode");
      setActiveState<LockedMode>();
    } else {
      setActiveState(returnStateId_);
    }
    return mcx::state_machine::EventStatus::EVENT_DONE;
  }
  return mcx::state_machine::EventStatus::EVENT_REPEAT;
}

mcx::state_machine::EventStatus InstrumentCalibrateMode::terminateEvent() {
  log_error("Instrument Calibration timeout, leaving Instrument Calibration mode (timer = {})",
            fsmdata_.instrumentCalibrationTimeoutSec);
  fsmdata_.ctrl.out.gotoInstrumentCalibration = false;
  if (returnStateId_ < 0) {
    log_error("InstrumentCalibrateMode: Return state is not set, switching to LockedMode");
    setActiveState<LockedMode>();
  } else {
    setActiveState(returnStateId_);
  }
  return mcx::state_machine::EVENT_DONE;
}

} // namespace logic::meril