/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_meril_modes_adapter_calibration.h"
#include "lgc_meril_modes_locked.h"

namespace logic::meril {

void AdapterCalibrateMode::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::ADAPTER_CALIBRATION_M;
  // if instrument is connected then don't do this:
  if (fsmdata_.ctrl.out.instrumentIsConnected) {
    log_info(
        "Sterile Adapter Calibration is Terminated since an instrument is attached, leaving Adapter Calibration mode");
    setActiveState<LockedMode>();
  } else {
    fsmdata_.ctrl.out.gotoSterileAdapterCalibration = true;
    fsmdata_.ctrl.disablePVALimiter = true; // this is for all axes, todo: in future split for instrument and robot.
  }
  returnStateId_ = -1;
}

void AdapterCalibrateMode::leave() {
  fsmdata_.ctrl.out.gotoInstrumentCalibration = false;
  fsmdata_.ctrl.disablePVALimiter = false;
}

mcx::state_machine::EventStatus AdapterCalibrateMode::calibrateAdapter() {
  if (fsmdata_.ctrl.in.sterileAdapterCalibrationDone) {
    log_info("Sterile Adapter Calibration is Done, leaving Adapter Calibration mode");
    fsmdata_.ctrl.out.gotoSterileAdapterCalibration = false;
    setActiveState<LockedMode>();
    return mcx::state_machine::EventStatus::EVENT_DONE;
  }
  return mcx::state_machine::EventStatus::EVENT_REPEAT;
}

mcx::state_machine::EventStatus AdapterCalibrateMode::terminateEvent() {
  log_error("Sterile Adapter Calibration timeout, leaving Adapter Calibration mode (timer = {})",
            fsmdata_.sterileAdapterCalibrationTimeoutSec);
  fsmdata_.ctrl.out.gotoSterileAdapterCalibration = false;
  setActiveState<LockedMode>();
  return mcx::state_machine::EVENT_DONE;
}

} // namespace logic::meril