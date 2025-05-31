/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_meril_modes_unlocked.h"
#include "lgc_meril_modes_common.h"
#include "lgc_meril_modes_instrument_calibration.h"
#include "lgc_meril_modes_locked.h"

namespace logic::meril {

void TransitionToLockedMode::enter() { fsmdata_.ctrl.out.gotoManipulatorManual = true; }

void TransitionToLockedMode::leave() { fsmdata_.ctrl.out.gotoManipulatorManual = false; }

mcx::state_machine::EventStatus TransitionToLockedMode::waitingForZeroVelocity() {
  if (fsmdata_.ctrl.velocityDetectorsNotActive) {
    return terminateEvent();
  }
  return mcx::state_machine::EVENT_REPEAT;
}

mcx::state_machine::EventStatus TransitionToLockedMode::terminateEvent() {
  if (!fsmdata_.ctrl.in.jointManipulatorIsLimiting) {
    if (returnStateId_ < 0) {
      log_error("UnlockedMode: Return state is not set, switching to LockedMode");
      returnStateId_ = LOGIC_STATE_ID(LockedMode);
    }
    setActiveState(returnStateId_);
  } else {
    log_error("UnlockedMode: Can not switch to [{}] {} mode while manipulator is limiting", returnStateId_,
              stateName(returnStateId_));
  }
  return mcx::state_machine::EVENT_DONE;
}

void UnlockedMode::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::UNLOCKED_M;
  fsmdata_.ctrl.out.gotoManipulatorManual = true;
  returnStateId_ = -1;
}

void UnlockedMode::leave() { fsmdata_.ctrl.out.gotoManipulatorManual = false; }

mcx::state_machine::EventStatus UnlockedMode::gotoUnlocked() { return gotoLocked(); }

mcx::state_machine::EventStatus UnlockedMode::gotoTeachFulcrum() {
  if (common::teachFulcrum(fsmdata_)) {
    if (!fsmdata_.ctrl.in.isCameraRobot) {
      const auto ptr = setActiveState<InstrumentCalibrateMode>();
      ptr->setReturnState(LOGIC_STATE_ID(LockedMode));
      addEvent({&SuperMerilModes::calibrateInstrument}, fsmdata_.instrumentCalibrationTimeoutSec);
    }
    addEvent({&SuperMerilModes::gotoLocked});
    return mcx::state_machine::EventStatus::EVENT_DONE;
  }
  return mcx::state_machine::EventStatus::EVENT_NONE;
}

mcx::state_machine::EventStatus UnlockedMode::gotoLocked() {
  const auto ptr = setActiveState<TransitionToLockedMode>();
  ptr->setReturnState(returnStateId_);
  addEvent({&SuperMerilModes::waitingForZeroVelocity}, fsmdata_.transitionTimeoutSec);
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

/// UNLOCKED INSTRUMENT MODE ------------------------------------------------------------------------------------------
void UnlockedInstrumentMode::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::UNLOCKED_INSTRUMENT_M;
  fsmdata_.ctrl.out.gotoManipulatorManual = true;
  returnStateId_ = -1;
}

void UnlockedInstrumentMode::leave() {
  fsmdata_.ctrl.out.gotoManipulatorManual = false;
  fsmdata_.ctrl.out.enableManipulatorManualLinear = false;
}

mcx::state_machine::EventStatus UnlockedInstrumentMode::gotoUnlocked() { return gotoLocked(); }

mcx::state_machine::EventStatus UnlockedInstrumentMode::gotoLocked() {
  if (!fsmdata_.ctrl.in.jointManipulatorIsLimiting) {
    if (returnStateId_ < 0) {
      log_error("UnlockedInstrumentMode: Return state is not set, switching to LockedMode");
      returnStateId_ = LOGIC_STATE_ID(LockedMode);
    }
    setActiveState(returnStateId_);
  } else {
    log_error("UnlockedInstrumentMode: Can not switch to [{}] {} mode while manipulator is limiting", returnStateId_,
              stateName(returnStateId_));
  }
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus UnlockedInstrumentMode::gotoSurgicalMode() {
  // return common::gotoSurgicalMode(*this, fsmdata_);
  // this is disabled for safety reasons. once an assistant is manually moving the robot, it can't be overruled by the
  // surgeon to go to surgical mode.
  return mcx::state_machine::EventStatus::EVENT_NONE;
}

mcx::state_machine::EventStatus UnlockedInstrumentMode::enableManipulatorManualLinear() {
  // fsmdata_.ctrl.out.enableManipulatorManualLinear = !fsmdata_.ctrl.out.enableManipulatorManualLinear;
  // return mcx::state_machine::EventStatus::EVENT_DONE;
  return mcx::state_machine::EventStatus::EVENT_NONE;
}

mcx::state_machine::EventStatus UnlockedInstrumentMode::gotoInstrumentStraighten() {
  fsmdata_.ctrl.out.gotoInstrumentStraightenRoll = false; // only for surgical mode
  return common::gotoInstrumentStraighten(*this, fsmdata_, id());
}

// ResetFulcrumUnlockedMode
void ResetFulcrumUnlockedMode::enter() {
  fsmdata_.ctrl.out.gotoResetFulcrum = true;
  returnStateId_ = -1;
}

void ResetFulcrumUnlockedMode::leave() { fsmdata_.ctrl.out.gotoResetFulcrum = false; }

mcx::state_machine::EventStatus ResetFulcrumUnlockedMode::gotoUnlocked() {
  if (fsmdata_.ctrl.in.fulcrumIsStored) {
    return mcx::state_machine::EVENT_REPEAT;
  }
  if (const auto statePtr = setActiveState<UnlockedMode>()) {
    statePtr->setReturnState(returnStateId_);
  }
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus ResetFulcrumUnlockedMode::terminateEvent() {
  log_info("Timeout during resetting fulcrum: {} sec, switching back to Locked Mode", fsmdata_.resetFulcrumTimoutSec);
  setActiveState<LockedMode>();
  return mcx::state_machine::EVENT_DONE;
}

} // namespace logic::meril