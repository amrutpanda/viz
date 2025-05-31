/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_meril_modes_symbolic_move.h"
#include "lgc_meril_modes_common.h"
#include "lgc_meril_modes_locked.h"
#include "lgc_meril_modes_sleep.h"

namespace logic::meril {
void SymbolicMoveMode::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::MOVE_TO_SYMBOLIC_POSITION_M;
  //  disable pva limiter
  fsmdata_.ctrl.disablePVALimiter = true;
  timer_ = 0;
}

void SymbolicMoveMode::leave() {
  fsmdata_.ctrl.out.gotoSymbolicMove = false;
  fsmdata_.ctrl.disablePVALimiter = false;
  fsmdata_.ctrl.out.updateJointLimits = false;
}

void SymbolicMoveMode::iterate(double dtSec) {
  // check if instrument is connected (tactile switch on sterile adapter == 3) or this is a camera robot (no tactile
  // switch) how can we check that the camera is not connected while doing symbolic moves?
  if ((fsmdata_.ctrl.out.instrumentIsConnected == false && !fsmdata_.ctrl.in.isCameraRobot) ||
      fsmdata_.ctrl.in.isCameraRobot) {
    // allow movement using symb.pos. even if camera is attached. (no input telling camera is attached.)
    fsmdata_.ctrl.out.updateJointLimits = true;
    fsmdata_.ctrl.out.gotoSymbolicMove = true;

    if ((fsmdata_.ctrl.in.symbolicMoveIsDone && fsmdata_.ctrl.in.symbolicMoveIsStarted) ||
        fsmdata_.ctrl.out.gotoLockedDirect || (timer_ > fsmdata_.symbolicPositioningTimeoutSec)) {
      if (timer_ < fsmdata_.symbolicPositioningTimeoutSec) {
        log_info("Symbolic Move is Done, leaving Symbolic Move mode");
      } else {
        log_warning("Symbolic Move timeout, leaving Symbolic Move mode (timer = {})", timer_);
      }
      if (fsmdata_.ctrl.in.isAtParkPosition) {
        setActiveState<ToSleepModeTransition>();
      } else {
        setActiveState<LockedMode>();
      }
    }
  } else {
    log_error("Symbolic Move can not be used when an instrument is connected. Going to locked mode");
    setActiveState<LockedMode>();
  }
  timer_ += dtSec;
}

mcx::state_machine::EventStatus SymbolicMoveMode::gotoInstrumentStraighten() {
  fsmdata_.ctrl.out.gotoInstrumentStraightenRoll = true;
  return common::gotoInstrumentStraighten(*this, fsmdata_, id());
}

mcx::state_machine::EventStatus SymbolicMoveMode::gotoSleep() {
  if (!fsmdata_.ctrl.in.symbolicMoveIsDone) {
    log_info("Symbolic Move is NOT Done, go to SLEEP mode");
  }
  setActiveState<ToSleepModeTransition>();
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus SymbolicMoveMode::gotoLocked() {
  if (!fsmdata_.ctrl.in.symbolicMoveIsDone) {
    log_info("Symbolic Move is NOT Done, go to LOCKED mode");
  }
  setActiveState<LockedMode>();
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

} // namespace logic::meril