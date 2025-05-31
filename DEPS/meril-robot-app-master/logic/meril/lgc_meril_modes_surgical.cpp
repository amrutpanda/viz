/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_meril_modes_surgical.h"
#include "lgc_meril_modes_common.h"
#include "lgc_meril_modes_instrument_straighten.h"
#include "lgc_meril_modes_locked.h"

namespace logic::meril {

void SurgicalMode::enter() {
  fsmdata_.ctrl.out.enableStopAtRetractPosition = false;
  fsmdata_.ctrl.out.enableStopAtCalibrationPosition = false;
  fsmdata_.currentFsmModeOut = MerilModes::SURGICAL_M;
}

void SurgicalMode::iterate(double dtSec) {
  if (!fsmdata_.ctrl.in.surgicalModeIsAllowed) {
    // use surgical mode is allowed since it depends on the watchdog. fulcrum should be
    // invalid for longer duration to leave surgical mode.
    log_error("Surgical Mode is No longer allowed, leaving surgical mode");
    setActiveState<LockedMode>();
  }
}

mcx::state_machine::EventStatus SurgicalMode::gotoLocked() {
  setActiveState<LockedMode>();
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus SurgicalMode::gotoInstrumentStraighten() {
  fsmdata_.ctrl.out.gotoInstrumentStraightenRoll = true; // only for surgical mode
  return common::gotoInstrumentStraighten(*this, fsmdata_, id());
  ;
}

mcx::state_machine::EventStatus SurgicalMode::gotoUnlocked() { return common::gotoUnlocked(*this, fsmdata_, id()); }

mcx::state_machine::EventStatus SurgicalMode::gotoCameraReverseDirection() {
  if (fsmdata_.ctrl.in.isCameraRobot) {
    fsmdata_.ctrl.out.cameraDirectionIsReversed = !fsmdata_.ctrl.out.cameraDirectionIsReversed;
    if (fsmdata_.ctrl.out.cameraDirectionIsReversed) {
      log_info("Camera direction is reversed");
    } else {
      log_info("Camera direction is not reversed");
    }
  }
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

} // namespace logic::meril