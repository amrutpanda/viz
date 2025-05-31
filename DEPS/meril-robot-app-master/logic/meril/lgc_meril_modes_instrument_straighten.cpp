/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_meril_modes_instrument_straighten.h"
#include "lgc_meril_modes_locked.h"

namespace logic::meril {

void InstrumentStraightenMode::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::INSTRUMENT_STRAIGHTEN_M;
  fsmdata_.ctrl.out.gotoInstrumentStraighten = true;
  timer_ = 0;
  returnStateId_ = -1;
}

void InstrumentStraightenMode::leave() {
  fsmdata_.ctrl.out.gotoInstrumentStraighten = false;
  fsmdata_.ctrl.out.gotoInstrumentStraightenRoll = false;
}

void InstrumentStraightenMode::iterate(double dtSec) {
  if (fsmdata_.ctrl.in.instrumentStraightenDone) {
    log_info("Straighten is Done, leaving Straighten mode");
    if (returnStateId_ < 0) {
      log_error("InstrumentStraightenMode: Return state is not set, switching to LockedMode");
      setActiveState<LockedMode>();
    } else {
      setActiveState(returnStateId_);
    }
  } else if (timer_ > fsmdata_.instrumentStraighteningTimeoutSec) {
    log_warning("Straighten timeout, leaving Straighten mode (timer = {})", timer_);
    if (returnStateId_ < 0) {
      log_error("InstrumentStraightenMode: Return state is not set, switching to LockedMode");
      setActiveState<LockedMode>();
    } else {
      setActiveState(returnStateId_);
    }
  }
  timer_ += dtSec;
}

} // namespace logic::meril
