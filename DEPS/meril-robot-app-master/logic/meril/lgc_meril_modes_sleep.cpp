/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_meril_modes_sleep.h"
#include "lgc_meril_modes_common.h"
#include "lgc_meril_modes_locked.h"

namespace logic::meril {

void ToSleepModeTransition::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::TO_SLEEP_T;
  addEvent({&SuperMerilModes::waitingForOffState});
}

mcx::state_machine::EventStatus ToSleepModeTransition::waitingForOffState() {
  if (fsmdata_.actualRobotState == States::OFF_S || fsmdata_.actualRobotState == States::ESTOP_OFF_S) {
    setActiveState<SleepMode>();
    return mcx::state_machine::EventStatus::EVENT_DONE;
  }
  return mcx::state_machine::EventStatus::EVENT_REPEAT;
}

void SleepMode::enter() {
  fsmdata_.ctrl.out.updateJointLimits = true;
  fsmdata_.currentFsmModeOut = MerilModes::SLEEP_M;
}

void SleepMode::leave() { fsmdata_.ctrl.out.updateJointLimits = false; }

void SleepMode::iterate(double dtSec) {
  if (fsmdata_.actualRobotState == States::ENGAGED_S) {
    setActiveState<LockedMode>();
  }
}

} // namespace logic::meril