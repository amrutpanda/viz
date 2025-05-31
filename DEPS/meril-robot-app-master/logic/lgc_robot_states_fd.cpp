/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_robot_states_fd.h"
#include "constants/const_errorcodes.h"
#include "lgc_robot_states_data.h"
#include "lgc_robot_states_es.h"

namespace logic {

void ForcedIdleResetState::enter() { smData_.actualState = States::RESET_FORCEDIDLE_T; }

mcx::state_machine::EventStatus ForcedIdleResetState::resetEStopErrors() {
  smData_.ctrl.out.resetErrors = false;
  clearErrorMonitor();
  setActiveState<IdleState>();
  return mcx::state_machine::EVENT_DONE;
}

//
void ForcedIdleState::enter() { smData_.actualState = States::FORCEDIDLE_S; }

mcx::state_machine::EventStatus ForcedIdleState::acknowledgeErrors() {
  setActiveState<ForcedIdleResetState>();
  addEvent(&SuperState::resetEStopRelay); // See base class
  return mcx::state_machine::EVENT_DONE;
}

//
mcx::state_machine::EventStatus ToForcedIdleState::terminateEvent() {
  emergencyStop({ES_STATE_TRANSITION_TIMEOUT});
  return mcx::state_machine::EVENT_DONE;
}

void ToForcedIdleState::enter() { smData_.actualState = States::TO_FORCEDIDLE_T; }

mcx::state_machine::EventStatus ToForcedIdleState::waitingIdle() {
  // Note: gotoStandstill is set in maincontrolloop internally
  if (smData_.ctrl.in.isAtSmoothstop || !smData_.ctrl.in.isSmoothstopActive) {
    setActiveState<ForcedIdleState>();
    smData_.ctrl.out.syncControlLoop = true;
    return mcx::state_machine::EVENT_DONE;
  }
  return mcx::state_machine::EVENT_REPEAT;
}

} // namespace logic
