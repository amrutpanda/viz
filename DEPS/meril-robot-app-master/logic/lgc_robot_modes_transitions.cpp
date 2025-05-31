/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_robot_modes_transitions.h"
#include "constants/const_errorcodes.h"

#define DO_PAUSE_SWITCH_OFF                                                                                            \
  {                                                                                                                    \
    fsmdata_.ctrl.out.gotoPauseMode = false;                                                                           \
    const bool pause_state_off = fsmdata_.ctrl.in.pauseModeSwitchOff;                                                  \
    if (!pause_state_off) {                                                                                            \
      return mcx::state_machine::EVENT_REPEAT;                                                                         \
    }                                                                                                                  \
  };

namespace logic {

// PauseToSemiAutoTransition

void PauseToSemiAutoModeTransition::enter() { fsmdata_.currentFsmModeOut = Modes::PAUSE_TO_SEMI_AUTO_T; }

mcx::state_machine::EventStatus PauseToSemiAutoModeTransition::waitingSemiAutoModeEvent() {

  // switch pause on
  // switch move_to_start on
  fsmdata_.ctrl.out.gotoPauseMode = true;
  fsmdata_.ctrl.out.gotoManualMode = false;
  fsmdata_.ctrl.out.gotoSemiautoMode = true;
  const bool semiautoStateOn = (fsmdata_.ctrl.in.manualModeSwitchOff && fsmdata_.ctrl.in.semiautoModeSwitchOn);
  if (!semiautoStateOn) {
    return mcx::state_machine::EVENT_REPEAT;
  }
  // switch pause off
  DO_PAUSE_SWITCH_OFF;

  setActiveState<SemiAutoMode>();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus PauseToSemiAutoModeTransition::terminateEvent() {
  log_error("Failed to complete transition from Pause to SemiAuto Mode");
  warning({WA_FAILED_TO_SWITCH_SEMIAUTO_MODE});
  setActiveState<PauseMode>();
  return mcx::state_machine::EVENT_DONE;
}

// PauseToManualJointModeTransition

void PauseToManualJointModeTransition::enter() { fsmdata_.currentFsmModeOut = Modes::PAUSE_TO_MANUAL_JOINT_T; }

mcx::state_machine::EventStatus PauseToManualJointModeTransition::waitingManualJointModeEvent() {
  // switch pause on
  // switch manual and joint on
  fsmdata_.ctrl.out.gotoPauseMode = true;
  fsmdata_.ctrl.out.gotoManualMode = true;
  fsmdata_.ctrl.out.gotoJointMode = true;
  fsmdata_.ctrl.out.gotoSemiautoMode = false;
  const bool manualJointOn = (fsmdata_.ctrl.in.manualModeSwitchOn && fsmdata_.ctrl.in.jointModeSwitchOn &&
                              fsmdata_.ctrl.in.semiautoModeSwitchOff);
  if (!manualJointOn) {
    return mcx::state_machine::EVENT_REPEAT;
  }
  // switch pause off
  DO_PAUSE_SWITCH_OFF;

  setActiveState<ManualJointMode>();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus PauseToManualJointModeTransition::terminateEvent() {
  log_error("Failed to complete transition from Pause to Manual Joint Mode");
  warning({WA_FAILED_TO_SWITCH_JOINT_MODE});
  setActiveState<PauseMode>();
  return mcx::state_machine::EVENT_DONE;
}

// PauseToManualCartModeTransition
void PauseToManualCartModeTransition::enter() { fsmdata_.currentFsmModeOut = Modes::PAUSE_TO_MANUAL_CART_T; }

mcx::state_machine::EventStatus PauseToManualCartModeTransition::waitingManualCartModeEvent() {
  // switch pause on
  // switch manual and joint on
  fsmdata_.ctrl.out.gotoPauseMode = true;
  fsmdata_.ctrl.out.gotoManualMode = true;
  fsmdata_.ctrl.out.gotoJointMode = false;
  fsmdata_.ctrl.out.gotoSemiautoMode = false;
  const bool manualCartOn = (fsmdata_.ctrl.in.manualModeSwitchOn && fsmdata_.ctrl.in.jointModeSwitchOff &&
                             fsmdata_.ctrl.in.semiautoModeSwitchOff);
  if (!manualCartOn) {
    return mcx::state_machine::EVENT_REPEAT;
  }
  // switch pause off
  DO_PAUSE_SWITCH_OFF;

  setActiveState<ManualCartMode>();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus PauseToManualCartModeTransition::terminateEvent() {
  log_error("Failed to complete transition from Pause to Manual Cart Mode");
  warning({WA_FAILED_TO_SWITCH_CARTESIAN_MODE});
  setActiveState<PauseMode>();
  return mcx::state_machine::EVENT_DONE;
}

// PauseToTorqueModeTransition
void PauseToTorqueModeTransition::enter() { fsmdata_.currentFsmModeOut = Modes::PAUSE_TO_TORQUE_T; }

mcx::state_machine::EventStatus PauseToTorqueModeTransition::waitingTorqueModeEvent() {
  // switch pause on
  // switch torque mode
  fsmdata_.ctrl.out.gotoPauseMode = true;
  fsmdata_.ctrl.out.gotoManualMode = true;
  fsmdata_.ctrl.out.gotoJointMode = true;
  const bool ready = fsmdata_.ctrl.in.manualModeSwitchOn && fsmdata_.ctrl.in.jointModeSwitchOn;
  if (!ready) {
    return mcx::state_machine::EVENT_REPEAT;
  } // switch pause off
  DO_PAUSE_SWITCH_OFF;

  fsmdata_.ctrl.out.drivesAreInTorqueMode = true;

  setActiveState<TorqueMode>();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus PauseToTorqueModeTransition::terminateEvent() {
  log_error("Failed to complete transition from Pause to Torque Mode");
  warning({WA_FAILED_TO_SWITCH_TORQUE_MODE});
  setActiveState<PauseMode>();
  return mcx::state_machine::EVENT_DONE;
}

// ToPauseModeTransition
void ToPauseModeTransition::enter() {}

void ToPauseModeTransition::iterate(double dtSec) {}

mcx::state_machine::EventStatus ToPauseModeTransition::waitingPauseModeEvent() {
  fsmdata_.ctrl.out.gotoPauseMode = true;
  const auto pauseStateOn = fsmdata_.ctrl.in.pauseModeSwitchOn;
  if (!pauseStateOn) {
    return mcx::state_machine::EVENT_REPEAT;
  }

  // should wait for position sync. We need to know that the sync is completed (that target = actual)
  // and only then we can switch torqueMode to positionMode.
  fsmdata_.ctrl.out.drivesAreInTorqueMode = false;

  setActiveState<PauseMode>();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus ToPauseModeTransition::terminateEvent() {
  log_error("Transition to pause mode takes too long, switching anyways");
  warning({WA_FAILED_TO_SWITCH_PAUSE_MODE});
  setActiveState<PauseMode>();
  return mcx::state_machine::EVENT_DONE;
}

} // namespace logic
