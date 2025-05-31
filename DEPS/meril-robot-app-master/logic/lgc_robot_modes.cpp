/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_robot_modes.h"
#include "lgc_robot_modes_transitions.h"

namespace logic {

SuperMode::SuperMode(ModeFSMData& fsmdata) : fsmdata_(fsmdata) {}

SuperMode::~SuperMode() = default;

void SuperMode::registerUserEvents() {
  this->addEventName(&SuperMode::gotoInitEvent, "gotoInitEvent");
  this->addEventName(&SuperMode::gotoPauseModeEvent, "gotoPauseModeEvent");

  this->addEventName(&SuperMode::gotoManualJointModeEvent, "gotoManualJointModeEvent");
  this->addEventName(&SuperMode::gotoManualCartModeEvent, "gotoManualCartModeEvent");
  this->addEventName(&SuperMode::gotoSemiAutoModeEvent, "gotoSemiAutoModeEvent");
  this->addEventName(&SuperMode::gotoTorqueModeEvent, "gotoTorqueModeEvent");

  this->addEventName(&SuperMode::waitingPauseModeEvent, "waitingPauseModeEvent");
  this->addEventName(&SuperMode::waitingManualJointModeEvent, "waitingManualJointModeEvent");
  this->addEventName(&SuperMode::waitingManualCartModeEvent, "waitingManualCartModeEvent");
  this->addEventName(&SuperMode::waitingSemiAutoModeEvent, "waitingSemiAutoModeEvent");
  this->addEventName(&SuperMode::waitingTorqueModeEvent, "waitingTorqueModeEvent");
}

mcx::state_machine::EventStatus SuperMode::gotoInitEvent(double timeoutSec) { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMode::gotoPauseModeEvent(double timeoutSec) {
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus SuperMode::gotoManualJointModeEvent(double timeoutSec) {
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus SuperMode::gotoManualCartModeEvent(double timeoutSec) {
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus SuperMode::gotoSemiAutoModeEvent(double timeoutSec) {
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus SuperMode::gotoTorqueModeEvent(double engagedTimerSec, double gotoTorqueInitialDelay,
                                                               double timeoutSec) {
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus SuperMode::waitingPauseModeEvent() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMode::waitingManualJointModeEvent() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMode::waitingManualCartModeEvent() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMode::waitingSemiAutoModeEvent() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMode::waitingTorqueModeEvent() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMode::terminateEvent() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus BaseMode::gotoPauseModeEvent(double timeoutSec) {
  setActiveState<ToPauseModeTransition>();
  addEvent({&SuperMode::waitingPauseModeEvent}, timeoutSec);
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus BaseMode::gotoManualJointModeEvent(double timeoutSec) {
  setActiveState<PauseToManualJointModeTransition>();
  addEvent({&SuperMode::waitingManualJointModeEvent}, timeoutSec);
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus BaseMode::gotoManualCartModeEvent(double timeoutSec) {
  setActiveState<PauseToManualCartModeTransition>();
  addEvent({&SuperMode::waitingManualCartModeEvent}, timeoutSec);
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus BaseMode::gotoSemiAutoModeEvent(double timeoutSec) {
  setActiveState<PauseToSemiAutoModeTransition>();
  addEvent({&SuperMode::waitingSemiAutoModeEvent}, timeoutSec);
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus BaseMode::gotoTorqueModeEvent(double engagedTimerSec, double gotoTorqueInitialDelay,
                                                              double timeoutSec) {
  setActiveState<PauseToTorqueModeTransition>();
  auto delaySec = gotoTorqueInitialDelay - engagedTimerSec;
  if (delaySec > 0) {
    addEvent({&SuperMode::delayEvent}, delaySec);
  }
  addEvent({&SuperMode::waitingTorqueModeEvent}, timeoutSec);
  return mcx::state_machine::EVENT_DONE;
}

// Torque mode

mcx::state_machine::EventStatus TorqueMode::gotoPauseModeEvent(double timeoutSec) {
  if (allowedToSwitch()) {
    setActiveState<ToPauseModeTransition>();
    addEvent({&SuperMode::waitingPauseModeEvent}, timeoutSec);
  } else {
    log_error("Can not leave torque mode");
  }
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus TorqueMode::gotoManualJointModeEvent(double timeoutSec) {
  if (allowedToSwitch()) {
    setActiveState<PauseToManualJointModeTransition>();
    addEvent({&SuperMode::waitingManualJointModeEvent}, timeoutSec);
  } else {
    log_error("Can not leave torque mode");
  }
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus TorqueMode::gotoManualCartModeEvent(double timeoutSec) {
  if (allowedToSwitch()) {
    setActiveState<PauseToManualCartModeTransition>();
    addEvent({&SuperMode::waitingManualCartModeEvent}, timeoutSec);
  } else {
    log_error("Can not leave torque mode");
  }
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus TorqueMode::gotoSemiAutoModeEvent(double timeoutSec) {
  if (allowedToSwitch()) {
    setActiveState<PauseToSemiAutoModeTransition>();
    addEvent({&SuperMode::waitingSemiAutoModeEvent}, timeoutSec);
  } else {
    log_error("Can not leave torque mode");
  }
  return mcx::state_machine::EVENT_DONE;
}

} // namespace logic