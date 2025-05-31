/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_robot_states.h"
#include "constants/const_errorcodes.h"
#include "lgc_robot_states_data.h"
#include "lgc_robot_states_fd.h"

namespace logic {

void InitState::enter() {
  smData_.actualState = States::INIT_S;
  // Initial state of switches at startup:
  smData_.ctrl.out.gotoEngaged = false;
  emergencyStop({ES_ESTOP_INPUT_OPENED});
  addEvent(&SuperState::delayEvent, 0.1);
  addEvent(&SuperState::acknowledgeErrors);
}

//

void SaveConfigurationState::enter() { savingTimeoutSec_ = 0; }

mcx::state_machine::EventStatus SaveConfigurationState::waitingSaveConfiguration(const char* fileName,
                                                                                 mcx::parameter_server::Parameter* root,
                                                                                 int returnStateId) {

  // run async save at first iterate
  if (savingTimeoutSec_ == 0) {
    saveFuture_ = smData_.fileSerialization.save(fileName, root, 10, 10, false);
  }
  // check if save is complete
  if (saveFuture_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    auto dataSaved = saveFuture_.get();
    // check result of the save
    if (!dataSaved) {
      log_error("Failed to save parameter file: {}", fileName);
      warning({WA_CONFIGURATION_ERROR});
    } else {
      log_info("Saved configuration to parameter file: {}", fileName);
    }
    setActiveState(returnStateId);
    return mcx::state_machine::EVENT_DONE;
  }
  // increment the timers
  savingTimeoutSec_ += getDtSec();
  // wait until save is not resolved
  return mcx::state_machine::EVENT_REPEAT;
}

mcx::state_machine::EventStatus SaveConfigurationState::terminateEvent() {
  emergencyStop({ES_SAVE_CONFIGURATION_TIMEOUT});
  return mcx::state_machine::EVENT_DONE;
}

//
void ReferencingState::enter() {
  referencingTimeoutSec_ = 0;
  smData_.referencing.out.setHardwareSnapshot = true;
  smData_.referencing.out.setHardwareReference = true;
}

void ReferencingState::leave() {
  smData_.referencing.out.setHardwareSnapshot = false;
  smData_.referencing.out.setHardwareReference = false;
}

mcx::state_machine::EventStatus ReferencingState::waitingReferencing(double timeoutSec) {

  if (referencingTimeoutSec_ <= timeoutSec) {
    referencingTimeoutSec_ += getDtSec();
    return mcx::state_machine::EVENT_REPEAT;
  }

  setActiveState<OffState>();

  // Save the data and check it after saving is complete
  addEvent(&SuperState::savePersistentData, smData_.timings.persistenceTimeout);
  addEvent(&SuperState::waitPersistentData, smData_.timings.persistenceTimeout);
  addEvent(&SuperState::checkPersistentData, smData_.timings.persistenceTimeout);
  addEvent(&SuperState::waitPersistentData, smData_.timings.persistenceTimeout);

  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus ReferencingState::terminateEvent() {
  log_error("Failed to switch from Reference to Off");
  emergencyStop({ES_ESTOP_FAILED_TO_SWITCH_STATE, (uint32_t)id()});
  return mcx::state_machine::EVENT_DONE;
}

//

void OffState::enter() {
  // setStatusColor(RGB_IDLE);
  if ((smData_.actualState != States::RESET_ESTOP_T) and (smData_.actualState != States::OFF_S)) {
    addEvent(&SuperState::savePersistentData, smData_.timings.persistenceTimeout);
  }
  smData_.actualState = States::OFF_S;
}

void OffState::iterate(double dtSec) {
  smData_.ctrl.out.resetErrors = !smData_.ctrl.out.resetErrors; // Toggle while in off state, set to false on leave
}

void OffState::leave() { smData_.ctrl.out.resetErrors = false; }

mcx::state_machine::EventStatus OffState::loadPersistentData() {
  smData_.persistenceCommandOutHandle.write(mcx::control3::PersistenceEvents::LOAD);
  smData_.persistenceCommandOutHandle.updateOutputOnce();

  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus OffState::waitPersistentData() { return mcx::state_machine::EVENT_DONE; }

mcx::state_machine::EventStatus OffState::checkPersistentData() {
  smData_.persistenceCommandOutHandle.write(mcx::control3::PersistenceEvents::CHECK);
  smData_.persistenceCommandOutHandle.updateOutputOnce();

  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus OffState::gotoIdle(double timeoutSec) {
  if (smData_.ctrl.in.isReferenced) {
    setActiveState<OffToIdleState>();
    addEvent(&SuperState::waitingIdle, timeoutSec);
    return mcx::state_machine::EVENT_DONE;
  } else {
    emergencyStop({ES_NOT_REFERENCED});
    log_warning("Trying to goto Idle without referencing");
  }
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus OffState::gotoEngaged(double timeoutSec) {
  if (gotoIdle(timeoutSec) == mcx::state_machine::EVENT_DONE) {
    addEvent({&SuperState::gotoEngaged, timeoutSec});
  }
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus OffState::gotoReferencing(double timeSec) {

  setActiveState<ReferencingState>();
  addEvent({&SuperState::waitingReferencing, timeSec}, smData_.timings.referenceTimeSec);

  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus OffState::gotoSaveConfiguration(const char* fileName,
                                                                mcx::parameter_server::Parameter* root) {
  setActiveState<SaveConfigurationState>();
  addEvent({&SuperState::waitingSaveConfiguration, fileName, root, OffState::id()}, smData_.timings.saveTimeSec);

  return mcx::state_machine::EVENT_DONE;
}

//

void OffToIdleState::enter() {
  smData_.actualState = States::OFF_TO_IDLE_T;
  timerSec_ = 0;
}

mcx::state_machine::EventStatus OffToIdleState::waitingIdle() {
  // Delay to allow brakes to release
  timerSec_ += getDtSec();
  if (timerSec_ >= smData_.timings.gotoIdleDelaySec) {
    smData_.ctrl.out.gotoEngaged = true;
    if (smData_.ctrl.in.isAtEngaged) {
      setActiveState<IdleState>();
      return mcx::state_machine::EVENT_DONE;
    }
  }
  return mcx::state_machine::EVENT_REPEAT;
}

mcx::state_machine::EventStatus OffToIdleState::gotoOff(double timeoutSec) {
  setActiveState<OffToIdleState>();
  addEvent(&SuperState::waitingIdle, timeoutSec);
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus OffToIdleState::terminateEvent() {
  log_error("Failed to switch from Off to Idle");
  emergencyStop({ES_OFF_TO_IDLE_TIMEOUT});
  return mcx::state_machine::EVENT_DONE;
}

//

void IdleToOffState::enter() { smData_.actualState = States::IDLE_TO_OFF_T; }

mcx::state_machine::EventStatus IdleToOffState::waitingOff() {
  smData_.ctrl.out.gotoEngaged = false;
  if (smData_.ctrl.in.isAtIdle) {
    setActiveState<OffState>();
    return mcx::state_machine::EVENT_DONE;
  }
  return mcx::state_machine::EVENT_REPEAT;
}

mcx::state_machine::EventStatus IdleToOffState::gotoIdle(double timeoutSec) {
  if (smData_.ctrl.in.isReferenced) {
    setActiveState<OffToIdleState>();
    addEvent(&SuperState::waitingIdle, timeoutSec);
  } else {
    log_warning("Trying to goto disengaged without referencing");
  }

  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus IdleToOffState::terminateEvent() {
  log_error("Failed to switch from Idle to Off");
  emergencyStop({IDLE_TO_OFF_TIMEOUT});
  return mcx::state_machine::EVENT_DONE;
}

//

void IdleState::enter() { smData_.actualState = States::IDLE_S; }

mcx::state_machine::EventStatus IdleState::gotoOff(double timeoutSec) {
  setActiveState<IdleToOffState>();
  addEvent(&SuperState::waitingOff, timeoutSec);
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus IdleState::gotoEngaged(double timeoutSec) {
  setActiveState<IdleToEngagedState>();
  addEvent(&SuperState::waitingEngaged, timeoutSec);
  return mcx::state_machine::EVENT_DONE;
}

//

void IdleToEngagedState::enter() {
  smData_.actualState = States::IDLE_TO_ENGAGED_T;
  timerSec_ = 0;
}

mcx::state_machine::EventStatus IdleToEngagedState::waitingEngaged() {
  setActiveState<EngagedState>();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus IdleToEngagedState::gotoIdle(double timeoutSec) {
  setActiveState<PausedToIdleState>();
  addEvent(&SuperState::waitingIdle, timeoutSec);
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus IdleToEngagedState::terminateEvent() {
  log_error("Failed to switch from Idle to Engaged");
  emergencyStop({ES_ESTOP_FAILED_TO_SWITCH_STATE, static_cast<uint32_t>(id())});
  return mcx::state_machine::EVENT_DONE;
}

//
void EngagedState::enter() { smData_.actualState = States::ENGAGED_S; }

void EngagedState::iterate(double dtSec) { smData_.engagedStateTimer += dtSec; }

mcx::state_machine::EventStatus EngagedState::gotoOff(double timeoutSec) {
  gotoIdle(timeoutSec);
  addEvent({&SuperState::gotoOff, timeoutSec});
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus EngagedState::gotoIdle(double timeoutSec) {
  setActiveState<EngagedToPausedState>();
  addEvent(&SuperState::waitingPause, smData_.timings.waitingPauseSec);
  addEvent(&SuperState::waitingIdle, smData_.timings.waitingIdleSec);
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus EngagedState::forcedDisengaged_(const mcx::state_machine::Error& error) {
  setActiveState<ToForcedIdleState>();
  addEvent(&SuperState::waitingIdle, smData_.timings.waitingIdleSec);
  return mcx::state_machine::EVENT_DONE;
}

//

void EngagedToPausedState::enter() { smData_.actualState = States::ENGAGED_TO_PAUSED_T; }

mcx::state_machine::EventStatus EngagedToPausedState::waitingPause() {
  if (smData_.ctrl.in.isAtPause) {
    setActiveState<PausedToIdleState>();
    return mcx::state_machine::EVENT_DONE;
  }
  return mcx::state_machine::EVENT_REPEAT;
}

mcx::state_machine::EventStatus EngagedToPausedState::gotoEngaged(double timeoutSec) {
  setActiveState<EngagedState>();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus EngagedToPausedState::terminateEvent() {
  setActiveState<PausedToIdleState>();
  return mcx::state_machine::EVENT_DONE;
}

//

void PausedToIdleState::enter() { smData_.actualState = States::PAUSED_TO_IDLE_T; }

mcx::state_machine::EventStatus PausedToIdleState::waitingIdle() {
  setActiveState<IdleState>();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus PausedToIdleState::gotoEngaged(double timeoutSec) {
  setActiveState<IdleToEngagedState>();
  addEvent(&SuperState::waitingEngaged, timeoutSec);
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus PausedToIdleState::terminateEvent() {
  log_error("Failed to switch from Pause to Idle");
  emergencyStop({ES_ESTOP_FAILED_TO_SWITCH_STATE, static_cast<uint32_t>(id())});
  return mcx::state_machine::EVENT_DONE;
}

} // namespace logic
