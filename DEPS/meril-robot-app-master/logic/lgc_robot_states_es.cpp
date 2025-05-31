/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_robot_states_es.h"
#include "constants/const_errorcodes.h"
#include "lgc_robot_states.h"
#include "lgc_robot_states_data.h"

namespace logic {

void EStopResetState::enter() {
  // setStatusColor(RGB_RESETERROR);
  timer_ = smData_.timings.resetEstopSec;
  smData_.actualState = States::RESET_ESTOP_T;
}

mcx::state_machine::EventStatus EStopResetState::setNoEStopRelay() {
  smData_.bus.out.do_no_estop = true;
  smData_.ctrl.out.resetErrors = false;
  addEvent(&SuperState::delayEvent, smData_.timings.setNoEstopSec);
  addEvent(&SuperState::resetEStopRelay);
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus EStopResetState::resetEStopRelay() {

  smData_.bus.out.reset_estop = true;
  smData_.ctrl.out.resetErrors = true;

  if (timer_ >= 0) {
    timer_ -= getDtSec();
    return mcx::state_machine::EVENT_REPEAT;
  }

  smData_.bus.out.reset_estop = false;
  smData_.ctrl.out.resetErrors = false;

  addEvent(&SuperState::delayEvent, smData_.timings.resetEstopSec);
  addEvent(&SuperState::waitForEcatRecover, smData_.timings.ecatRecoverTimeoutSec);

  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus EStopResetState::waitForEcatRecover() {

  // check if estop is pressed
  if (!smData_.bus.in.estop_not_active) {
    setActiveState<EStopOpenCircState>();
    return mcx::state_machine::EVENT_DONE;
  }

  if (smData_.bus.busHasError) {
    return mcx::state_machine::EVENT_REPEAT;
  } else {
    addEvent(&SuperState::delayEvent, smData_.timings.resetSlavesSec);
    addEvent(&SuperState::resetEStopErrors);
  }

  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus EStopResetState::resetEStopErrors() {

  smData_.bus.out.reset_estop = false;
  smData_.ctrl.out.resetErrors = false;

  if (!smData_.bus.busHasError && smData_.bus.in.estop_not_active) {
    clearErrorMonitor();
    // smData_.recoveryAfterOff = true;
    setActiveState<OffState>();
    addEvent(&SuperState::delayEvent, smData_.timings.persistenceLoadDelay);       // Else 0 is read from encoder
    addEvent(&SuperState::waitPersistentData, smData_.timings.persistenceTimeout); // In case we are still saving
    addEvent(&SuperState::loadPersistentData, smData_.timings.persistenceTimeout);
    addEvent(&SuperState::waitPersistentData, smData_.timings.persistenceTimeout);
    addEvent(&SuperState::checkPersistentData, smData_.timings.persistenceTimeout);
    addEvent(&SuperState::waitPersistentData, smData_.timings.persistenceTimeout);
  } else {
    log_error("Resetting EStop Errors failed, busHasError = {}, estop_not_active = {}", smData_.bus.busHasError,
              smData_.bus.in.estop_not_active);
    setActiveState<EStopOpenCircState>();
  }

  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus EStopResetState::terminateEvent() {
  emergencyStop(ES_ESTOP_FAILED_TO_SWITCH_STATE);
  setActiveState<EStopOpenCircState>();
  return mcx::state_machine::EVENT_DONE;
}

//
void EStopOffState::enter() { smData_.actualState = States::ESTOP_OFF_S; }

mcx::state_machine::EventStatus EStopOffState::acknowledgeErrors() {
  setActiveState<EStopResetState>();
  addEvent(&SuperState::setNoEStopRelay);
  return mcx::state_machine::EVENT_DONE;
}

//

void EStopOpenCircState::enter() {
  // setStatusColor(RGB_ERROR);
  smData_.bus.out.do_no_estop = false;
  smData_.ctrl.out.gotoEngaged = false;
  if ((smData_.actualState != States::INIT_S) && (smData_.actualState != States::OFF_S) &&
      (smData_.actualState != States::RESET_ESTOP_T)) {
    addEvent(&SuperState::delayEvent, smData_.timings.persistenceSaveDelay);
    addEvent(&SuperState::savePersistentData, smData_.timings.persistenceTimeout);
  }

  smData_.actualState = States::TO_ESTOP_T;
  setActiveState<EStopOffState>();
}

} // namespace logic
