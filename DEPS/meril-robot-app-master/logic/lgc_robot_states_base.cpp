/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_robot_states_base.h"
#include "lgc_robot_states_es.h"

namespace logic {
SuperState::SuperState(StateFSMData& data) : smData_(data) {}

SuperState::~SuperState() = default;

void SuperState::registerUserEvents() {
  this->addEventName(&SuperState::gotoOff, "gotoOff");
  this->addEventName(&SuperState::gotoEngaged, "gotoEngaged");
  this->addEventName(&SuperState::gotoReferencing, "gotoReferencing");
  this->addEventName(&SuperState::savePersistentData, "savePersistentData");
  this->addEventName(&SuperState::waitPersistentData, "waitPersistentData");
  this->addEventName(&SuperState::loadPersistentData, "loadPersistentData");
  this->addEventName(&SuperState::checkPersistentData, "checkPersistentData");

  this->addEventName(&SuperState::waitingReferencing, "waitingReferencing");
  this->addEventName(&SuperState::gotoIdle, "gotoIdle");
  this->addEventName(&SuperState::waitingEngaged, "waitingEngaged");
  this->addEventName(&SuperState::waitingIdle, "waitingIdle");
  this->addEventName(&SuperState::waitingPause, "waitingPause");
  this->addEventName(&SuperState::waitingOff, "waitingOff");
  this->addEventName(&SuperState::gotoSaveConfiguration, "gotoSaveConfiguration");
  this->addEventName(&SuperState::waitingSaveConfiguration, "waitingSaveConfiguration");

  this->addEventName(&SuperState::setNoEStopRelay, "setNoEStopRelay");
  this->addEventName(&SuperState::resetEStopRelay, "resetEStopRelay");
  this->addEventName(&SuperState::waitForEcatRecover, "waitForEcatRecover");
  this->addEventName(&SuperState::resetEStopErrors, "resetEStopErrors");
}

mcx::state_machine::EventStatus SuperState::gotoIdle(double timeoutSec) { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::waitingIdle() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::waitingPause() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::gotoOff(double timeoutSec) { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::waitingOff() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::gotoEngaged(double timeoutSec) { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::waitingEngaged() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::savePersistentData() {
  smData_.persistenceCommandOutHandle.write(mcx::control3::PersistenceEvents::SAVE);
  smData_.persistenceCommandOutHandle.updateOutputOnce();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus SuperState::waitPersistentData() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::loadPersistentData() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::checkPersistentData() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::gotoReferencing(double timeSec) { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::waitingReferencing(double timeoutSec) {
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus SuperState::gotoSaveConfiguration(const char* fileName,
                                                                  mcx::parameter_server::Parameter* root) {
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus
SuperState::waitingSaveConfiguration(const char* fileName, mcx::parameter_server::Parameter* root, int returnStateId) {
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus SuperState::setNoEStopRelay() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::resetEStopRelay() {
  smData_.ctrl.out.resetErrors = true;
  addEvent(&SuperState::delayEvent, smData_.timings.resetEstopSec);
  addEvent(&SuperState::resetEStopErrors);
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus SuperState::waitForEcatRecover() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperState::resetEStopErrors() {
  smData_.ctrl.out.resetErrors = false;
  clearErrorMonitor();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus SuperState::warning_(const mcx::state_machine::Error& error) {
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus SuperState::forcedDisengaged_(const mcx::state_machine::Error& error) {
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus SuperState::shutdown_(const mcx::state_machine::Error& error) {
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus SuperState::emergencyStop_(const mcx::state_machine::Error& error) {
  setActiveState<EStopOpenCircState>();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus SuperState::acknowledgeErrors() {
  addEvent(&SuperState::resetEStopRelay); // only reset control errors
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus SuperState::terminateEvent() { return mcx::state_machine::EVENT_NONE; }

} // namespace logic
