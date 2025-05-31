/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_meril_modes_base.h"
#include "lgc_meril_modes_locked.h"
#include "lgc_meril_modes_sleep.h"

namespace logic::meril {

SuperMerilModes::SuperMerilModes(MerilModesFSMData& fsmdata) : fsmdata_(fsmdata) {}

SuperMerilModes::~SuperMerilModes() = default;

void SuperMerilModes::registerUserEvents() {
  this->addEventName(&SuperMerilModes::gotoSleep, "gotoSleep");
  this->addEventName(&SuperMerilModes::gotoSleepEstopActive, "gotoSleepEstopActive");
  this->addEventName(&SuperMerilModes::gotoMoveSymbolicPosition, "gotoMoveSymbolicPosition");
  this->addEventName(&SuperMerilModes::gotoLocked, "gotoLocked");
  this->addEventName(&SuperMerilModes::gotoLockedForceIdleActive, "gotoLockedForceIdleActive");
  this->addEventName(&SuperMerilModes::gotoUnlocked, "gotoUnlocked");
  this->addEventName(&SuperMerilModes::waitingForZeroVelocity, "waitingForZeroVelocity");
  this->addEventName(&SuperMerilModes::gotoUnlockedInstrument, "gotoUnlockedInstrument");
  this->addEventName(&SuperMerilModes::gotoInstrumentRetract, "gotoInstrumentRetract");
  this->addEventName(&SuperMerilModes::gotoInstrumentExchange, "gotoInstrumentExchange");
  this->addEventName(&SuperMerilModes::gotoInstrumentExchangeReturn, "gotoInstrumentExchangeReturn");
  this->addEventName(&SuperMerilModes::exchangeAbort, "exchangeAbort");

  this->addEventName(&SuperMerilModes::gotoInstrumentStraighten, "gotoInstrumentStraighten");

  this->addEventName(&SuperMerilModes::gotoSurgicalMode, "gotoSurgicalMode");
  this->addEventName(&SuperMerilModes::gotoTeachFulcrum, "gotoTeachFulcrum");
  this->addEventName(&SuperMerilModes::gotoResetFulcrum, "gotoResetFulcrum");
  this->addEventName(&SuperMerilModes::waitResetFulcrum, "waitResetFulcrum");

  this->addEventName(&SuperMerilModes::gotoInstrumentConnect, "gotoInstrumentConnect");
  this->addEventName(&SuperMerilModes::gotoInstrumentDisconnect, "gotoInstrumentDisconnect");
  this->addEventName(&SuperMerilModes::gotoManualInstrumentConnect, "gotoManualInstrumentConnect");
  this->addEventName(&SuperMerilModes::gotoInstrumentCalibrate, "gotoInstrumentCalibrate");
  this->addEventName(&SuperMerilModes::gotoAdapterCalibrate, "gotoAdapterCalibrate");
  this->addEventName(&SuperMerilModes::waitingForOffState, "waitingForOffState");

  this->addEventName(&SuperMerilModes::storeRetractPosition, "storeRetractPosition");
  this->addEventName(&SuperMerilModes::alignToFulcrum, "alignToFulcrum");
  this->addEventName(&SuperMerilModes::waitExchangeInstrument, "waitExchangeInstrument");
  this->addEventName(&SuperMerilModes::updateInstrumentSettings, "updateInstrumentSettings");
  this->addEventName(&SuperMerilModes::lockedEngageExchange, "lockedEngageExchange");
  this->addEventName(&SuperMerilModes::calibrateInstrument, "calibrateInstrument");
  this->addEventName(&SuperMerilModes::calibrateAdapter, "calibrateAdapter");
  this->addEventName(&SuperMerilModes::homeInstrumentRoll, "homeInstrumentRoll");
  this->addEventName(&SuperMerilModes::cameraAlignmentAndStraighten, "cameraAlignmentAndStraighten");
  this->addEventName(&SuperMerilModes::exchangeDone, "exchangeDone");
  this->addEventName(&SuperMerilModes::enableManipulatorManualLinear, "enableManipulatorManualLinear");
  // this->addEventName(&SuperMerilModes::resetInstrumentRetract, "resetInstrumentRetract");
}

mcx::state_machine::EventStatus SuperMerilModes::storeRetractPosition() { return mcx::state_machine::EVENT_NONE; }
mcx::state_machine::EventStatus SuperMerilModes::alignToFulcrum() { return mcx::state_machine::EVENT_NONE; }
mcx::state_machine::EventStatus SuperMerilModes::waitExchangeInstrument() { return mcx::state_machine::EVENT_NONE; }
mcx::state_machine::EventStatus SuperMerilModes::updateInstrumentSettings() { return mcx::state_machine::EVENT_NONE; }
mcx::state_machine::EventStatus SuperMerilModes::lockedEngageExchange() { return mcx::state_machine::EVENT_NONE; }
mcx::state_machine::EventStatus SuperMerilModes::calibrateInstrument() { return mcx::state_machine::EVENT_NONE; }
mcx::state_machine::EventStatus SuperMerilModes::calibrateAdapter() { return mcx::state_machine::EVENT_NONE; }
mcx::state_machine::EventStatus SuperMerilModes::homeInstrumentRoll() { return mcx::state_machine::EVENT_NONE; }
mcx::state_machine::EventStatus SuperMerilModes::cameraAlignmentAndStraighten() {
  return mcx::state_machine::EVENT_NONE;
}
mcx::state_machine::EventStatus SuperMerilModes::exchangeDone() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::enableManipulatorManualLinear() {
  return mcx::state_machine::EVENT_NONE;
}

// mcx::state_machine::EventStatus SuperMerilModes::resetInstrumentRetract() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoLocked() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoLockedForceIdleActive() {
  setActiveState<LockedMode>();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus SuperMerilModes::gotoUnlocked() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::waitingForZeroVelocity() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoSleep() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoSleepEstopActive() {
  setActiveState<SleepMode>();
  return mcx::state_machine::EVENT_DONE;
}

mcx::state_machine::EventStatus SuperMerilModes::gotoMoveSymbolicPosition() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoUnlockedInstrument() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoInstrumentRetract() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoInstrumentExchange() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoInstrumentStraighten() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoTeachFulcrum() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoResetFulcrum() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::waitResetFulcrum() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoSurgicalMode() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoInstrumentConnect() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoInstrumentDisconnect() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoManualInstrumentConnect() {
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus SuperMerilModes::gotoInstrumentCalibrate() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoAdapterCalibrate() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoCameraReverseDirection() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::waitingForOffState() { return mcx::state_machine::EVENT_NONE; }

mcx::state_machine::EventStatus SuperMerilModes::gotoInstrumentExchangeReturn() {
  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus SuperMerilModes::exchangeAbort() { return mcx::state_machine::EVENT_NONE; }

} // namespace logic::meril
