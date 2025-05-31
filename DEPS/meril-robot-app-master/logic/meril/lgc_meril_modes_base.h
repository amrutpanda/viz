/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef MERIL_ROBOT_LGC_MERIL_MODES_BASE_H
#define MERIL_ROBOT_LGC_MERIL_MODES_BASE_H

#include "lgc_meril_modes_data.h"
#include <mcx/core.h>

namespace logic::meril {
class SuperMerilModes : public mcx::state_machine::State<SuperMerilModes> {
public:
  explicit SuperMerilModes(MerilModesFSMData& fsmdata);

  ~SuperMerilModes() override;

  using State::enter;
  using State::leave;

  void registerUserEvents() override;

  mcx::state_machine::EventStatus warning_(const mcx::state_machine::Error& error) override {
    return mcx::state_machine::EventStatus::EVENT_NONE;
  }

  mcx::state_machine::EventStatus forcedDisengaged_(const mcx::state_machine::Error& error) override {
    // for Meril this shall need to change: only a standstill is allowed after an error. The Robot sags when it goes to
    // forced Idle error
    return mcx::state_machine::EventStatus::EVENT_NONE;
  }

  mcx::state_machine::EventStatus shutdown_(const mcx::state_machine::Error& error) override {
    return mcx::state_machine::EventStatus::EVENT_NONE;
  }

  mcx::state_machine::EventStatus emergencyStop_(const mcx::state_machine::Error& error) override {
    return mcx::state_machine::EventStatus::EVENT_NONE;
  }

  mcx::state_machine::EventStatus terminateEvent() override { return mcx::state_machine::EventStatus::EVENT_NONE; }

  // virtual mcx::state_machine::EventStatus resetInstrumentRetract();
  virtual mcx::state_machine::EventStatus gotoLocked();
  virtual mcx::state_machine::EventStatus gotoLockedForceIdleActive();
  virtual mcx::state_machine::EventStatus gotoUnlocked();
  virtual mcx::state_machine::EventStatus waitingForZeroVelocity();
  virtual mcx::state_machine::EventStatus gotoSleep();
  virtual mcx::state_machine::EventStatus gotoSleepEstopActive();
  virtual mcx::state_machine::EventStatus gotoMoveSymbolicPosition();
  virtual mcx::state_machine::EventStatus gotoUnlockedInstrument();
  virtual mcx::state_machine::EventStatus gotoInstrumentRetract();
  virtual mcx::state_machine::EventStatus gotoInstrumentExchange();

  virtual mcx::state_machine::EventStatus storeRetractPosition();
  virtual mcx::state_machine::EventStatus alignToFulcrum();
  virtual mcx::state_machine::EventStatus waitExchangeInstrument();
  virtual mcx::state_machine::EventStatus updateInstrumentSettings();
  virtual mcx::state_machine::EventStatus lockedEngageExchange();
  virtual mcx::state_machine::EventStatus calibrateInstrument();
  virtual mcx::state_machine::EventStatus calibrateAdapter();
  virtual mcx::state_machine::EventStatus homeInstrumentRoll();
  virtual mcx::state_machine::EventStatus cameraAlignmentAndStraighten();
  virtual mcx::state_machine::EventStatus exchangeDone();
  virtual mcx::state_machine::EventStatus enableManipulatorManualLinear();

  virtual mcx::state_machine::EventStatus gotoInstrumentStraighten();

  virtual mcx::state_machine::EventStatus gotoTeachFulcrum();
  virtual mcx::state_machine::EventStatus gotoResetFulcrum();
  virtual mcx::state_machine::EventStatus waitResetFulcrum();

  virtual mcx::state_machine::EventStatus gotoSurgicalMode();
  virtual mcx::state_machine::EventStatus gotoInstrumentConnect();
  virtual mcx::state_machine::EventStatus gotoInstrumentDisconnect();
  virtual mcx::state_machine::EventStatus gotoManualInstrumentConnect();
  virtual mcx::state_machine::EventStatus gotoInstrumentCalibrate();
  virtual mcx::state_machine::EventStatus gotoAdapterCalibrate();
  virtual mcx::state_machine::EventStatus gotoCameraReverseDirection();
  virtual mcx::state_machine::EventStatus waitingForOffState();
  virtual mcx::state_machine::EventStatus gotoInstrumentExchangeReturn();
  virtual mcx::state_machine::EventStatus exchangeAbort();

  void setReturnState(int stateId) { returnStateId_ = stateId; }

protected:
  MerilModesFSMData& fsmdata_;
  int returnStateId_{-1};
};
} // namespace logic::meril

#endif // MERIL_ROBOT_LGC_MERIL_MODES_BASE_H
