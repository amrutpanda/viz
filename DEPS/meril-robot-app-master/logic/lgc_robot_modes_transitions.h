/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef MOTORCORTEX_ROBOT_LGC_ROBOT_MODES_TRANSITIONS_H
#define MOTORCORTEX_ROBOT_LGC_ROBOT_MODES_TRANSITIONS_H

#include "lgc_robot_modes.h"

namespace logic {

class PauseToSemiAutoModeTransition : public BaseMode {
public:
  explicit PauseToSemiAutoModeTransition(ModeFSMData& fsmdata) : BaseMode(fsmdata) {};

  ~PauseToSemiAutoModeTransition() override = default;

  void enter() override;

  mcx::state_machine::EventStatus waitingSemiAutoModeEvent() override;

  mcx::state_machine::EventStatus terminateEvent() override;

  mcx::state_machine::EventStatus gotoSemiAutoModeEvent(double timeoutSec) override {
    return mcx::state_machine::EVENT_NONE;
  }
};

class PauseToManualJointModeTransition : public BaseMode {
public:
  explicit PauseToManualJointModeTransition(ModeFSMData& fsmdata) : BaseMode(fsmdata) {};

  ~PauseToManualJointModeTransition() override = default;

  void enter() override;

  mcx::state_machine::EventStatus waitingManualJointModeEvent() override;

  mcx::state_machine::EventStatus terminateEvent() override;

  mcx::state_machine::EventStatus gotoManualJointModeEvent(double timeoutSec) override {
    return mcx::state_machine::EVENT_NONE;
  }
};

class PauseToManualCartModeTransition : public BaseMode {
public:
  explicit PauseToManualCartModeTransition(ModeFSMData& fsmdata) : BaseMode(fsmdata) {};

  ~PauseToManualCartModeTransition() override = default;

  void enter() override;

  mcx::state_machine::EventStatus waitingManualCartModeEvent() override;

  mcx::state_machine::EventStatus terminateEvent() override;

  mcx::state_machine::EventStatus gotoManualCartModeEvent(double timeoutSec) override {
    return mcx::state_machine::EVENT_NONE;
  }
};

class PauseToTorqueModeTransition : public BaseMode {
public:
  explicit PauseToTorqueModeTransition(ModeFSMData& fsmdata) : BaseMode(fsmdata) {};

  ~PauseToTorqueModeTransition() override = default;

  void enter() override;

  mcx::state_machine::EventStatus waitingTorqueModeEvent() override;

  mcx::state_machine::EventStatus terminateEvent() override;

  mcx::state_machine::EventStatus gotoTorqueModeEvent(double, double, double) override {
    return mcx::state_machine::EVENT_NONE;
  }
  mcx::state_machine::EventStatus gotoPauseModeEvent(double timeoutSec) override {
    return mcx::state_machine::EVENT_NONE;
  }
};

class ToPauseModeTransition : public BaseMode {
public:
  explicit ToPauseModeTransition(ModeFSMData& fsmdata) : BaseMode(fsmdata) {};

  ~ToPauseModeTransition() override = default;

  void enter() override;

  void iterate(double dtSec) override;

  mcx::state_machine::EventStatus waitingPauseModeEvent() override;

  mcx::state_machine::EventStatus terminateEvent() override;

  mcx::state_machine::EventStatus gotoPauseModeEvent(double timeoutSec) override {
    return mcx::state_machine::EVENT_NONE;
  }
  mcx::state_machine::EventStatus gotoTorqueModeEvent(double, double, double) override {
    return mcx::state_machine::EVENT_NONE;
  }
};

} // namespace logic

#endif // MOTORCORTEX_ROBOT_LGC_ROBOT_MODES_TRANSITIONS_H
