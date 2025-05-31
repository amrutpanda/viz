/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_ROBOT_MODES_H
#define LGC_ROBOT_MODES_H

#include "lgc_robot_modes_data.h"
#include "lgc_robot_states_data.h"
#include "lgc_timeout.h"
#include <mcx/core.h>

namespace logic {

class SuperMode : public mcx::state_machine::State<SuperMode> {
public:
  explicit SuperMode(ModeFSMData& fsmdata);

  ~SuperMode() override;

  using mcx::state_machine::State<SuperMode>::enter;
  using mcx::state_machine::State<SuperMode>::leave;

  void registerUserEvents() override;

  virtual mcx::state_machine::EventStatus gotoInitEvent(double timeoutSec);

  virtual mcx::state_machine::EventStatus gotoPauseModeEvent(double timeoutSec);

  virtual mcx::state_machine::EventStatus gotoManualJointModeEvent(double timeoutSec);

  virtual mcx::state_machine::EventStatus gotoManualCartModeEvent(double timeoutSec);

  virtual mcx::state_machine::EventStatus gotoSemiAutoModeEvent(double timeoutSec);

  virtual mcx::state_machine::EventStatus gotoTorqueModeEvent(double engagedTimerSec, double gotoTorqueInitialDelay,
                                                              double timeoutSec);

  virtual mcx::state_machine::EventStatus waitingPauseModeEvent();

  virtual mcx::state_machine::EventStatus waitingManualJointModeEvent();

  virtual mcx::state_machine::EventStatus waitingManualCartModeEvent();

  virtual mcx::state_machine::EventStatus waitingSemiAutoModeEvent();

  virtual mcx::state_machine::EventStatus waitingTorqueModeEvent();

  mcx::state_machine::EventStatus warning_(const mcx::state_machine::Error& error) override {
    return mcx::state_machine::EventStatus::EVENT_NONE;
  }

  mcx::state_machine::EventStatus forcedDisengaged_(const mcx::state_machine::Error& error) override {
    return mcx::state_machine::EventStatus::EVENT_NONE;
  }

  mcx::state_machine::EventStatus shutdown_(const mcx::state_machine::Error& error) override {
    return mcx::state_machine::EventStatus::EVENT_NONE;
  }

  mcx::state_machine::EventStatus emergencyStop_(const mcx::state_machine::Error& error) override {
    return mcx::state_machine::EventStatus::EVENT_NONE;
  }

  mcx::state_machine::EventStatus terminateEvent() override;

protected:
  ModeFSMData& fsmdata_;
};

class BaseMode : public SuperMode {
public:
  explicit BaseMode(ModeFSMData& fsmdata) : SuperMode(fsmdata) {};

  mcx::state_machine::EventStatus gotoPauseModeEvent(double timeoutSec) override;
  mcx::state_machine::EventStatus gotoManualJointModeEvent(double timeoutSec) override;
  mcx::state_machine::EventStatus gotoManualCartModeEvent(double timeoutSec) override;
  mcx::state_machine::EventStatus gotoSemiAutoModeEvent(double timeoutSec) override;
  mcx::state_machine::EventStatus gotoTorqueModeEvent(double engagedTimerSec, double gotoTorqueInitialDelay,
                                                      double timeoutSec) override;
};

class InitMode : public BaseMode {
public:
  explicit InitMode(ModeFSMData& fsmdata) : BaseMode(fsmdata) {};

  ~InitMode() override = default;

  void enter() override {
    fsmdata_.currentFsmModeOut = Modes::INIT_M;
    addEvent({&SuperMode::gotoPauseModeEvent, DEFAULT_TIME_OUT});
  }
};

class PauseMode : public BaseMode {
public:
  explicit PauseMode(ModeFSMData& fsmdata) : BaseMode(fsmdata) {};

  ~PauseMode() override = default;

  void enter() override { fsmdata_.currentFsmModeOut = Modes::PAUSE_M; }

  void iterate(double dtSec) override { fsmdata_.ctrl.out.gotoPauseMode = true; }

  mcx::state_machine::EventStatus gotoPauseModeEvent(double timeoutSec) override {
    return mcx::state_machine::EVENT_NONE;
  }
};

class ManualJointMode : public BaseMode {
public:
  explicit ManualJointMode(ModeFSMData& fsmdata) : BaseMode(fsmdata) {};

  ~ManualJointMode() override = default;

  void enter() override { fsmdata_.currentFsmModeOut = Modes::MANUAL_JOINT_MODE_M; }

  mcx::state_machine::EventStatus gotoManualJointModeEvent(double timeoutSec) override {
    return mcx::state_machine::EVENT_NONE;
  }
};

class ManualCartMode : public BaseMode {
public:
  explicit ManualCartMode(ModeFSMData& fsmdata) : BaseMode(fsmdata) {};

  ~ManualCartMode() override = default;

  void enter() override { fsmdata_.currentFsmModeOut = Modes::MANUAL_CART_MODE_M; }

  void iterate(double dtSec) override {
    //    if (fsmdata_.fulcrumWatchdog.active) {
    //      gotoPauseModeEvent(fsmdata_.fulcrumWatchdog.timeout);
    //    }
  }

  mcx::state_machine::EventStatus gotoManualCartModeEvent(double timeoutSec) override {
    return mcx::state_machine::EVENT_NONE;
  }
};

class SemiAutoMode : public BaseMode {
public:
  explicit SemiAutoMode(ModeFSMData& fsmdata) : BaseMode(fsmdata) {};

  ~SemiAutoMode() override = default;

  void enter() override { fsmdata_.currentFsmModeOut = Modes::SEMI_AUTO_MODE_M; }

  mcx::state_machine::EventStatus gotoSemiAutoModeEvent(double timeoutSec) override {
    return mcx::state_machine::EVENT_NONE;
  }
};

class TorqueMode : public BaseMode {
public:
  explicit TorqueMode(ModeFSMData& fsmdata) : BaseMode(fsmdata) {};

  ~TorqueMode() override = default;

  void enter() override { fsmdata_.currentFsmModeOut = Modes::TORQUE_M; }

  [[nodiscard]] bool allowedToSwitch() const {
    return !fsmdata_.torqueStateLeaveConditions.positionError && !fsmdata_.torqueStateLeaveConditions.pvaLimitActive;
  };

  mcx::state_machine::EventStatus gotoPauseModeEvent(double timeoutSec) override;

  mcx::state_machine::EventStatus gotoManualJointModeEvent(double timeoutSec) override;

  mcx::state_machine::EventStatus gotoManualCartModeEvent(double timeoutSec) override;

  mcx::state_machine::EventStatus gotoSemiAutoModeEvent(double timeoutSec) override;

  mcx::state_machine::EventStatus gotoTorqueModeEvent(double, double, double) override {
    return mcx::state_machine::EVENT_NONE;
  }
};

} // namespace logic
#endif /* LGC_ROBOT_MODES_H */
