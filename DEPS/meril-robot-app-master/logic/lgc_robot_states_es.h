/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_ROBOT_STATES_ES_H
#define LGC_ROBOT_STATES_ES_H

#include "lgc_robot_states_base.h"
#include "lgc_robot_states_data.h"

namespace logic {

class EStopSuperState : public SuperState {
protected:
  explicit EStopSuperState(StateFSMData& data) : SuperState(data){};

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
    // Trigger only new errors
    if (getErrorMonitor()->hasError(error)) {
      return mcx::state_machine::EVENT_NONE;
    }
    return mcx::state_machine::EVENT_DONE;
  }

  mcx::state_machine::EventStatus acknowledgeErrors() override { return mcx::state_machine::EventStatus::EVENT_NONE; }

  mcx::state_machine::EventStatus terminateEvent() override { return mcx::state_machine::EventStatus::EVENT_NONE; }
};

class EStopResetState final : public EStopSuperState {
public:
  explicit EStopResetState(StateFSMData& data) : EStopSuperState(data){};

  ~EStopResetState() override = default;
  ;

  void enter() override;

  mcx::state_machine::EventStatus setNoEStopRelay() override;

  mcx::state_machine::EventStatus resetEStopRelay() override;

  mcx::state_machine::EventStatus waitForEcatRecover() override;

  mcx::state_machine::EventStatus resetEStopErrors() override;

  mcx::state_machine::EventStatus terminateEvent() override;

private:
  double timer_{};
};

class EStopOffState final : public EStopSuperState {
public:
  explicit EStopOffState(StateFSMData& data) : EStopSuperState(data){};

  ~EStopOffState() override = default;
  ;

  void enter() override;

  mcx::state_machine::EventStatus acknowledgeErrors() override;
};

class EStopOpenCircState final : public EStopSuperState {
public:
  explicit EStopOpenCircState(StateFSMData& data) : EStopSuperState(data){};

  ~EStopOpenCircState() override = default;
  ;

  void enter() override;
};

} // namespace logic

#endif /* LGC_ROBOT_STATES_ES_H */
