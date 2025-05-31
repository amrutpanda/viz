/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_ROBOT_STATES_FD_H
#define LGC_ROBOT_STATES_FD_H

#include "lgc_robot_states.h"
#include "lgc_robot_states_data.h"

namespace logic {

class ForcedIdleSuperState : public SuperState {
protected:
  explicit ForcedIdleSuperState(StateFSMData& data) : SuperState(data){};

  mcx::state_machine::EventStatus warning_(const mcx::state_machine::Error& error) override {
    return mcx::state_machine::EventStatus::EVENT_NONE;
  }

  mcx::state_machine::EventStatus forcedIdle_(const mcx::state_machine::Error& error) {
    return mcx::state_machine::EventStatus::EVENT_NONE;
  }

  mcx::state_machine::EventStatus acknowledgeErrors() override { return mcx::state_machine::EventStatus::EVENT_NONE; }

  mcx::state_machine::EventStatus terminateEvent() override { return mcx::state_machine::EventStatus::EVENT_NONE; }
};

class ForcedIdleResetState final : public ForcedIdleSuperState {
public:
  explicit ForcedIdleResetState(StateFSMData& data) : ForcedIdleSuperState(data){};

  ~ForcedIdleResetState() override = default;

  void enter() override;

  mcx::state_machine::EventStatus resetEStopErrors() override;
};

class ForcedIdleState final : public ForcedIdleSuperState {
public:
  explicit ForcedIdleState(StateFSMData& data) : ForcedIdleSuperState(data){};

  ~ForcedIdleState() override = default;

  void enter() override;

  mcx::state_machine::EventStatus acknowledgeErrors() override;
};

class ToForcedIdleState final : public ForcedIdleSuperState {
public:
  explicit ToForcedIdleState(StateFSMData& data) : ForcedIdleSuperState(data){};

  ~ToForcedIdleState() override = default;

  void enter() override;

  mcx::state_machine::EventStatus waitingIdle() override;

  mcx::state_machine::EventStatus terminateEvent() override;
};
} // namespace logic

#endif /* LGC_ROBOT_STATES_FD_H */
