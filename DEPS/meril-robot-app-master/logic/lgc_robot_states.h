/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_ROBOT_STATES_H
#define LGC_ROBOT_STATES_H

#include "lgc_robot_states_base.h"
#include "lgc_robot_states_data.h"

namespace logic {

class InitState final : public SuperState {
public:
  explicit InitState(StateFSMData& data) : SuperState(data) {};

  ~InitState() override = default;

  void enter() override;
};

class SaveConfigurationState final : public SuperState {
  double savingTimeoutSec_{};
  std::shared_future<bool> saveFuture_;

public:
  explicit SaveConfigurationState(StateFSMData& data) : SuperState(data) {};

  ~SaveConfigurationState() override = default;

  void enter() override;

  mcx::state_machine::EventStatus waitingSaveConfiguration(const char* fileName, mcx::parameter_server::Parameter* root,
                                                           int returnStateId) override;

  mcx::state_machine::EventStatus terminateEvent() override;
};

class ReferencingState final : public SuperState {
  double referencingTimeoutSec_{};

public:
  explicit ReferencingState(StateFSMData& data) : SuperState(data) {};

  ~ReferencingState() override = default;

  void enter() override;

  void leave() override;

  mcx::state_machine::EventStatus waitingReferencing(double timeoutSec) override;

  mcx::state_machine::EventStatus terminateEvent() override;
};

class OffState final : public SuperState {
public:
  explicit OffState(StateFSMData& data) : SuperState(data) {};

  ~OffState() override = default;

  void enter() override;

  void iterate(double dtSec) override;

  void leave() override;

  mcx::state_machine::EventStatus gotoIdle(double timeoutSec) override;

  mcx::state_machine::EventStatus gotoEngaged(double timeoutSec) override;

  mcx::state_machine::EventStatus gotoReferencing(double timeSec) override;

  mcx::state_machine::EventStatus loadPersistentData() override;

  mcx::state_machine::EventStatus waitPersistentData() override;

  mcx::state_machine::EventStatus checkPersistentData() override;

  mcx::state_machine::EventStatus gotoSaveConfiguration(const char* fileName,
                                                        mcx::parameter_server::Parameter* root) override;
};

class OffToIdleState final : public SuperState {
public:
  explicit OffToIdleState(StateFSMData& data) : SuperState(data) {};

  ~OffToIdleState() override = default;

  void enter() override;

  mcx::state_machine::EventStatus waitingIdle() override;

  mcx::state_machine::EventStatus gotoOff(double timeoutSec) override;

  mcx::state_machine::EventStatus terminateEvent() override;

private:
  double timerSec_{};
};

class IdleToOffState final : public SuperState {
public:
  explicit IdleToOffState(StateFSMData& data) : SuperState(data) {};

  ~IdleToOffState() override = default;

  void enter() override;

  mcx::state_machine::EventStatus waitingOff() override;

  mcx::state_machine::EventStatus gotoIdle(double timeoutSec) override;

  mcx::state_machine::EventStatus terminateEvent() override;
};

class IdleState final : public SuperState {
public:
  explicit IdleState(StateFSMData& data) : SuperState(data) {};

  ~IdleState() override = default;

  void enter() override;

  mcx::state_machine::EventStatus gotoOff(double timeoutSec) override;

  mcx::state_machine::EventStatus gotoEngaged(double timeoutSec) override;
};

class IdleToEngagedState final : public SuperState {
public:
  explicit IdleToEngagedState(StateFSMData& data) : SuperState(data) {};

  ~IdleToEngagedState() override = default;

  void enter() override;

  mcx::state_machine::EventStatus waitingEngaged() override;

  mcx::state_machine::EventStatus gotoIdle(double timeoutSec) override;

  mcx::state_machine::EventStatus terminateEvent() override;

private:
  double timerSec_{};
};

class EngagedState final : public SuperState {
public:
  explicit EngagedState(StateFSMData& data) : SuperState(data) {};

  ~EngagedState() override = default;

  void enter() override;

  void iterate(double dtSec) override;

  mcx::state_machine::EventStatus gotoOff(double timeoutSec) override;

  mcx::state_machine::EventStatus gotoIdle(double timeoutSec) override;

  mcx::state_machine::EventStatus forcedDisengaged_(const mcx::state_machine::Error& error) override;
};

class EngagedToPausedState final : public SuperState {
public:
  explicit EngagedToPausedState(StateFSMData& data) : SuperState(data) {};

  ~EngagedToPausedState() override = default;

  void enter() override;

  mcx::state_machine::EventStatus waitingPause() override;

  mcx::state_machine::EventStatus gotoEngaged(double timeoutSec) override;

  mcx::state_machine::EventStatus terminateEvent() override;
};

class PausedToIdleState final : public SuperState {
public:
  explicit PausedToIdleState(StateFSMData& data) : SuperState(data) {};

  ~PausedToIdleState() override = default;

  void enter() override;

  mcx::state_machine::EventStatus waitingIdle() override;

  mcx::state_machine::EventStatus gotoEngaged(double timeoutSec) override;

  mcx::state_machine::EventStatus terminateEvent() override;
};

}; // namespace logic

#endif /* LGC_ROBOT_STATES_H */
