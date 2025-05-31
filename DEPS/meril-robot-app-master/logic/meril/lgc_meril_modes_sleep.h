/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_MERIL_MODES_SLEEP_H
#define LGC_MERIL_MODES_SLEEP_H

#include "lgc_meril_modes_base.h"

namespace logic::meril {

class ToSleepModeTransition final : public SuperMerilModes {
public:
  explicit ToSleepModeTransition(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~ToSleepModeTransition() override = default;
  void enter() override;
  mcx::state_machine::EventStatus waitingForOffState() override;
};

class SleepMode final : public SuperMerilModes {
public:
  explicit SleepMode(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~SleepMode() override = default;
  void enter() override;
  void leave() override;
  void iterate(double dtSec) override;

  mcx::state_machine::EventStatus gotoSleep() override { return mcx::state_machine::EVENT_NONE; }
  mcx::state_machine::EventStatus gotoSleepEstopActive() override { return mcx::state_machine::EVENT_NONE; }
  mcx::state_machine::EventStatus gotoLocked() override { return mcx::state_machine::EVENT_NONE; }
  mcx::state_machine::EventStatus gotoLockedForceIdleActive() override { return mcx::state_machine::EVENT_NONE; }
};

} // namespace logic::meril

#endif // LGC_MERIL_MODES_SLEEP_H
