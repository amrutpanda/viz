/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_MERIL_MODES_UNLOCKED_H
#define LGC_MERIL_MODES_UNLOCKED_H

#include "lgc_meril_modes_base.h"

namespace logic::meril {

class TransitionToLockedMode final : public SuperMerilModes {
public:
  explicit TransitionToLockedMode(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~TransitionToLockedMode() override = default;
  void enter() override;
  void leave() override;
  mcx::state_machine::EventStatus waitingForZeroVelocity() override;
  mcx::state_machine::EventStatus terminateEvent() override;
};

class UnlockedMode final : public SuperMerilModes {
public:
  explicit UnlockedMode(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~UnlockedMode() override = default;
  void enter() override;
  void leave() override;

  mcx::state_machine::EventStatus gotoUnlocked() override;
  mcx::state_machine::EventStatus gotoLocked() override;
  mcx::state_machine::EventStatus gotoTeachFulcrum() override;
};

class UnlockedInstrumentMode final : public SuperMerilModes {
public:
  explicit UnlockedInstrumentMode(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~UnlockedInstrumentMode() override = default;
  void enter() override;
  void leave() override;
  mcx::state_machine::EventStatus gotoUnlocked() override;
  mcx::state_machine::EventStatus gotoLocked() override;
  mcx::state_machine::EventStatus enableManipulatorManualLinear() override;
  mcx::state_machine::EventStatus gotoInstrumentStraighten() override;
  mcx::state_machine::EventStatus gotoSurgicalMode() override;
};

class ResetFulcrumUnlockedMode final : public SuperMerilModes {
public:
  explicit ResetFulcrumUnlockedMode(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~ResetFulcrumUnlockedMode() override = default;
  void enter() override;
  void leave() override;
  mcx::state_machine::EventStatus gotoUnlocked() override;
  mcx::state_machine::EventStatus terminateEvent() override;
};

} // namespace logic::meril

#endif // LGC_MERIL_MODES_UNLOCKED_H
