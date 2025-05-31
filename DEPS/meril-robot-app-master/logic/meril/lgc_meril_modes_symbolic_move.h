/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_MERIL_MODES_SYMBOLIC_MOVE_H
#define LGC_MERIL_MODES_SYMBOLIC_MOVE_H

#include "lgc_meril_modes_base.h"

namespace logic::meril {
class SymbolicMoveMode final : public SuperMerilModes {
public:
  explicit SymbolicMoveMode(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~SymbolicMoveMode() override = default;
  void enter() override;
  void leave() override;
  void iterate(double dtSec) override;
  mcx::state_machine::EventStatus gotoLocked() override;
  mcx::state_machine::EventStatus gotoSleep() override;
  mcx::state_machine::EventStatus gotoInstrumentStraighten() override;

private:
  double timer_{};
};
} // namespace logic::meril

#endif // LGC_MERIL_MODES_SYMBOLIC_MOVE_H
