/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_MERIL_MODES_SURGICAL_H
#define LGC_MERIL_MODES_SURGICAL_H

#include "lgc_meril_modes_base.h"

namespace logic::meril {

class SurgicalMode final : public SuperMerilModes {
public:
  explicit SurgicalMode(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~SurgicalMode() override = default;
  void enter() override;
  void iterate(double dtSec) override;
  mcx::state_machine::EventStatus gotoLocked() override;
  mcx::state_machine::EventStatus gotoInstrumentStraighten() override;
  mcx::state_machine::EventStatus gotoUnlocked() override;
  mcx::state_machine::EventStatus gotoCameraReverseDirection() override;
};

} // namespace logic::meril

#endif // LGC_MERIL_MODES_SURGICAL_H
