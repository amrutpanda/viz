/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef MERIL_ROBOT_LGC_MERIL_MODES_INSTRUMENT_STRAIGHTEN_H
#define MERIL_ROBOT_LGC_MERIL_MODES_INSTRUMENT_STRAIGHTEN_H

#include "lgc_meril_modes_base.h"

namespace logic::meril {

class InstrumentStraightenMode final : public SuperMerilModes {
public:
  explicit InstrumentStraightenMode(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~InstrumentStraightenMode() override = default;
  void enter() override;
  void leave() override;
  void iterate(double dtSec) override;

private:
  double timer_{};
};

} // namespace logic::meril

#endif // MERIL_ROBOT_LGC_MERIL_MODES_INSTRUMENT_STRAIGHTEN_H
