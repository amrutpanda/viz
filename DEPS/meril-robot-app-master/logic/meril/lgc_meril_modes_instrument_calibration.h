/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_MERIL_MODES_INSTRUMENT_CALIBRATION_H
#define LGC_MERIL_MODES_INSTRUMENT_CALIBRATION_H

#include "lgc_meril_modes_base.h"

namespace logic::meril {

class InstrumentCalibrateMode final : public SuperMerilModes {
public:
  explicit InstrumentCalibrateMode(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~InstrumentCalibrateMode() override = default;
  void enter() override;
  void leave() override;
  mcx::state_machine::EventStatus calibrateInstrument() override;
  mcx::state_machine::EventStatus terminateEvent() override;
};

} // namespace logic::meril

#endif // LGC_MERIL_MODES_INSTRUMENT_CALIBRATION_H
