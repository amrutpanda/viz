/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_MERIL_MODES_LOCKED_H
#define LGC_MERIL_MODES_LOCKED_H

#include "lgc_meril_modes_base.h"

namespace logic::meril {

class LockedMode final : public SuperMerilModes {
public:
  explicit LockedMode(MerilModesFSMData& data) : SuperMerilModes(data) {}
  ~LockedMode() override = default;
  void enter() override;
  void leave() override;

  mcx::state_machine::EventStatus gotoLockedForceIdleActive() override { return mcx::state_machine::EVENT_NONE; }
  mcx::state_machine::EventStatus gotoSleep() override;
  mcx::state_machine::EventStatus gotoMoveSymbolicPosition() override;
  mcx::state_machine::EventStatus gotoUnlocked() override;
  mcx::state_machine::EventStatus gotoSurgicalMode() override;
  mcx::state_machine::EventStatus gotoInstrumentExchange() override;
  mcx::state_machine::EventStatus gotoInstrumentStraighten() override;
  mcx::state_machine::EventStatus gotoInstrumentCalibrate() override;
  mcx::state_machine::EventStatus gotoAdapterCalibrate() override;
  mcx::state_machine::EventStatus gotoTeachFulcrum() override;
  mcx::state_machine::EventStatus gotoResetFulcrum() override;
  mcx::state_machine::EventStatus gotoManualInstrumentConnect() override;
  mcx::state_machine::EventStatus gotoInstrumentConnect() override;
  mcx::state_machine::EventStatus gotoInstrumentDisconnect() override;

private:
  double instrumentConnectTimerSec_{};
};

} // namespace logic::meril

#endif // LGC_MERIL_MODES_LOCKED_H
