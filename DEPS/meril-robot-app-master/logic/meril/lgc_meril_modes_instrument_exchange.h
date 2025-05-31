/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_MERIL_MODES_EXCHANGE_INSTRUMENT_H
#define LGC_MERIL_MODES_EXCHANGE_INSTRUMENT_H

#include "lgc_meril_modes_base.h"
#include "lgc_meril_modes_data.h"

namespace logic::meril {

class InstrumentExchangeMode final : public SuperMerilModes {
public:
  explicit InstrumentExchangeMode(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~InstrumentExchangeMode() override = default;
  void enter() override;
  void leave() override;

  mcx::state_machine::EventStatus storeRetractPosition() override;
  mcx::state_machine::EventStatus alignToFulcrum() override;
  mcx::state_machine::EventStatus waitExchangeInstrument() override;
  mcx::state_machine::EventStatus updateInstrumentSettings() override;
  mcx::state_machine::EventStatus lockedEngageExchange() override;
  mcx::state_machine::EventStatus calibrateInstrument() override;
  mcx::state_machine::EventStatus homeInstrumentRoll() override;
  mcx::state_machine::EventStatus cameraAlignmentAndStraighten() override;
  mcx::state_machine::EventStatus exchangeDone() override;
};

class StraightenInstrumentBeforeStore final : public SuperMerilModes {
public:
  explicit StraightenInstrumentBeforeStore(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~StraightenInstrumentBeforeStore() override = default;
  void enter() override;
};

class RetractInstrument final : public SuperMerilModes {
public:
  explicit RetractInstrument(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~RetractInstrument() override = default;
  void enter() override;
  void leave() override;
  void iterate(double dtSec) override;
  mcx::state_machine::EventStatus gotoInstrumentExchange() override;
  mcx::state_machine::EventStatus gotoUnlocked() override;
  mcx::state_machine::EventStatus gotoInstrumentStraighten() override;

private:
  double timer_{};
};

class WaitingForInstrumentExchange final : public SuperMerilModes {
public:
  explicit WaitingForInstrumentExchange(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~WaitingForInstrumentExchange() override = default;
  void enter() override;
  void leave() override;
  void iterate(double dtSec) override;
  mcx::state_machine::EventStatus gotoManualInstrumentConnect() override;
  mcx::state_machine::EventStatus gotoInstrumentConnect() override;
  mcx::state_machine::EventStatus gotoInstrumentDisconnect() override;
  mcx::state_machine::EventStatus exchangeAbort() override;

private:
  double timer_{};
  double instrumentConnectTimerSec_{};
};

class InstrumentInsertTillFulcrum final : public SuperMerilModes {
public:
  explicit InstrumentInsertTillFulcrum(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~InstrumentInsertTillFulcrum() override = default;
  void enter() override;
  void leave() override;
  void iterate(double dtSec) override;
  mcx::state_machine::EventStatus gotoLocked() override;
  mcx::state_machine::EventStatus gotoUnlocked() override;

private:
  double timer_{};
};

class InstrumentInsertAndTeachFulcrum final : public SuperMerilModes {
public:
  explicit InstrumentInsertAndTeachFulcrum(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~InstrumentInsertAndTeachFulcrum() override = default;
  void enter() override;
  void leave() override;
  void iterate(double dtSec) override;
  mcx::state_machine::EventStatus gotoUnlocked() override;
  mcx::state_machine::EventStatus gotoTeachFulcrum() override;

private:
  double timer_{};
};

class InstrumentInsert final : public SuperMerilModes {
public:
  explicit InstrumentInsert(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~InstrumentInsert() override = default;
  void enter() override;
  void leave() override;
  void iterate(double dtSec) override;
  mcx::state_machine::EventStatus gotoUnlocked() override;
  mcx::state_machine::EventStatus gotoLocked() override;
};

} // namespace logic::meril

#endif // LGC_MERIL_MODES_EXCHANGE_INSTRUMENT_H
