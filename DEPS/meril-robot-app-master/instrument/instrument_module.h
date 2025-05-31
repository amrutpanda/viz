/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef INSTRUMENT_MODULE_H
#define INSTRUMENT_MODULE_H

#include "instrument_description.h"
#include <mcx/core.h>

using InstrumentVisualizationArray = std::array<bool, 16>;

class InstrumentModule final : public mcx::container::Module {
public:
  explicit InstrumentModule(AvailableInstruments availableInstruments)
      : availableInstruments_{std::move(availableInstruments)} {}

  ~InstrumentModule() override = default;

private:
  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override;

  bool initPhase1_() override;

  bool initPhase2_() override;

  bool startOp_() override;

  bool stopOp_() override;

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override;

  InstrumentVisualizationArray visualizationNumber_{};
  AvailableInstruments availableInstruments_;
  InstrumentData activeInstrument_{};
  mcx::parameter_server::GroupHandle activeInstrumentHandle_;
  INSTRUMENT_ID_TYPE activeInstrumentId_{NO_INSTRUMENT_ID};
  INSTRUMENT_ID_TYPE prevInstrumentId_{NO_INSTRUMENT_ID - 1};
};

#endif /* INSTRUMENT_MODULE_H */