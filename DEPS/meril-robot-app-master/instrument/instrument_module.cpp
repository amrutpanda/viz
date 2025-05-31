/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "instrument_module.h"

using namespace mcx;

namespace {
InstrumentVisualizationArray setInstrumentVisualizationBoolArray(uint16_t index) {
  InstrumentVisualizationArray boolArray = {};
  if (index < 16) {
    boolArray[index] = true; // Set only the specified index to true
  }

  return boolArray;
}

} // namespace

void InstrumentModule::create_(const char* name, parameter_server::Parameter* parameterServer, uint64_t dtMicroS) {}

bool InstrumentModule::initPhase1_() {
  using namespace mcx::parameter_server;
  if (availableInstruments_.displayInTheTree) {
    for (auto& instrumentData : availableInstruments_.instrumentsMap) {
      auto& instrument = instrumentData.second;
      addParameter(fmt::format("AvailableInstruments/{}/{}", instrument.category, instrument.name).c_str(),
                   ParameterType::INPUT, &instrumentData.second)
          .updateOutput(false);
    }
  }
  activeInstrumentHandle_ = addParameter("ActiveInstrument", ParameterType::OUTPUT, &activeInstrument_);
  activeInstrumentHandle_.updateOutput(false);
  addParameter("activeInstrumentId", ParameterType::INPUT, &activeInstrumentId_);
  addParameter("visualizationNumber", ParameterType::OUTPUT, visualizationNumber_.data(), visualizationNumber_.size());
  return true;
}

bool InstrumentModule::initPhase2_() { return true; }

bool InstrumentModule::startOp_() { return true; }

bool InstrumentModule::stopOp_() { return true; }

bool InstrumentModule::iterateOp_(const container::TaskTime& systemTime, container::UserTime* userTime) {
  if (prevInstrumentId_ != activeInstrumentId_) {
    prevInstrumentId_ = activeInstrumentId_;
    if (availableInstruments_.instrumentsMap.contains(activeInstrumentId_)) {
      activeInstrument_ = availableInstruments_.instrumentsMap[activeInstrumentId_];
      visualizationNumber_ = setInstrumentVisualizationBoolArray(activeInstrument_.visualizationNumber);
      activeInstrumentHandle_.updateOutputOnce();
      log_info("Active Instrument Id: {}", activeInstrument_.id);
      log_info("  category: {}", activeInstrument_.category);
      log_info("  name: {}", activeInstrument_.name);
      log_info("  scaling - pitch:{}, yaw: {}, roll: {}", activeInstrument_.pitchScaleFactor,
               activeInstrument_.yawScaleFactor, activeInstrument_.rollScaleFactor);
      log_info("  pinchGain: {}, pinchOffset: {}", activeInstrument_.pinchGain, activeInstrument_.pinchOffset);
      log_info("  pinchLimits: [{}]", fmt::join(activeInstrument_.pinchLimits, ","));
      log_info("  pitchLimits: [{}]", fmt::join(activeInstrument_.pitchLimits, ","));
      log_info("  currentLimits: [{}]", fmt::join(activeInstrument_.currentLimits, ","));
      log_info("  backlashCompensation: [{}]", fmt::join(activeInstrument_.backlashCompensation, ","));
      log_info("  transformationMatrix_4x4: [{}]", fmt::join(activeInstrument_.transformationMatrix_4x4, ","));
      log_info("  inverseTransformationMatrix_4x4: [{}]",
               fmt::join(activeInstrument_.inverseTransformationMatrix_4x4, ","));
    } else {
      log_error("Active instrument id is not valid: {}", activeInstrumentId_);
    }
  }
  return true;
}