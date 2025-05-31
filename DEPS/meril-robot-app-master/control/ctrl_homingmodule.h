/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef CTRL_HOMINGMODULE_H
#define CTRL_HOMINGMODULE_H

#include "lgc_homing_def.h"
#include <mcx/control3/ctrl_transducer.h>
#include <mcx/control3/ctrl_windowdetector.h>
#include <mcx/core.h>

using HomingMethods = logic::HomingMethods;
using HomingStates = logic::HomingStates;

class HomingModule final : public mcx::container::Module {

  using TriggerDetectors = std::vector<mcx::control3::WindowDetector>;

public:
  static constexpr double DEFAULT_RETURN_JOG_FACTOR = 1;

  explicit HomingModule(unsigned int numberOfActuators = 1);

  ~HomingModule() override = default;

private:
  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override;

  bool initPhase1_() override;

  bool initPhase2_() override;

  bool startOp_() override;

  bool stopOp_() override;

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override;

  bool gotoHoming_{};
  bool gotoManualAdjust_{};
  bool gotoJogging_{};
  std::vector<mcx::control3::ReferencingToState> fromTransducer_;
  std::vector<mcx::control3::StateToReferencing> toTransducer_;
  std::vector<double> jogVelocityTarget_; // link to inputJogVelocity in axis or actuator.
  std::vector<double> backlash_;
  std::vector<double> backlashRange_;
  std::vector<double> snapshotForward_;
  std::vector<double> snapshotBackward_;
  std::vector<double> absoluteEncoderPositionActual_;
  std::vector<double> absoluteEncoderPositionReference_;
  std::vector<double> gearRatio_;
  std::vector<double> manualJogVelocity_;
  std::vector<double> jogVelocity_;
  std::vector<HomingMethods> method_{HomingMethods::BYPASS};
  std::vector<HomingStates> state_{HomingStates::OFF};
  HomingStates logicState_{HomingStates::OFF};
  double backlashFactor_{1.0};
  unsigned int numberOfActuators_{};
  TriggerDetectors triggerDetectors_{};
};

#endif /* CTRL_HOMINGMODULE_H */