/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "ctrl_compliancecontrol.h"
#include <mcx/mechanics/mech_data_types.h>

namespace control {

using namespace mcx;

ComplianceControl::ComplianceControl(unsigned int numberOfChannels)
    : numberOfChannels_(numberOfChannels), referencedPVA_(numberOfChannels), actualPVA_(numberOfChannels),
      inputPVA_(numberOfChannels), outputPVA_(numberOfChannels), virtualMass_(numberOfChannels),
      measuredTorque_(numberOfChannels), inputTorque_(numberOfChannels), outputTorque_(numberOfChannels),
      admittanceModels_(numberOfChannels) {}

void ComplianceControl::create_(const char* name, parameter_server::Parameter* parameterServer, uint64_t dtMicroS) {
  createSubmodules(utils::make_span(admittanceModels_), "admittanceControl");
}

bool ComplianceControl::initPhase1_() {
  using namespace mcx::parameter_server;
  addParameter("enableAdmittanceControl", ParameterType::PARAMETER, &enableAdmittanceControl_);
  addParameter("resetPosition", ParameterType::INPUT, &resetPosition_);
  addParameter("resetVelocity", ParameterType::INPUT, &resetVelocity_);

  addParameterVec("virtualMass", ParameterType::INPUT, virtualMass_);
  addParameterVec("measuredTorque", ParameterType::INPUT, measuredTorque_);
  addParameterVec("staticTorque", ParameterType::INPUT, inputTorque_);

  addParameterVec("referencedPVA/position", ParameterType::INPUT, referencedPVA_.pos);
  addParameterVec("referencedPVA/velocity", ParameterType::INPUT, referencedPVA_.vel);
  addParameterVec("referencedPVA/acceleration", ParameterType::INPUT, referencedPVA_.acc);

  addParameterVec("admittancePVA/position", ParameterType::OUTPUT, outputPVA_.pos);
  addParameterVec("admittancePVA/velocity", ParameterType::OUTPUT, outputPVA_.vel);
  addParameterVec("admittancePVA/acceleration", ParameterType::OUTPUT, outputPVA_.acc);

  return true;
}

bool ComplianceControl::initPhase2_() { return true; }

bool ComplianceControl::startOp_() { return true; }

bool ComplianceControl::stopOp_() { return true; }

bool ComplianceControl::iterateOp_(const container::TaskTime& systemTime, container::UserTime* userTime) {

  for (unsigned int cnt = 0; cnt < numberOfChannels_; ++cnt) {
    // ADMITTANCE CONTROL
    admittanceModels_[cnt].setResetVelocity(resetVelocity_);
    admittanceModels_[cnt].setResetPosition(resetPosition_);
    admittanceModels_[cnt].setDisableDynamics(disableDynamics_ || !enableAdmittanceControl_);
    admittanceModels_[cnt].setActuatorVirtualMass(virtualMass_[cnt]);
    admittanceModels_[cnt].setInputPVA(inputPVA_.pos[cnt], inputPVA_.vel[cnt], inputPVA_.acc[cnt]);
    admittanceModels_[cnt].setStaticTorque(inputTorque_[cnt]);
    admittanceModels_[cnt].setMeasuredTorque(measuredTorque_[cnt]);
    admittanceModels_[cnt].setReferencePVA(referencedPVA_.pos[cnt], referencedPVA_.vel[cnt], referencedPVA_.acc[cnt]);
    admittanceModels_[cnt].setActualPVA(actualPVA_.pos[cnt], actualPVA_.vel[cnt], actualPVA_.acc[cnt]);
    admittanceModels_[cnt].iterate(systemTime, userTime);
    outputPVA_.pos[cnt] = admittanceModels_[cnt].getOutputPosition();
    outputPVA_.vel[cnt] = admittanceModels_[cnt].getOutputVelocity();
    outputPVA_.acc[cnt] = admittanceModels_[cnt].getOutputAcceleration();
  }
  return true;
}

} // namespace control
