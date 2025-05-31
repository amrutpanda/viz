/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "ctrl_symbolicpositionmodule.h"
#include "mcx/control3/ctrl_helper.h"
#include <mcx/control3/ctrl_joints.h>

using namespace mcx;

SymbolicPositionModule::SymbolicPositionModule(unsigned int numberOfJoints, std::vector<std::string> symbolicNames)
    : numberOfJoints_{numberOfJoints} {

  const unsigned int numberOfSymbolicPositions = symbolicNames.size();
  numberOfChannels_ = numberOfSymbolicPositions;
  jointSymbolicPosition_.resize(numberOfJoints);
  syncJointPositions_.resize(numberOfJoints);
  gotoJointMoveToSymbolicPositionPrevious_.resize(numberOfJoints);
  jointSymbolicPositions_.resize(numberOfSymbolicPositions); // resize struct
  for (size_t i = 0; i < numberOfSymbolicPositions; i++) {   // resize content of struct
    jointSymbolicPositions_[i].number = i;
    jointSymbolicPositions_[i].name = symbolicNames[i];
    jointSymbolicPositions_[i].tolerance = SYMBOLIC_POSITION_TOLERANCE;
    jointSymbolicPositions_[i].moveOrder.resize(numberOfJoints);
    jointSymbolicPositions_[i].isBusy.resize(numberOfJoints);
    jointSymbolicPositions_[i].isAtPosition.resize(numberOfJoints);
    jointSymbolicPositions_[i].moveToPosition.resize(numberOfJoints);
    jointSymbolicPositions_[i].position.resize(numberOfJoints);
  }
  outputPositions_.resize(numberOfJoints);

  for (size_t i = 0; i < numberOfJoints; i++) {
    jointSetpointGenerators_.push_back(std::make_unique<control3::SetpointGenerator>());
  }
  jointSetpointGenerators_.shrink_to_fit();
}

void SymbolicPositionModule::create_(const char* name, parameter_server::Parameter* parameterServer,
                                     uint64_t dtMicroS) {

  std::string moduleName;
  for (size_t i = 0; i < jointSetpointGenerators_.size(); i++) {
    moduleName = "setpointGenerators/setpointGenerator" + std::to_string(i);
    createSubmodule(jointSetpointGenerators_[i].get(), moduleName.c_str());
  }
}

bool SymbolicPositionModule::initPhase1_() {
  using ParamType = parameter_server::ParameterType;

  addParameter("start", ParamType::INPUT, &startMove_);
  addParameter("select", ParamType::INPUT, &selectMove_);
  addParameter("timeScaleFactor", ParamType::INPUT, &timeScaleFactor_);
  addParameter("currentMoveOrder", ParamType::OUTPUT, &currentMoveOrder_);
  addParameterVec("outputPositions", ParamType::OUTPUT, outputPositions_);

  //  unsigned int counter = 0;
  for (auto& el : jointSymbolicPositions_) {
    //    std::string paramName = fmt::format("symbolicPositions/symbolicPosition{:02}", ++counter);
    std::string paramName = fmt::format("symbolicPositions/{}", el.name);
    addParameter(fmt::format("{}/teach", paramName).c_str(), ParamType::INPUT, &el.teach);
    addParameterVec(fmt::format("{}/moveToPosition", paramName).c_str(), ParamType::INPUT, el.moveToPosition);

    addParameterVec(fmt::format("{}/isAtPosition", paramName).c_str(), ParamType::OUTPUT, el.isAtPosition);
    addParameter(fmt::format("{}/done", paramName).c_str(), ParamType::OUTPUT, &el.done);

    addParameterVec(fmt::format("{}/moveOrder", paramName).c_str(), ParamType::PARAMETER, el.moveOrder);
    addParameterVec(fmt::format("{}/position", paramName).c_str(), ParamType::PARAMETER, el.position);
    addParameter(fmt::format("{}/tolerance", paramName).c_str(), ParamType::PARAMETER, &el.tolerance);
  }
  return true;
}

bool SymbolicPositionModule::initPhase2_() { return true; }

bool SymbolicPositionModule::startOp_() { return true; }

bool SymbolicPositionModule::stopOp_() { return true; }

bool SymbolicPositionModule::checkAtSymbolicPosition(const control3::JointPositions& jointPositionsTarget,
                                                     const control3::JointPositions& jointPositionsActual,
                                                     unsigned int channel) {
  bool done = true;
  for (size_t i = 0; i < numberOfJoints_; i++) {
    if (jointSymbolicPositions_[channel].moveOrder[i] > 0) {
      const bool errorBelowThreshold =
          std::fabs(jointPositionsTarget[i] - jointPositionsActual[i]) < jointSymbolicPositions_[channel].tolerance;
      jointSymbolicPositions_[channel].isAtPosition[i] = jointSetpointGenerators_[i]->isDone() && errorBelowThreshold;
      done &= jointSymbolicPositions_[channel].isAtPosition[i];
    }
  }
  jointSymbolicPositions_[channel].done = done;
  return done;
}

control3::JointPositions&
SymbolicPositionModule::moveToSymbolicPositions(control3::JointPositions& jointPositionsTarget,
                                                const control3::JointPositions& jointPositionsActual,
                                                unsigned int channel, bool startMove) {

  // move all joints with moveOrder 1
  // iff all joint with moveOrder 1 are Done then
  // move all joints with moveOrder 2, etc
  startMove_ = startMove;

  if (checkAtSymbolicPosition(jointPositionsActual, channel)) {
    startMove_ = false;
  } else {
    for (size_t order = 1; order < numberOfJoints_; order++) {
      bool orderDone = true;
      for (size_t i = 0; i < numberOfJoints_; i++) {
        if (jointSymbolicPositions_[channel].moveOrder[i] == order) {
          orderDone &= jointSymbolicPositions_[channel].isAtPosition[i];
        }
      }
      if (!orderDone) {
        currentMoveOrder_ = order;
        break;
      }
    }
  }

  // channel is the symbolic position that the system should move to (e.g. moveToStart1)
  for (size_t i = 0; i < numberOfJoints_; i++) {
    if (startMove_ && jointSymbolicPositions_[channel].moveOrder[i] == currentMoveOrder_) {
      if (!jointSymbolicPositions_[channel].isBusy[i]) {
        jointSymbolicPositions_[channel].isBusy[i] = true;
        jointSetpointGenerators_[i]->setInput(jointSymbolicPositions_[channel].position[i]);
      }
    } else {
      jointSymbolicPositions_[channel].isBusy[i] = false;
      jointSetpointGenerators_[i]->resetTo(jointPositionsActual[i]);
    }
    jointPositionsTarget[i] = jointSetpointGenerators_[i]->getOutput();
  }

  return jointPositionsTarget;
}

bool SymbolicPositionModule::iterateOp_(const container::TaskTime& systemTime, container::UserTime* userTime) {
  unsigned int i = 0;
  for (const auto& generator : jointSetpointGenerators_) {
    const double timeScaleFactor = control3::rateLimit(timeScaleFactor_, timeScaleFactorPrev_, 3.0, 30.0, getDtSec());
    timeScaleFactorPrev_ = timeScaleFactor;
    generator->setTimeScaleFactor(timeScaleFactor);
    generator->iterate(systemTime, userTime);
    outputPositions_[i++] = generator->getOutput();
  }

  return true;
}
