/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "ctrl_homingmodule.h"
#include <mcx/control3/ctrl_joints.h>

using namespace mcx;

HomingModule::HomingModule(unsigned int numberOfActuators)
    : numberOfActuators_{numberOfActuators}, triggerDetectors_{numberOfActuators} {
  fromTransducer_.resize(numberOfActuators_);
  toTransducer_.resize(numberOfActuators_);
  absoluteEncoderPositionReference_.resize(numberOfActuators_);
  absoluteEncoderPositionActual_.resize(numberOfActuators_);
  gearRatio_.resize(numberOfActuators_);
  jogVelocity_.resize(numberOfActuators_);
  jogVelocityTarget_.resize(numberOfActuators_);
  manualJogVelocity_.resize(numberOfActuators_);
  snapshotForward_.resize(numberOfActuators_);
  snapshotBackward_.resize(numberOfActuators_);
  backlash_.resize(numberOfActuators_);
  backlashRange_.resize(numberOfActuators_);
  method_.resize(numberOfActuators_);
  state_.resize(numberOfActuators_);
  constexpr double DEFAULT_HOMING_VELOCITY = 0.1;
  std::ranges::fill(jogVelocity_, DEFAULT_HOMING_VELOCITY);
}

void HomingModule::create_(const char* name, parameter_server::Parameter* parameterServer, uint64_t dtMicroS) {

  createSubmodules(utils::make_span(triggerDetectors_), "detectors/triggerDetector");
}

bool HomingModule::initPhase1_() {
  using namespace mcx::parameter_server;
  addParameter("gotoHoming", ParameterType::INPUT, &gotoHoming_);             // enable homing module in homing state
  addParameter("gotoManualAdjust", ParameterType::INPUT, &gotoManualAdjust_); // enable homing module in homing state
  addParameter("gotoJogging", ParameterType::OUTPUT, &gotoJogging_);          // enable homing module in homing state
  addParameterVec("manualJogVelocity", ParameterType::INPUT, manualJogVelocity_);

  addParameterVec("absoluteEncoderPositionReference", ParameterType::PARAMETER, absoluteEncoderPositionReference_);
  addParameterVec("absoluteEncoderPositionActual", ParameterType::INPUT, absoluteEncoderPositionActual_);
  addParameterVec("gearRatio", ParameterType::PARAMETER, gearRatio_);

  addParameterVec("jogVelocityTarget", ParameterType::OUTPUT, jogVelocityTarget_);
  addParameterVec("snapshotForward", ParameterType::OUTPUT, snapshotForward_);
  addParameterVec("snapshotBackward", ParameterType::OUTPUT, snapshotBackward_);
  addParameterVec("state", ParameterType::OUTPUT, state_);
  addParameter("logicState", ParameterType::OUTPUT, &logicState_);

  addParameterVec("backlash", ParameterType::PARAMETER, backlash_);
  addParameter("backlashFactor", ParameterType::PARAMETER, &backlashFactor_);
  addParameterVec("jogVelocity", ParameterType::PARAMETER, jogVelocity_);
  addParameterVec("method", ParameterType::PARAMETER, method_);

  size_t counter = 0;
  for (auto& el : fromTransducer_) {
    addParameter(fmt::format(":fromTransducer/:actuator{:02}", ++counter).data(), ParameterType::INPUT, &el);
  }
  counter = 0;
  for (auto& el : toTransducer_) {
    addParameter(fmt::format(":toTransducer/:actuator{:02}", ++counter).data(), ParameterType::OUTPUT, &el);
  }

  return true;
}

bool HomingModule::initPhase2_() { return true; }

bool HomingModule::startOp_() {
  for (auto& el : triggerDetectors_) {
    el.setAutoReset(true);
    //    el.setTooHigh(2.5);
    //    el.setHigh(1.5);
    //    el.setLow(-1.5);
    //    el.setTooLow(-2.5);
  }
  return true;
}

bool HomingModule::stopOp_() { return true; }

bool HomingModule::iterateOp_(const container::TaskTime& systemTime, container::UserTime* userTime) {

  logicState_ = HomingStates::DONE;
  gotoJogging_ = false;

  // this is performed when the actuator is not powered yet (not engaged)
  //  for (unsigned int i = 0; i < numberOfActuators_; i++) {
  //    if (method_[i] == HomingMethods::REFERENCE_FROM_EXTERNAL_ENCODER) {
  //      toTransducer_[i].hardwareSnapshotTarget =
  //      (absoluteEncoderPositionActual_[i]-absoluteEncoderPositionReference_[i]) * gearRatio_[i] - incremental encoder
  //      value; toTransducer_[i].setHardwareSnapshot = true;
  //    }
  //  }

  if (gotoHoming_ | gotoManualAdjust_) { // I'm in Homing State
    gotoJogging_ = true;
    for (unsigned int i = 0; i < numberOfActuators_; i++) {

      // reset output variables
      toTransducer_[i].setHardwareSnapshot = false;
      toTransducer_[i].setHardwareReference = false;
      toTransducer_[i].reset = false;
      toTransducer_[i].hardwareSnapshotTarget = 0.0;
      jogVelocityTarget_[i] = 0.0;

      triggerDetectors_[i].iterate(systemTime, userTime);

      if (gotoManualAdjust_) {
        state_[i] = HomingStates::MOVE_MANUAL;
      }

      if (fromTransducer_[i].hasHardwareSnapshot) {
        // react to external trigger...
        state_[i] = HomingStates::SET_REFERENCE;
      }

      if (method_[i] == HomingMethods::BYPASS) {
        state_[i] = HomingStates::DONE;
      }

      const double jogFactor =
          (method_[i] == HomingMethods::REFERENCE_IN_CENTER_BETWEEN_TRIGGERS) ? DEFAULT_RETURN_JOG_FACTOR : 1.0;

      switch (state_[i]) {
      case HomingStates::OFF:
        state_[i] = HomingStates::INITIALIZE;
        break;
      case HomingStates::INITIALIZE:
        // reset variables
        toTransducer_[i].reset = true;
        jogVelocityTarget_[i] = 0.0; // reset jog velocity (overwritten in states)

        backlash_[i] = 0.0; // reset backlash to not interfere the homing procedure

        if (method_[i] == HomingMethods::REFERENCE_AT_TRIGGER ||
            method_[i] == HomingMethods::REFERENCE_IN_CENTER_BETWEEN_TRIGGERS) {
          state_[i] = HomingStates::MOVE_FORWARD_TO_TRIGGER;
        } else if (method_[i] == HomingMethods::REFERENCE_DIRECT) {
          state_[i] = HomingStates::SET_REFERENCE;
        } else if (method_[i] == HomingMethods::REFERENCE_MANUAL_AT_TRIGGER) {
          state_[i] = HomingStates::MOVE_MANUAL_TO_TRIGGER;
        } else if (method_[i] == HomingMethods::REFERENCE_MANUAL) {
          state_[i] = HomingStates::MOVE_MANUAL;
        }
        break;
      case HomingStates::MOVE_FORWARD_TO_TRIGGER:
        if (triggerDetectors_[i].getWindowDetectorState().isHigh) { // || triggerDetectors_[i].getTooHigh()) {
          if (method_[i] == HomingMethods::REFERENCE_AT_TRIGGER) {
            state_[i] = HomingStates::SET_SNAPSHOT;
          }
          if (method_[i] == HomingMethods::REFERENCE_IN_CENTER_BETWEEN_TRIGGERS) {
            snapshotForward_[i] = fromTransducer_[i].inputHardware;
            state_[i] = HomingStates::MOVE_BACKWARD_TO_TRIGGER;
          }
        } else {
          // set forward (positive) jogVelocity is linked to actuatorControl inputJogVelocity
          jogVelocityTarget_[i] = jogVelocity_[i];
        }
        break;
      case HomingStates::MOVE_BACKWARD_TO_TRIGGER:
        if (triggerDetectors_[i].getWindowDetectorState().isLow) { //} || triggerDetectors_[i].getTooLow()) {
          if (method_[i] == HomingMethods::REFERENCE_AT_TRIGGER) {
            state_[i] = HomingStates::SET_SNAPSHOT;
          }
          if (method_[i] == HomingMethods::REFERENCE_IN_CENTER_BETWEEN_TRIGGERS) {
            snapshotBackward_[i] = fromTransducer_[i].inputHardware;
            state_[i] = HomingStates::SET_SNAPSHOT;
          }
        } else {
          // set backward (negative) jogVelocity
          jogVelocityTarget_[i] = -jogVelocity_[i] * jogFactor;
        }
        break;
      case HomingStates::MOVE_FORWARD_TO_NO_TRIGGER:
        if (triggerDetectors_[i].getWindowDetectorState().isLow) {
          jogVelocityTarget_[i] = jogVelocity_[i];
        }
        break;
      case HomingStates::MOVE_BACKWARD_TO_NO_TRIGGER:
        if (triggerDetectors_[i].getWindowDetectorState().isHigh) {
          jogVelocityTarget_[i] = jogVelocity_[i];
        }
        break;
      case HomingStates::MOVE_MANUAL_TO_TRIGGER:
        jogVelocityTarget_[i] = manualJogVelocity_[i]; // reset jog velocity (overwritten in states)
        // set forward (positive) jogVelocity
        if (triggerDetectors_[i].getWindowDetectorState().isHigh ||
            triggerDetectors_[i].getWindowDetectorState().isLow) { //} || triggerDetectors_[i].getTooLow()) {
          manualJogVelocity_[i] = 0.0;
          state_[i] = HomingStates::SET_SNAPSHOT;
        }
        break;
      case HomingStates::MOVE_MANUAL:
        jogVelocityTarget_[i] = manualJogVelocity_[i]; // reset jog velocity (overwritten in states)
        break;
      case HomingStates::SET_SNAPSHOT:
        if (method_[i] == HomingMethods::REFERENCE_IN_CENTER_BETWEEN_TRIGGERS) {
          backlashRange_[i] = (snapshotForward_[i] - snapshotBackward_[i]) / 2; // linked to actuatorControl backlash
          toTransducer_[i].hardwareSnapshotTarget = snapshotBackward_[i] + (backlashRange_[i]);
        }
        toTransducer_[i].setHardwareSnapshot = true;
        // wait till confirmation of snapshot taken
        if (fromTransducer_[i].hasHardwareSnapshot) {
          state_[i] = HomingStates::SET_REFERENCE;
        }
        break;
      case HomingStates::SET_REFERENCE:
        toTransducer_[i].setHardwareReference = true;
        if (fromTransducer_[i].isHardwareReferenced) {
          backlash_[i] = backlashRange_[i] * backlashFactor_;
          state_[i] = HomingStates::DONE;
        }
        // timeout?
        break;
      case HomingStates::DONE:
        break;
      case HomingStates::ABORT:
        log_warning("{}", fmt::format("Calibration procedure for actuator {} is ABORTED", i + 1));
        break;
      default:
        break;
      } // switch

      if ((state_[i] != HomingStates::DONE) && (state_[i] != HomingStates::ABORT)) {
        logicState_ = HomingStates::BUSY;
      }
    }
  } else {
    for (unsigned int i = 0; i < numberOfActuators_; i++) {
      triggerDetectors_[i].iterate(systemTime, userTime);
      jogVelocityTarget_[i] = 0.0;
      state_[i] = HomingStates::OFF;
      logicState_ = HomingStates::OFF;
    }
  }

  return true;
}
