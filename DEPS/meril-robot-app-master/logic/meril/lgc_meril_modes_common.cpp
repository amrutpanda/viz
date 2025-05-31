/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_meril_modes_common.h"
#include "lgc_meril_modes_base.h"
#include "lgc_meril_modes_instrument_straighten.h"
#include "lgc_meril_modes_surgical.h"
#include "lgc_meril_modes_unlocked.h"

namespace logic::meril::common {

bool teachFulcrum(MerilModesFSMData& fsmdata) {
  if (fsmdata.ctrl.in.fulcrumIsStored) {
    log_error("Fulcrum is stored, cannot teach fulcrum");
  }
  if (!fsmdata.ctrl.cartIsDocked) {
    log_error("Cart is not docked, cannot teach fulcrum");
  }
  if (fsmdata.ctrl.in.jointManipulatorIsLimiting) {
    log_error("A manipulator joint is limiting in position, cannot teach fulcrum");
  }
  if (fsmdata.instrument.activeInstrumentId == NO_INSTRUMENT_ID && !fsmdata.ctrl.in.isCameraRobot) {
    log_error("No instrument is connected, cannot teach fulcrum");
  }
  // teach fulcrum can only be done if:
  // * there is no fulcrum (active or stored)
  // * there is an instrument attached or this is a camera robot
  // * the cart is docked and can't move
  // * the manipulator is not in any of its joint limits
  if (!fsmdata.ctrl.in.fulcrumIsStored &&
      (fsmdata.instrument.activeInstrumentId != NO_INSTRUMENT_ID || fsmdata.ctrl.in.isCameraRobot) &&
      fsmdata.ctrl.cartIsDocked && !fsmdata.ctrl.in.jointManipulatorIsLimiting) {
    fsmdata.ctrl.out.gotoTeachFulcrum = true;
    fsmdata.ctrl.out.gotoResetFulcrum = false;
    fsmdata.ctrl.out.manipulatorAdmittanceJoints = FULCRUM;
    return true;
  }
  return false;
}

mcx::state_machine::EventStatus gotoManualInstrumentConnect(MerilModesFSMData& fsmdata) {
  // manual, when pin status is Connected and the user pressed gotoManualGuiButton, perform an update
  fsmdata.instrument.control.gotoManualConnectGuiButton = false;
  fsmdata.instrument.activeInstrumentId = fsmdata.instrument.control.type;
  fsmdata.ctrl.out.instrumentIsConnected = fsmdata.instrument.activeInstrumentId != NO_INSTRUMENT_ID;
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus gotoInstrumentConnect(MerilModesFSMData& fsmdata, double& instrumentConnectTimerSec,
                                                      const double dtSec) {

  auto& rfid = fsmdata.instrument.rfid;
  const auto lastCommandIssued =
      rfid.state.commandNumber >= MerilModesFSMData::Instrument::RFID::Reader::COMMANDS.size();
  if (fsmdata.instrument.rfid.in.type != 0 && lastCommandIssued) {
    fsmdata.ctrl.out.instrumentIsConnected = true;
    fsmdata.instrument.activeInstrumentId = fsmdata.instrument.rfid.in.type;
    log_info("Instrument {} connected, {} usages from {} ", rfid.in.type, rfid.in.numberOfUsages, rfid.in.usageLimit);
    return mcx::state_machine::EVENT_DONE;
  }

  if ((rfid.state.commandNumber < MerilModesFSMData::Instrument::RFID::Reader::COMMANDS.size()) &&
      (instrumentConnectTimerSec == 0)) {
    rfid.out.controlWord = MerilModesFSMData::Instrument::RFID::Reader::COMMANDS[rfid.state.commandNumber++];
    log_info("Instrument connect in progress, sending command: {}", rfid.out.controlWord);
  }

  instrumentConnectTimerSec += dtSec;
  if (instrumentConnectTimerSec >= rfid.state.commandCycleSec) {
    instrumentConnectTimerSec = 0;
    if (lastCommandIssued) {
      log_error("Failed to connect instrument, retrying in {} seconds", rfid.state.commandRetrySec);
      fsmdata.instrument.rfid.state.commandNumber = 0;
      instrumentConnectTimerSec = -rfid.state.commandRetrySec;
    }
  }

  return mcx::state_machine::EVENT_NONE;
}

mcx::state_machine::EventStatus gotoInstrumentDisconnect(MerilModesFSMData& fsmdata,
                                                         double& instrumentConnectTimerSec) {
  fsmdata.ctrl.out.instrumentIsConnected = false;
  fsmdata.instrument.activeInstrumentId = NO_INSTRUMENT_ID;
  instrumentConnectTimerSec = 0;

  auto& rfid = fsmdata.instrument.rfid;
  rfid.state.commandNumber = 0;
  rfid.out.controlWord = MerilModesFSMData::Instrument::RFID::Reader::COMMANDS[rfid.state.commandNumber];

  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus gotoUnlocked(SuperMerilModes& fsm, MerilModesFSMData& fsmdata, int returnStateId) {
  SuperMerilModes* newStatePtr{};
  if (fsmdata.ctrl.in.fulcrumIsValid) {
    newStatePtr = fsm.setActiveState<UnlockedInstrumentMode>();
  } else {
    // reset the fulcrum if fulcrum is stored but not valid
    if (fsmdata.ctrl.in.fulcrumIsStored) {
      // reset fulcrum.
      newStatePtr = fsm.setActiveState<ResetFulcrumUnlockedMode>();
      fsm.addEvent({&SuperMerilModes::gotoUnlocked}, fsmdata.resetFulcrumTimoutSec);
    } else {
      newStatePtr = fsm.setActiveState<UnlockedMode>();
    }
  }
  if (newStatePtr) {
    newStatePtr->setReturnState(returnStateId);
  } else {
    log_error("Failed to set return state");
  }
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus gotoSurgicalMode(SuperMerilModes& fsm, MerilModesFSMData& fsmdata) {
  // allowed to go to surgical mode flag
  if (fsmdata.ctrl.in.surgicalModeIsAllowed) {
    if (gotoInstrumentStraighten(fsm, fsmdata, LOGIC_STATE_ID(SurgicalMode)) == mcx::state_machine::EVENT_NONE) {
      fsm.setActiveState<SurgicalMode>();
    }
    return mcx::state_machine::EventStatus::EVENT_DONE;
  }
  return mcx::state_machine::EventStatus::EVENT_NONE;
}

mcx::state_machine::EventStatus gotoInstrumentStraighten(SuperMerilModes& fsm, MerilModesFSMData& fsmdata,
                                                         int returnStateId) {
  if (!fsmdata.ctrl.in.instrumentStraightenDone) {
    const auto statePtr = fsm.setActiveState<InstrumentStraightenMode>();
    statePtr->setReturnState(returnStateId);
    return mcx::state_machine::EventStatus::EVENT_DONE;
  }
  return mcx::state_machine::EventStatus::EVENT_NONE;
}

} // namespace logic::meril::common