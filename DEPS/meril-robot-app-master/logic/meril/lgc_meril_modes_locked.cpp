/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_meril_modes_locked.h"
#include "lgc_meril_modes_adapter_calibration.h"
#include "lgc_meril_modes_common.h"
#include "lgc_meril_modes_instrument_calibration.h"
#include "lgc_meril_modes_instrument_exchange.h"
#include "lgc_meril_modes_reset_fulcrum.h"
#include "lgc_meril_modes_sleep.h"
#include "lgc_meril_modes_symbolic_move.h"

namespace logic::meril {

void LockedMode::enter() {
  fsmdata_.currentFsmModeOut = MerilModes::LOCKED_M;
  fsmdata_.ctrl.instrumentExchange.retractPositionIsStored = false;
  fsmdata_.ctrl.out.cameraDirectionIsReversed = false;

  // make sure that rfid is active when we are in locked mode
  fsmdata_.instrument.rfid.out.header = InstrumentConnectHeader::CONNECT;
}

void LockedMode::leave() {
  // make sure that rfid is off when not in locked mode
  fsmdata_.instrument.rfid.out.header = InstrumentConnectHeader::LOCKED;
}

mcx::state_machine::EventStatus LockedMode::gotoSleep() {
  setActiveState<ToSleepModeTransition>();
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus LockedMode::gotoMoveSymbolicPosition() {
  // allowed to go to move mode flag
  const auto ptr = setActiveState<ResetFulcrum>();
  ptr->setReturnState(LOGIC_STATE_ID(SymbolicMoveMode));
  addEvent({&SuperMerilModes::waitResetFulcrum});
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus LockedMode::gotoUnlocked() { return common::gotoUnlocked(*this, fsmdata_, id()); }

mcx::state_machine::EventStatus LockedMode::gotoSurgicalMode() { return common::gotoSurgicalMode(*this, fsmdata_); }

mcx::state_machine::EventStatus LockedMode::gotoInstrumentExchange() {
  if (!fsmdata_.ctrl.in.isCameraRobot) {
    setActiveState<InstrumentExchangeMode>();
    const auto fulcrum = fsmdata_.ctrl.in.fulcrumIsValid;
    const auto instrument = fsmdata_.ctrl.out.instrumentIsConnected;
    const auto isOutside = fsmdata_.ctrl.in.instrumentIsOutsideFulcrum;

    if (fulcrum && instrument && isOutside) {
      addEvent({&SuperMerilModes::waitExchangeInstrument});
    } else if (fulcrum && instrument && !isOutside) {
      // do instrument straightening if necessary and storeRetractPosition, otherwise call storeRetractPosition
      if (common::gotoInstrumentStraighten(*this, fsmdata_, LOGIC_STATE_ID(StraightenInstrumentBeforeStore)) ==
          mcx::state_machine::EventStatus::EVENT_NONE) {
        addEvent({&SuperMerilModes::storeRetractPosition});
      }
    } else if (fulcrum && !instrument && isOutside) {
      addEvent({&SuperMerilModes::waitExchangeInstrument});
    } else if (fulcrum && !instrument && !isOutside) {
      addEvent({&SuperMerilModes::lockedEngageExchange});
    } else if (!fulcrum && instrument && isOutside) {
      addEvent({&SuperMerilModes::waitExchangeInstrument});
    } else if (!fulcrum && instrument && !isOutside) {
      addEvent({&SuperMerilModes::waitExchangeInstrument});
    } else if (!fulcrum && !instrument && !isOutside) {
      addEvent({&SuperMerilModes::waitExchangeInstrument});
    } else if (!fulcrum && !instrument && isOutside) {
      addEvent({&SuperMerilModes::waitExchangeInstrument});
    } else {
      log_error("Cannot enter instrument exchange mode, invalid state: fulcrum: {}, instrument: {}, isOutside: {}",
                fulcrum, instrument, isOutside);
    }
  }
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus LockedMode::gotoInstrumentStraighten() {
  fsmdata_.ctrl.out.gotoInstrumentStraightenRoll = false; // only for surgical mode
  return common::gotoInstrumentStraighten(*this, fsmdata_, id());
}

mcx::state_machine::EventStatus LockedMode::gotoManualInstrumentConnect() {
  if (!fsmdata_.ctrl.in.isCameraRobot) {
    return common::gotoManualInstrumentConnect(fsmdata_);
  }
  return mcx::state_machine::EventStatus::EVENT_NONE;
}

mcx::state_machine::EventStatus LockedMode::gotoInstrumentConnect() {
  if (!fsmdata_.ctrl.in.isCameraRobot) {
    return common::gotoInstrumentConnect(fsmdata_, instrumentConnectTimerSec_, getDtSec());
  }
  return mcx::state_machine::EventStatus::EVENT_NONE;
}

mcx::state_machine::EventStatus LockedMode::gotoInstrumentDisconnect() {
  if (!fsmdata_.ctrl.in.isCameraRobot) {
    return common::gotoInstrumentDisconnect(fsmdata_, instrumentConnectTimerSec_);
  }
  return mcx::state_machine::EventStatus::EVENT_NONE;
}

mcx::state_machine::EventStatus LockedMode::gotoInstrumentCalibrate() {
  if (!fsmdata_.ctrl.in.isCameraRobot) {
    const auto ptr = setActiveState<InstrumentCalibrateMode>();
    ptr->setReturnState(id());
    addEvent({&SuperMerilModes::calibrateInstrument}, fsmdata_.instrumentCalibrationTimeoutSec);
  }
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus LockedMode::gotoAdapterCalibrate() {
  if (!fsmdata_.ctrl.in.isCameraRobot) {
    setActiveState<AdapterCalibrateMode>();
    addEvent({&SuperMerilModes::calibrateAdapter}, fsmdata_.instrumentCalibrationTimeoutSec);
  }
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

mcx::state_machine::EventStatus LockedMode::gotoTeachFulcrum() {
  if (common::teachFulcrum(fsmdata_) && (!fsmdata_.ctrl.in.isCameraRobot)) {
    const auto ptr = setActiveState<InstrumentCalibrateMode>();
    ptr->setReturnState(id());
    addEvent({&SuperMerilModes::calibrateInstrument}, fsmdata_.instrumentCalibrationTimeoutSec);
    return mcx::state_machine::EventStatus::EVENT_DONE;
  }
  return mcx::state_machine::EventStatus::EVENT_NONE;
}

mcx::state_machine::EventStatus LockedMode::gotoResetFulcrum() {
  const auto ptr = setActiveState<ResetFulcrum>();
  ptr->setReturnState(id());
  addEvent({&SuperMerilModes::waitResetFulcrum});
  // if (fsmdata_.ctrl.in.instrumentIsOutsideFulcrum || !fsmdata_.ctrl.in.fulcrumIsValid ||
  //     fsmdata_.ctrl.settings.enableMaintenanceMode || !fsmdata_.ctrl.out.instrumentIsConnected) {
  //   fsmdata_.ctrl.out.gotoTeachFulcrum = false;
  //   fsmdata_.ctrl.out.gotoResetFulcrum = true;
  //   fsmdata_.ctrl.out.manipulatorAdmittanceJoints = NO_FULCRUM;
  // } else {
  //   log_error("Cannot reset fulcrum, instrument is inside fulcrum");
  // }
  return mcx::state_machine::EventStatus::EVENT_DONE;
}

} // namespace logic::meril