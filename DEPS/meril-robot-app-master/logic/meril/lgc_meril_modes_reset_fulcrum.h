/*
* All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_MERIL_MODES_RESETFULCRUM_H
#define LGC_MERIL_MODES_RESETFULCRUM_H

#include "lgc_meril_modes_base.h"
#include "lgc_meril_modes_locked.h"

namespace logic::meril {

class ResetFulcrum final : public SuperMerilModes {

public:
  explicit ResetFulcrum(MerilModesFSMData& data) : SuperMerilModes(data) {};
  ~ResetFulcrum() override = default;
  void enter() override {
    timer_ = 0;
    fsmdata_.ctrl.out.gotoResetFulcrum = true;
    fsmdata_.ctrl.out.gotoTeachFulcrum = false;
    fsmdata_.ctrl.out.manipulatorAdmittanceJoints = NO_FULCRUM;
  }
  void leave() override { fsmdata_.ctrl.out.gotoResetFulcrum = false; }
  mcx::state_machine::EventStatus waitResetFulcrum() override {
    if (timer_ > fsmdata_.resetFulcrumTimoutSec) {
      if (returnStateId_ < 0) {
        log_error("ResetFulcrum: Return state is not set, switching to LockedMode");
        setActiveState<LockedMode>();
      } else {
        setActiveState(returnStateId_);
      }
      return mcx::state_machine::EVENT_DONE;
    }
    timer_ += getDtSec();
    return mcx::state_machine::EVENT_REPEAT;
  }

private:
  double timer_{};
};

} // namespace logic::meril

#endif // LGC_MERIL_MODES_RESETFULCRUM_H
