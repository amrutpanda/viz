/*
 * All rights reserved. Copyright (c) 2014-2025 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_MERIL_MODES_COMMON_H
#define LGC_MERIL_MODES_COMMON_H

#include "lgc_meril_modes_base.h"
#include "lgc_meril_modes_data.h"
#include <mcx/core.h>

namespace logic::meril::common {

bool teachFulcrum(MerilModesFSMData& fsmdata);
mcx::state_machine::EventStatus gotoManualInstrumentConnect(MerilModesFSMData& fsmdata);
mcx::state_machine::EventStatus gotoInstrumentConnect(MerilModesFSMData& fsmdata, double& instrumentConnectTimerSec,
                                                      double dtSec);
mcx::state_machine::EventStatus gotoInstrumentDisconnect(MerilModesFSMData& fsmdata, double& instrumentConnectTimerSec);
mcx::state_machine::EventStatus gotoUnlocked(SuperMerilModes& fsm, MerilModesFSMData& fsmdata, int returnStateId);
mcx::state_machine::EventStatus gotoSurgicalMode(SuperMerilModes& fsm, MerilModesFSMData& fsmdata);
mcx::state_machine::EventStatus gotoInstrumentStraighten(SuperMerilModes& fsm, MerilModesFSMData& fsmdata,
                                                         int returnStateId);

} // namespace logic::meril::common

#endif // LGC_MERIL_MODES_COMMON_H
