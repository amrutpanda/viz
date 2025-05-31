/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_ROBOT_MODES_DATA_H
#define LGC_ROBOT_MODES_DATA_H

#include "lgc_motion_def.h"
#include <mcx/control3/ctrl_modes.h>
#include <mcx/core.h>
#include <mcx/motion.h>

namespace logic {

// Control <-> Modes

struct ModeToControlHandles {
  mcx::parameter_server::ParamHandle resetCollisionDetectors;
};

struct ModeFSMData {

  struct {
    bool positionError;
    bool pvaLimitActive;
  } torqueStateLeaveConditions{};

  struct {
    mcx::control3::ControlToMode in{};
    mcx::control3::ModeToControl out{};
    ModeToControlHandles outHandles{};
  } ctrl;

  struct {
    mcx::motion::InterpreterToMode in{};
    mcx::motion::ModeToInterpreter out{};
    mcx::motion::InterpreterToModeHandles inHandles{};
  } interpreter;

  Modes currentFsmModeOut{Modes::INIT_M};
  ModeEvents externalModeEventIn{ModeEvents::GOTO_NONE_E};
};

} // namespace logic

#endif /* LGC_ROBOT_MODES_DATA_H */
