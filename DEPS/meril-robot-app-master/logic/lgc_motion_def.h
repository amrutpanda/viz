/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_MOTION_DEF_H
#define LGC_MOTION_DEF_H

namespace logic {

enum class States {
  INIT_S = 0,
  OFF_S = 1,
  IDLE_S = 2,
  PAUSED_S = 3,
  ENGAGED_S = 4,
  HOMING_S = 5,
  FORCEDIDLE_S = 6,
  ESTOP_OFF_S = 7,

  // SWITCH_TO_TORQUE = 6,
  // SWITCH_TO_POSITION = 7,

  OFF_TO_IDLE_T = 102,
  OFF_TO_REFERENCING_T = 105,
  IDLE_TO_OFF_T = 201,
  PAUSED_TO_IDLE_T = 302,
  IDLE_TO_ENGAGED_T = 204,
  ENGAGED_TO_PAUSED_T = 403,
  TO_FORCEDIDLE_T = 600,
  RESET_FORCEDIDLE_T = 602,
  TO_ESTOP_T = 700,
  RESET_ESTOP_T = 701

};

enum class StateEvents {
  DO_NOTHING_E = -1,
  GOTO_OFF_E = 0,
  GOTO_IDLE_E = 1,
  GOTO_ENGAGED_E = 2,
  GOTO_REFERENCING_E = 4,
  // GOTO_TORQUE_E = 5,
  // GOTO_POSITION_E = 6,
  GOTO_HOMING_E = 7,
  FORCE_IDLE_E = 10,
  EMERGENCY_STOP_E = 20,
  SAVE_CONFIGURATION = 254,
  ACKNOWLEDGE_ERROR = 255

};

enum class Modes {
  // Modes
  INIT_M = 0,
  PAUSE_M = 1,
  AUTO_RUN_M = 2,
  MANUAL_JOINT_MODE_M = 3,
  MANUAL_CART_MODE_M = 4,
  SEMI_AUTO_MODE_M = 5,
  TORQUE_M = 6,
  MOVE_TO_START_M = 7,
  // Transitions between modes
  PAUSE_TO_AUTO_RUN_T = 102,
  PAUSE_TO_SEMI_AUTO_T = 105,
  PAUSE_TO_MANUAL_JOINT_T = 103,
  PAUSE_TO_MANUAL_CART_T = 104,
  PAUSE_TO_MOVE_TO_START_T = 107,
  MOVE_TO_START_TO_PAUSE_T = 701,
  MANUAL_CART_TO_PAUSE_T = 401,
  MANUAL_JOINT_TO_PAUSE_T = 301,
  MANUAL_CART_TO_MANUAL_JOINT_T = 403,
  MANUAL_JOINT_TO_MANUAL_CART_T = 304,
  PAUSE_TO_TORQUE_T = 106,
  TORQUE_TO_PAUSE_T = 601
};

enum class ModeEvents {
  GOTO_NONE_E = -1,
  GOTO_INIT_E = 0,
  GOTO_PAUSE_E = 1,
  GOTO_AUTO_RUN_E = 2,
  GOTO_MANUAL_JOINT_MODE_E = 3,
  GOTO_MANUAL_CART_MODE_E = 4,
  GOTO_MOVE_TO_START_E = 5,
  GOTO_TORQUE_MODE_E = 8,
  GOTO_SEMI_AUTO_E = 9
};

} // namespace logic

#endif
