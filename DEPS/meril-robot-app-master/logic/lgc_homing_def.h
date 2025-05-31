/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_HOMING_DEF_H
#define LGC_HOMING_DEF_H

namespace logic {
enum class HomingStates {
  OFF = 0,
  INITIALIZE = 1,
  BUSY = 2,
  MOVE_FORWARD_TO_TRIGGER = 3,
  MOVE_BACKWARD_TO_TRIGGER = 4,
  MOVE_FORWARD_TO_NO_TRIGGER = 5,
  MOVE_BACKWARD_TO_NO_TRIGGER = 6,
  MOVE_MANUAL_TO_TRIGGER = 7,
  MOVE_MANUAL = 8,
  SET_SNAPSHOT = 10,
  SET_REFERENCE = 11,
  DONE = 20,
  ABORT = 30
};

enum class HomingMethods {
  BYPASS = 0,
  REFERENCE_DIRECT = 1,
  REFERENCE_AT_TRIGGER = 2,
  REFERENCE_IN_CENTER_BETWEEN_TRIGGERS = 3,
  REFERENCE_MANUAL_AT_TRIGGER = 6,
  REFERENCE_MANUAL = 7,
  REFERENCE_FROM_EXTERNAL_ENCODER = 8
};
} // namespace logic

#endif // LGC_HOMING_DEF_H
