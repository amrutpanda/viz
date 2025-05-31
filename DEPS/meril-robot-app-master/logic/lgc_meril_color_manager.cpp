/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "lgc_motion.h"
#include "lgc_robot_states_es.h"
#include "lgc_robot_states_fd.h"
#include "meril/lgc_meril_modes_data.h"
#include "meril/lgc_meril_modes_instrument_calibration.h"
#include "meril/lgc_meril_modes_instrument_exchange.h"
#include "meril/lgc_meril_modes_surgical.h"
#include "meril/lgc_meril_modes_symbolic_move.h"
#include "meril/lgc_meril_modes_unlocked.h"

namespace logic {
/**
 * The colorManager class is responsible for handling and managing colors
 * within an application. It provides methods to manipulate, retrieve,
 * and store color-related data.
 *
 * This class can be used to centralize color-related operations, ensuring
 * consistent usage and modifications of colors throughout the application.
 *
 * Responsibilities of this class include:
 * - Storing and managing a collection of colors.
 * - Converting colors between different formats or color spaces.
 * - Providing utilities to create, compare, and manipulate colors.
 * - Managing default or theme-specific colors.
 *
 * Designed to improve code maintainability when handling colors, especially
 * in applications with dynamic or extensive visual requirements.
 */
void MotionLogic::colorManager() {
  // color management
  if (robotStatesFsm_.isStateActive<OffState>()) {
    // setStatusLights(IDLE);
    setStatusLights(RGB_IDLE);
  }
  if (robotStatesFsm_.isStateActive<OffToIdleState>()) {
    // setStatusLights(WHITE_BLINK);
    setStatusLights(RGB_IDLE);
  }
  if (robotStatesFsm_.isStateActive<IdleState>()) {
    // setStatusLights(WHITE);
    setStatusLights(RGB_IDLE);
  }
  if (robotStatesFsm_.isStateActive<ForcedIdleState>()) {
    // setStatusLights(RED);
    setStatusLights(RGB_ERROR);
  }
  if (robotStatesFsm_.isStateActive<EStopOffState>()) {
    // setStatusLights(RED);
    setStatusLights(RGB_ERROR);
  }
  if (robotStatesFsm_.isStateActive<EStopResetState>()) {
    // setStatusLights(RED_BLINK);
    setStatusLights(RGB_RESETERROR);
  }
  if (robotStatesFsm_.isStateActive<ReferencingState>() ||
      merilModesFsm_.isStateActive<meril::InstrumentCalibrateMode>() ||
      merilFsmData_.ctrl.out.gotoInstrumentStraighten) {
    // setStatusLights(BLUE_BLINK);
    setStatusLights(RGB_INSTRUMENTCALIBRATION);
  }
  if (robotStatesFsm_.isStateActive<EngagedState>()) {
    // setStatusLights(BLUE);
    setStatusLights(RGB_LOCKED);
    if (merilFsmData_.ctrl.out.gotoManipulatorManual) {
      if (merilFsmData_.ctrl.out.enableManipulatorManualLinear) {
        // setStatusLights(PINK_BLINK);
        setStatusLights(RGB_HANDGUIDINGLINEAR);
      } else {
        // setStatusLights(PINK);
        setStatusLights(RGB_HANDGUIDING);
      }
    }
    if (merilModesFsm_
            .isStateActive<meril::UnlockedMode, meril::UnlockedInstrumentMode, meril::TransitionToLockedMode>() ||
        merilFsmData_.ctrl.out.gotoManipulatorManual) {
      setStatusLights(RGB_HANDGUIDING);
      if (merilFsmData_.ctrl.out.enableManipulatorManualLinear) {
        setStatusLights(RGB_HANDGUIDINGLINEAR);
      }
    }
    if (merilModesFsm_.isStateActive<meril::SymbolicMoveMode>()) {
      // setStatusLights(YELLOW);
      setStatusLights(RGB_SYMBOLICMOVE);
    }
    if (merilModesFsm_.isStateActive<meril::InstrumentExchangeMode, meril::WaitingForInstrumentExchange>()) {
      // setStatusLights(YELLOW_BLINK);
      setStatusLights(RGB_INSTRUMENTEXCHANGE);
    }
    if (merilModesFsm_.isStateActive<meril::SurgicalMode>()) {
      // setStatusLights(GREEN); // cart is also green
      setStatusLights(RGB_SURGICALMODE);
    }
  }
}
} // namespace logic
