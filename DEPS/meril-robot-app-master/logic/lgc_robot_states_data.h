/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_ROBOT_STATES_DATA_H
#define LGC_ROBOT_STATES_DATA_H

#include "lgc_homing_def.h"
#include "lgc_motion_def.h"
#include <mcx/control3.h>
#include <mcx/core.h>

namespace logic {
struct RGBLight {
  double value[3]{};
  int led{};
};

enum LEDStatus : int {
  // States
  IDLE = 0, // (Blue quick blinking)
  GREEN = 1,
  RED = 2,
  YELLOW = 3,
  BLUE = 4,
  PINK = 9,
  ORANGE = 10,
  WHITE = 11,

  // Transition States
  GREEN_BLINK = 5,
  RED_BLINK = 6,
  YELLOW_BLINK = 7,
  BLUE_BLINK = 8,
  PINK_BLINK = 12,
  ORANGE_BLINK = 13,
  WHITE_BLINK = 14,

  // Special
  GREEN_FAST_BLINK = 15
};

#define MAXDOMAINS 5
struct DomainErrors {
  bool error[MAXDOMAINS]{};
};
/*
 * | No. | EK1914 out1 | EK1914 out2 | EK1914 out3 | Colour | Mode |
 * | --- | --- | --- | --- | ---                 | --- |
 * | 1   | 0   | 0   | 0   | Default (sky blue)  | IDLE |
 * | 2   | 0   | 0   | 1   | Blue                | LOCKED |
 * | 3   | 0   | 1   | 0   | Purple              | HAND GUIDING |
 * | 4   | 0   | 1   | 1   | White               | FULCRUM TEACH |
 * | 5   | 1   | 0   | 0   | White blink         | INSTRUMENT CALIBRATION |
 * | 6   | 0   | 1   | 0   | Purple              | HAND GUIDING |
 * | 7   | 1   | 0   | 1   | Green               | SURGICAL MODE |
 * | 8   | 0   | 0   | 1   | Blue                | LOCKED |
 * | 9   | 1   | 1   | 0   | Purple blink        | INSTRUMENT EXCHANGE |
 * | 10  | 1   | 1   | 1   | Red blink           | ERROR |
 */

static constexpr RGBLight RGB_IDLE{0, 0, 0, LEDStatus::IDLE};
static constexpr RGBLight RGB_LOCKED{0, 0, 1, LEDStatus::BLUE};
static constexpr RGBLight RGB_SYMBOLICMOVE{0, 1, 1, LEDStatus::YELLOW};
static constexpr RGBLight RGB_HANDGUIDING{0, 1, 0, LEDStatus::PINK};
static constexpr RGBLight RGB_HANDGUIDINGLINEAR{0, 1, 0, LEDStatus::PINK};
static constexpr RGBLight RGB_FULCRUMTEACH{0, 1, 1, LEDStatus::WHITE};
static constexpr RGBLight RGB_INSTRUMENTCALIBRATION{1, 0, 0, LEDStatus::WHITE_BLINK};
static constexpr RGBLight RGB_SURGICALMODE{1, 0, 1, LEDStatus::GREEN};
static constexpr RGBLight RGB_INSTRUMENTEXCHANGE{1, 1, 0, LEDStatus::PINK_BLINK};
static constexpr RGBLight RGB_ERROR{1, 1, 1, LEDStatus::RED_BLINK};
static constexpr RGBLight RGB_RESETERROR{1, 1, 1, LEDStatus::RED};
// States <-> Bus

struct StateToBus {
  BEGIN_VISITABLES(StateToBus);
  VISITABLE(bool, do_no_estop);
  VISITABLE(bool, reset_estop);
  VISITABLE(bool, run_safety_plc);
  VISITABLE(decltype(RGBLight::value), status_light);
  VISITABLE(int, status_led);
  END_VISITABLES;
};

struct BusToState {
  BEGIN_VISITABLES(BusToState);
  VISITABLE(bool, ecat_bus_error);
  VISITABLE(decltype(DomainErrors::error), ecat_domain_error);
  VISITABLE(bool, estop_not_active);
  VISITABLE(bool, circuitbreaker_us_tripped);
  VISITABLE(bool, circuitbreaker_up_tripped);
  END_VISITABLES;
};

struct StateFSMData {

  struct {
    mcx::control3::ControlToState in{};
    mcx::control3::StateToControl out{};
    bool disablePVALimiter{};

    static mcx::control3::ControlToState performAnd(const mcx::control3::ControlToState& lhs,
                                                    const mcx::control3::ControlToState& rhs) {
      return mcx::control3::ControlToState{.isAtEngaged = lhs.isAtEngaged && rhs.isAtEngaged,
                                           .isAtIdle = lhs.isAtIdle && rhs.isAtIdle,
                                           .isAtPause = lhs.isAtPause && rhs.isAtPause,
                                           .isAtSmoothstop = lhs.isAtSmoothstop && rhs.isAtSmoothstop,
                                           .isSmoothstopActive = lhs.isSmoothstopActive && rhs.isSmoothstopActive,
                                           .isReferenced = lhs.isReferenced && rhs.isReferenced};
    }
  } ctrl;

  struct {
    BusToState in;
    StateToBus out;
    bool busHasError{false};
  } bus;

  struct {
    mcx::control3::StateToReferencing out;
  } referencing;

  struct {
    double setNoEstopSec{2};
    double resetEstopSec{2};
    double ecatRecoverTimeoutSec{60};
    double resetSlavesSec{1.0};
    double gotoIdleDelaySec{0};
    double gotoEngageDelaySec{0};
    double referenceTimeSec{3.0};
    double waitingIdleSec{5.0};
    double waitingPauseSec{5.0};
    double saveTimeSec{5};
    double persistenceSaveDelay{0.5};
    double persistenceLoadDelay{0.5};
    double persistenceTimeout{3.0};
  } timings;

  States actualState{States::INIT_S};
  StateEvents actualEvent{StateEvents::DO_NOTHING_E};

  mcx::parameter_server::ParamHandle persistenceCommandOutHandle;
  mcx::control3::PersistenceEvents persistenceCommandOut{mcx::control3::PersistenceEvents::NONE};
  mcx::parameter_server::FileSerializationXml fileSerialization;
  double engagedStateTimer{};
};

} // namespace logic

#endif /* LGC_ROBOT_STATES_DATA_H */
