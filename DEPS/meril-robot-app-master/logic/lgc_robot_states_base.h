/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef LGC_ROBOT_STATES_BASE_H
#define LGC_ROBOT_STATES_BASE_H

#include "lgc_robot_states_data.h"
#include <mcx/core.h>

namespace logic {

class SuperState : public mcx::state_machine::State<SuperState> {
public:
  explicit SuperState(StateFSMData& data);

  ~SuperState() override;

  void registerUserEvents() override;

  using mcx::state_machine::State<SuperState>::enter;
  using mcx::state_machine::State<SuperState>::leave;

  virtual mcx::state_machine::EventStatus gotoIdle(double timeoutSec);

  virtual mcx::state_machine::EventStatus waitingIdle();

  virtual mcx::state_machine::EventStatus waitingPause();

  virtual mcx::state_machine::EventStatus gotoOff(double timeoutSec);

  virtual mcx::state_machine::EventStatus waitingOff();

  virtual mcx::state_machine::EventStatus gotoEngaged(double timeoutSec);

  virtual mcx::state_machine::EventStatus waitingEngaged();

  virtual mcx::state_machine::EventStatus savePersistentData();

  virtual mcx::state_machine::EventStatus waitPersistentData();

  virtual mcx::state_machine::EventStatus loadPersistentData();

  virtual mcx::state_machine::EventStatus checkPersistentData();

  virtual mcx::state_machine::EventStatus gotoReferencing(double timeSec);

  virtual mcx::state_machine::EventStatus waitingReferencing(double timeoutSec);

  virtual mcx::state_machine::EventStatus gotoSaveConfiguration(const char* fileName,
                                                                mcx::parameter_server::Parameter* root);

  virtual mcx::state_machine::EventStatus
  waitingSaveConfiguration(const char* fileName, mcx::parameter_server::Parameter* root, int returnStateId);

  virtual mcx::state_machine::EventStatus setNoEStopRelay();

  virtual mcx::state_machine::EventStatus resetEStopRelay();

  virtual mcx::state_machine::EventStatus waitForEcatRecover();

  virtual mcx::state_machine::EventStatus resetEStopErrors();

  mcx::state_machine::EventStatus warning_(const mcx::state_machine::Error& error) override;

  mcx::state_machine::EventStatus forcedDisengaged_(const mcx::state_machine::Error& error) override;

  mcx::state_machine::EventStatus shutdown_(const mcx::state_machine::Error& error) override;

  mcx::state_machine::EventStatus emergencyStop_(const mcx::state_machine::Error& error) override;

  mcx::state_machine::EventStatus acknowledgeErrors() override;

  mcx::state_machine::EventStatus terminateEvent() override;

  void setStatusColor(const RGBLight& statusLight) {
    smData_.bus.out.status_light[0] = statusLight.value[0];
    smData_.bus.out.status_light[1] = statusLight.value[1];
    smData_.bus.out.status_light[2] = statusLight.value[2];
  }

  void setStatusLED(LEDStatus statusLed) { smData_.bus.out.status_led = statusLed; }

protected:
  StateFSMData& smData_;
};
} // namespace logic

#endif /* LGC_ROBOT_STATES_BASE_H */
