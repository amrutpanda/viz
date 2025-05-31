/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef CTRL_SEMIAUTO_H
#define CTRL_SEMIAUTO_H

#include "ctrl_types.h"
#include <mcx/core.h>
#include <mcx/motion/mtn_interpreter_def.h>
#include <mcx/motion/mtn_joint_p2p.h>

namespace control {

class Mechanism;

class SemiautoGenerator final : public mcx::container::Module {
public:
  explicit SemiautoGenerator(unsigned int numberOfJoints);

  ~SemiautoGenerator() override = default;

  void setActualJointPositions(mcx::utils::span<const double> actualJointPositions) {
    mcx::utils::span<double>{posActual_} = actualJointPositions;
  }

  void setTargetJointPositions(mcx::utils::span<const double> targetJointPositions) {
    mcx::utils::span<double>{posTarget_} = targetJointPositions;
  }

  void setRunSpeedFactor(double runSpeedFactor) { runSpeedFactor_ = runSpeedFactor; }

  void setEngaged(bool engaged) { engaged_ = engaged; }

  [[nodiscard]] mcx::utils::span<const double> getJointSetpoints() const { return posSetpoint_; }

  [[nodiscard]] mcx::motion::MotionGeneratorStates getMotionState() const { return state_; }

private:
  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) final {}

  bool initPhase1_() override;

  bool initPhase2_() override { return true; }

  bool startOp_() override { return true; }

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) final;

  bool stopOp_() override { return true; }

private:
  mcx::motion::MotionGeneratorStates state_{mcx::motion::MotionGeneratorStates::WAITING_MOVE_CMD_S};

  bool engaged_{};
  bool engagedPrev_{};

  double runSpeedFactor_{1.0};
  double localTime_{0.0};
  mcx::parameter_server::ParamHandle targetHandle_;

  JointPositions posActual_;
  JointPositions posTarget_;
  JointPositions posSetpoint_;
  JointVelocities velSetpoint_;
  JointAccelerations accSetpoint_;

  double maxVelocity_{0.5};
  double maxAcceleration_{5.0};

  mcx::motion::TrajJointsP2P trajJointP2p_;
};

} // namespace control

#endif
