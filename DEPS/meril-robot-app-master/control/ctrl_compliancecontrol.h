/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef MCX_CTRL_COMPLIANCE_CONTROL_H
#define MCX_CTRL_COMPLIANCE_CONTROL_H

#include "ctrl_types.h"
#include "mcx/control3/ctrl_admittancemodel.h"
#include <mcx/control3.h>
#include <mcx/core.h>

/**
 * @brief The force position controller block for Admittance and Impedance controllers

 *
 * @Admittance controller
 *
 *
 * @Impedance controller
 * Impedance control is an approach to dynamic control relating force and position. It is often used in applications
 * where a manipulator interacts with its environment and the force position relation is of concern. Examples of such
 * applications include humans interacting with robots, where the force produced by the human relates to how fast the
 * robot should move/stop. Simpler control methods, such as position control or torque control, perform poorly when
 * the manipulator experiences contacts. Thus, impedance control is commonly used in these settings.
 *
 * Mechanical impedance is the ratio of force output to motion input. This is analogous to electrical impedance that
 * is the ratio of voltage output to current input (e.g. resistance is voltage divided by current). A "spring constant"
 * defines the force output for a displacement (extension or compression) of the spring. A "damping constant" defines
 * the force output for a velocity input. If we control the impedance of a mechanism, we are controlling the force of
 * resistance to external motions that are imposed by the environment.
 *
 * Mechanical admittance is the inverse of impedance - it defines the motions that result from a force input. If a
 * mechanism applies a force to the environment, the environment will move, or not move, depending on its properties
 * and the force applied. For example, a marble sitting on a table will react very differently to a given force than
 * will a log floating in a lake.
 *
 * @param[in] channelEnable[·] - boolean input if the channel is active
 * @param[out] output[·] - the output signal
 * @param fadeTime - time (in seconds) that it takes to fade out (from On to Off)
 *
 */
namespace control {

class ComplianceControl final : public mcx::container::Module {
  using AdmittanceModels = std::vector<mcx::control3::AdmittanceModel>;

public:
  ComplianceControl() = delete;

  explicit ComplianceControl(unsigned int numberOfChannels = 1);

  inline void setEnableAdmittanceControl(bool newValue) { enableAdmittanceControl_ = newValue; };

  /**@brief Sets the joint limit positions in the admittance controller (integrator output min/max and
   * positionLimitLookup.setX) */
  void setLimitPositions(const mcx::control3::JointPositions& lowerPositionLimit,
                         const mcx::control3::JointPositions& upperPositionLimit, const double springLimitingFactor) {
    for (unsigned int cnt = 0; cnt < numberOfChannels_; ++cnt) {
      admittanceModels_[cnt].setLimitPositions(lowerPositionLimit[cnt], upperPositionLimit[cnt], springLimitingFactor);
    }
  }
  void setMeasuredTorques(const JointTorques& newValue) { measuredTorque_ = newValue; };
  void setStaticTorques(const JointTorques& newValue) { inputTorque_ = newValue; };
  void setReferencePVA(const JointPVAIn& newValue) { referencedPVA_ = newValue; };
  void setActualPVA(const JointPVAIn& newValue) { actualPVA_ = newValue; };
  void setVirtualMass(const JointTorques& newValue) { virtualMass_ = newValue; };
  void setReferencePositions(const JointPositions& newValue) { referencedPVA_.pos = newValue; };
  void setReferenceVelocities(const JointVelocities& newValue) { referencedPVA_.vel = newValue; };
  void setInputPVA(const JointPVAIn& newValue) { inputPVA_ = newValue; };
  void setEnableAdmittanceController(mcx::utils::span<const bool> newValues) {
    size_t index = 0;
    for (auto& el : admittanceModels_) {
      if (index < newValues.size()) {
        el.setEnable(newValues[index++]);
      }
    }
  }
  void setEnableAdmittanceController(const bool newValue, unsigned int index) {
    admittanceModels_[index].setEnable(newValue);
  };

  void setResetPosition(bool reset) { resetPosition_ = reset; };
  void setResetVelocity(bool reset) { resetVelocity_ = reset; };
  void setDisableDynamics(bool disableDynamics) { disableDynamics_ = disableDynamics; };

  /**
   * @brief ..
   * @return ..
   */
  [[nodiscard]] const JointPVAOut& getOutputPVA() const { return outputPVA_; }
  [[nodiscard]] const JointPositions& getOutputPositions() const { return outputPVA_.pos; }
  [[nodiscard]] const JointVelocities& getOutputVelocities() const { return outputPVA_.vel; }
  [[nodiscard]] const JointTorques& getOutputTorques() const { return outputTorque_; }

private:
  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override;

  bool initPhase1_() override;

  bool initPhase2_() override;

  bool startOp_() override;

  bool stopOp_() override;

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override;

  unsigned int numberOfChannels_{1};

  JointPVAIn referencedPVA_{};
  JointPVAIn actualPVA_{};
  JointPVAIn inputPVA_{};
  JointPVAOut outputPVA_{};

  JointTorques virtualMass_{};
  JointTorques measuredTorque_{};
  JointTorques inputTorque_{};
  JointTorques outputTorque_{};

  bool enableAdmittanceControl_{false};
  bool resetPosition_{false};
  bool resetVelocity_{false};
  bool disableDynamics_{false};

  AdmittanceModels admittanceModels_{};
};

} // namespace control

#endif /* MCX_CTRL_COMPLIANCE_CONTROL_H */
