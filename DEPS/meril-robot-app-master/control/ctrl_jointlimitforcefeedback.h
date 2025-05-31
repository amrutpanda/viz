/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef CTRL_JOINTLIMITFORCEFEEDBACK_H
#define CTRL_JOINTLIMITFORCEFEEDBACK_H

#include "mcx/control3/ctrl_helper.h"
#include <mcx/control3.h>
#include <mcx/core.h>

namespace control {
template <unsigned int numberOfJoints = 6>
class JointPositionLimiterForce final : public mcx::container::Module {
public:
  explicit JointPositionLimiterForce() : numberOfJoints_(numberOfJoints), jointTorquesLimiter_(numberOfJoints) {
    enable_.resize(numberOfJoints);
    scalingVectors_.resize(numberOfJoints);
  };

  ~JointPositionLimiterForce() override = default;

  void setJacobian(const mcx::math::Matrix<6, numberOfJoints>& jacobian) { jacobian_ = jacobian; };

  void setJointPositions(const mcx::math::Vector<numberOfJoints>& jointPositions) { jointPositions_ = jointPositions; };

  void setJointVelocities(const mcx::math::Vector<numberOfJoints>& jointVelocities) {
    jointVelocities_ = jointVelocities;
  };

  void setJointPositionsLimit(const mcx::math::Vector<numberOfJoints>& upperLimits,
                              const mcx::math::Vector<numberOfJoints>& lowerLimits) {
    jointPositionsUpperLimit_ = upperLimits;
    jointPositionsLowerLimit_ = lowerLimits;
  };

  void setEnable(bool value) { surgicalEnabled_ = value; };

  [[nodiscard]] mcx::math::Vector6D getWrench() const { return wrench_; };

  mcx::math::Vector<numberOfJoints> getSpringPositions() { return springPositions_; };

private:
  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override {
    createSubmodule(&jointTorquesLimiter_, "jointTorquesLimiter");
  };

  bool initPhase1_() override {
    using namespace mcx::parameter_server;
    addParameterVec("jointDamping", ParameterType::PARAMETER, jointDamping_);
    addParameterVec("jointStiffness", ParameterType::PARAMETER, jointStiffness_);
    addParameterVec("jointSpringPositionsOffset", ParameterType::PARAMETER, jointSpringPositionsOffset_);
    addParameterVec("enable", ParameterType::PARAMETER, enable_);

    addParameter("twistScaling/enable", ParameterType::PARAMETER, &enableTwistScaling_);
    addParameter("enableStiffnessScaling", ParameterType::PARAMETER, &enableStiffnessScaling_);
    addParameter("enableDampingScaling", ParameterType::PARAMETER, &enableDampingScaling_);
    addParameter("enableLinearForces", ParameterType::PARAMETER, &enableLinearForces_);

    addParameterVec("jointPositions", ParameterType::INPUT, jointPositions_);
    addParameterVec("jointVelocities", ParameterType::INPUT, jointVelocities_);

    addParameterVec("jointTorques", ParameterType::OUTPUT, jointTorques_);
    addParameterVec("jointTorquesLimited", ParameterType::OUTPUT, jointTorquesLimited_);
    addParameterVec("springPositions", ParameterType::OUTPUT, springPositions_);
    addParameterVec("wrench", ParameterType::OUTPUT, wrench_);
    addParameter("rank", ParameterType::OUTPUT, &rank_);

    return true;
  };

  bool initPhase2_() override { return true; };

  bool startOp_() override { return true; };

  bool stopOp_() override { return true; };

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override {
    // only calculate the force if at least it is enabled for one joint
    bool enabled = false;
    springPositions_ = {};
    // calculate joint torques
    for (size_t cnt = 0; cnt < numberOfJoints_; cnt++) {
      bool forceActive = false;
      double springPositionZero;
      if (enable_[cnt]) {
        if (jointPositions_[cnt] >= jointPositionsUpperLimit_[cnt] - jointSpringPositionsOffset_[cnt]) {
          springPositionZero = jointPositionsUpperLimit_[cnt] - jointSpringPositionsOffset_[cnt];
          forceActive = true;
        } else if (jointPositions_[cnt] <= jointPositionsLowerLimit_[cnt] + jointSpringPositionsOffset_[cnt]) {
          springPositionZero = jointPositionsLowerLimit_[cnt] + jointSpringPositionsOffset_[cnt];
          forceActive = true;
        }
      }

      if (forceActive) {
        if (std::fabs(jointSpringPositionsOffset_[cnt]) > std::numeric_limits<double>::epsilon()) {
          double stiffness;
          double damping;
          springPositions_[cnt] = jointPositions_[cnt] - springPositionZero;
          if (enableStiffnessScaling_) {
            double stiffnessFactor = jointStiffness_[cnt] / jointSpringPositionsOffset_[cnt];
            stiffness =
                mcx::control3::limit(std::fabs(springPositions_[cnt] * stiffnessFactor), 0.0, jointStiffness_[cnt]);
          } else {
            stiffness = jointStiffness_[cnt];
          }

          if (enableDampingScaling_) {
            double dampingFactor = jointDamping_[cnt] / jointSpringPositionsOffset_[cnt];
            damping = mcx::control3::limit(std::fabs(springPositions_[cnt] * dampingFactor), 0.0, jointDamping_[cnt]);
          } else {
            damping = jointDamping_[cnt];
          }

          jointTorques_[cnt] = -(damping * jointVelocities_[cnt] + stiffness * springPositions_[cnt]);
        } else {
          jointTorques_[cnt] = -jointDamping_[cnt] * jointVelocities_[cnt];
        }
      } else {
        jointTorques_[cnt] = 0.0;
      }

      enabled |= enable_[cnt];
    }

    // Activate only on surgical is allowed via setEnable()
    enabled &= surgicalEnabled_;

    // limit the joint torques (should always iterate!)
    jointTorquesLimiter_.setInput(jointTorques_);
    jointTorquesLimiter_.iterate(systemTime, userTime);
    jointTorquesLimited_ = jointTorquesLimiter_.getOutput();

    if (enabled) {
      mcx::math::Matrix<numberOfJoints, 6> jacInv{};
      constexpr auto MY_SPECIAL_FLOAT_EPS = std::numeric_limits<float>::epsilon();
      if (numberOfJoints > 6) {
        // right moore-penrose pseudo inverse, in the case of numberOfJoints == 6, the result is the same as inv
        mcx::math::Matrix<numberOfJoints, 6> jacT = jacobian_.transpose();
        mcx::math::Matrix<6, 6> jacP = jacobian_.dot(jacT);
        rank_ = jacP.rank(MY_SPECIAL_FLOAT_EPS);
        if (rank_ == 6) { // full rank
          jacInv = jacT.dot(jacP.inv());
        } // what to do on fail?
      } else {
        // left moore-penrose pseudo inverse, in the case of numberOfJoints == 6, the result is the same as inv
        // todo: test
        mcx::math::Matrix<numberOfJoints, 6> jacT = jacobian_.transpose();
        mcx::math::Matrix<numberOfJoints, numberOfJoints> jacP = jacT.dot(jacobian_);
        rank_ = jacP.rank(MY_SPECIAL_FLOAT_EPS);
        if (rank_ == 6) {
          jacInv = jacP.inv().dot(jacT);
        }
      }
      mcx::math::Matrix<6, numberOfJoints> jacTInv = jacInv.transpose();

      if (enableTwistScaling_) {
        // jacTInv calculates the theoretic wrench that has non-zero values on axes which are zeros in the twist vector,
        // therefore jacTInv needs to be scaled such that the calculated wrench is in parallel to the twist
        twist_ = jacobian_.dot(jointVelocities_);

        for (size_t cntJ = 0; cntJ < numberOfJoints_; cntJ++) {
          if (enable_[cntJ]) {
            for (size_t cntT = 0; cntT < 6; cntT++) {
              if (springPositions_[cntJ] > 0) {
                scalingVectors_[cntJ][cntT] += twist_[cntT] * jacInv(cntJ, cntT);
              } else if (springPositions_[cntJ] < 0) {
                scalingVectors_[cntJ][cntT] -= twist_[cntT] * jacInv(cntJ, cntT);
              } else {
                scalingVectors_[cntJ][cntT] = 0;
              }

              if (scalingVectors_[cntJ][cntT] < 0) {
                scalingVectors_[cntJ][cntT] = 0;
              }
            }
          } else {
            scalingVectors_[cntJ] = 0;
          }
        }

        for (size_t cntJ = 0; cntJ < numberOfJoints_; cntJ++) {
          if (enable_[cntJ]) {
            mcx::math::Vector6D scalingVectorNormalized = scalingVectors_[cntJ];
            scalingVectorNormalized.normalize();
            for (size_t cntT = 0; cntT < 6; cntT++) {
              scalingMatrix_(cntJ, cntT) = scalingVectorNormalized[cntT];
              jacTInv(cntT, cntJ) *= scalingVectorNormalized[cntT];
            }
          }
        }
      }

      wrench_ = jacTInv.dot(jointTorquesLimited_);

      if (!enableLinearForces_) {
        for (size_t cnt = 0; cnt < 3; cnt++) {
          wrench_[cnt] = 0;
        }
      }
    } else {
      wrench_ = 0;
    }

    return true;
  };

  // variables
  unsigned int numberOfJoints_;

  mcx::control3::Limiter jointTorquesLimiter_;

  int rank_{};

  std::valarray<bool> enable_{};
  bool surgicalEnabled_{};
  bool enableTwistScaling_{};
  bool enableStiffnessScaling_{};
  bool enableDampingScaling_{};
  bool enableLinearForces_{};
  mcx::math::Vector<numberOfJoints> jointDamping_{};
  mcx::math::Vector<numberOfJoints> jointStiffness_{};
  mcx::math::Vector<numberOfJoints> jointTorques_{};
  mcx::math::Vector<numberOfJoints> jointTorquesLimited_;
  mcx::math::Vector<numberOfJoints> jointPositionsUpperLimit_;
  mcx::math::Vector<numberOfJoints> jointPositionsLowerLimit_;
  mcx::math::Vector<numberOfJoints> jointSpringPositionsOffset_{};
  mcx::math::Vector<numberOfJoints> jointPositions_{};
  mcx::math::Vector<numberOfJoints> jointVelocities_{};
  mcx::math::Vector<numberOfJoints> springPositions_{};
  mcx::math::Vector6D wrench_{};
  mcx::math::Vector6D twist_{};

  mcx::math::Matrix<6, numberOfJoints> jacobian_{};
  std::valarray<mcx::math::Vector6D> scalingVectors_{};
  mcx::math::Matrix<numberOfJoints, 6> scalingMatrix_{};
};
} // end namespace control

#endif /* CTRL_JOINTLIMITFORCEFEEDBACK_H */
