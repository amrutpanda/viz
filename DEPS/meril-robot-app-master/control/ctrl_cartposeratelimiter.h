/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef CTRL_CARTPOSERATELIMITER_H
#define CTRL_CARTPOSERATELIMITER_H

#include "mcx/math/math_pose.h"
#include <mcx/control3.h>
#include <mcx/core.h>

class CartPoseRateLimiter final : public mcx::container::Module {
public:
  CartPoseRateLimiter() = default;

  ~CartPoseRateLimiter() override = default;

  void setInput(const mcx::math::CartPose6& newCartPose) { poseRaw_.set(newCartPose); };

  void setInput(const mcx::math::Pose& newPose) { poseRaw_ = newPose; };

  [[nodiscard]] mcx::math::Pose getOutput() const { return poseRateLimited_; };

  [[nodiscard]] bool isLimiting() const { return isLimiting_; };

  void setReference(const mcx::math::CartPose6& reference) { poseReference_.set(reference); };

  void setReference(const mcx::math::Pose& reference) { poseReference_ = reference; };

  void setReset(bool reset) { reset_ = reset; };

  void setTranslationalRate(double rate) { maxTranslationalRate_ = rate; };

  void setRotationalRate(double rate) { maxRotationalRate_ = rate; };

private:
  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override;

  bool initPhase1_() override;

  bool initPhase2_() override;

  bool startOp_() override;

  bool stopOp_() override;

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override;

  mcx::math::Pose poseRaw_{};
  mcx::math::Pose poseRateLimited_{};
  mcx::math::Pose poseReference_{};

  mcx::math::Position velocity_{};

  mcx::math::CartPose6 cartPoseRaw_{};
  mcx::math::CartPose6 cartPoseRateLimited_{};

  double omega_{M_PI * 30};
  double magnitude_{};
  double maxTranslationalRate_{0.1};
  double maxRotationalRate_{10};

  bool isLimiting_{false};

  bool reset_{false};
};

#endif /* CTRL_CARTPOSERATELIMITER_H */
