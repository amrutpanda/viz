/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef CTRL_CARTIANPOSECONSTRAINT_H
#define CTRL_CARTIANPOSECONSTRAINT_H

#include <mcx/control3.h>
#include <mcx/core.h>

/**
 * @class CartesianPoseConstraint
 * @brief Defines a module for applying and managing a Cartesian pose constraint with braking and limiting capabilities.
 *
 * This class implements a constraint system to manage the Cartesian target poses, with built-in mechanisms
 * for constraint violations, braking, and limiting. The constraint can be dynamically adjusted and updated
 * with various properties.
 */
class CartesianPoseConstraint : public mcx::container::Module {
public:
  enum ConstraintDirection { STAY_INSIDE = 0, STAY_OUTSIDE = 1 };

  /// Default enable
  static constexpr bool DEFAULT_ENABLE = true;

  /// Default disable
  static constexpr bool DEFAULT_DISABLE = false;

  /**
   * @brief Creates a new cartesian pose constraint Object
   */
  CartesianPoseConstraint() = default;

  explicit CartesianPoseConstraint(std::string constraintName);

  ~CartesianPoseConstraint() override = default;

  mcx::math::Vector6D updateConstraint(const mcx::math::Vector6D& rawPose);

  [[nodiscard]] mcx::math::Vector6D getReferencePose() const { return referencePose_; }

  void setReferencePose(const mcx::math::Vector6D& referencePose) { referencePose_ = referencePose; }

  /** Set disable working of object.  */
  void setDisable(bool disable) { disable_ = disable; }

  [[nodiscard]] bool isViolating() const { return isViolating_; }

  [[nodiscard]] bool isBraking() const { return isBraking_; };

  [[nodiscard]] bool isLimiting() const { return isLimiting_; };

  std::string getName() { return constraintName_; }

  void setConstraintBoundary(double constraintBoundary) { constraintBoundary_ = constraintBoundary; };

  [[nodiscard]] double getConstraintBoundary() const { return constraintBoundary_; };

  [[nodiscard]] double getConvertedConstrainedPosition() const { return constrainedPositionConverted_; };

  [[nodiscard]] double getBrakingPoint() const { return brakingPoint_; };

  [[nodiscard]] double getBrakingRange() const { return brakingRange_; };

  [[nodiscard]] bool isEnabled() const { return isEnabled_; };

  [[nodiscard]] ConstraintDirection getConstraintDirection() const { return constraintDirection_; }

  /**
   * @brief Get the current limiting vector. The limiting vector is a normalized vector that indicates the
   * 3D direction in which the constraint is currently correcting the raw target pose
   */
  [[nodiscard]] mcx::math::Vector3D getLimitingVector() const { return limitingVector_; };

protected:
  virtual void calculateConstraint() = 0;

  virtual void calculateLimitingVector() = 0;

  virtual void calculateConvertedConstrainedPosition() = 0;

  void updateSoftBraking(double newPositionConverted);

  [[nodiscard]] mcx::math::Vector6D getRawPose() const { return rawPose_; }

  [[nodiscard]] mcx::math::Vector6D getConstrainedPose() const { return constrainedPose_; }

  [[nodiscard]] mcx::math::Vector6D getJumpCorrectedPose() const { return jumpCorrectedPose_; }

  void setConstrainedPose(const mcx::math::Vector6D& constrainedPose) { constrainedPose_ = constrainedPose; }

  void setViolating(bool isViolating) { isViolating_ = isViolating; }

  void setLimitingVector(const mcx::math::Vector3D& limitingVector) { limitingVector_ = limitingVector; };

  void setConvertedConstrainedPosition(const double convertedPosition) {
    constrainedPositionConverted_ = convertedPosition;
  };

  [[nodiscard]] double getMaxPositionJump() const { return maxPositionJump_; };

  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override;

  bool initPhase1_() override;

  bool initPhase2_() override;

  bool startOp_() override;

  bool stopOp_() override;

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override;

private:
  // parameter
  // soft braking parameters
  double brakingRange_{};
  double maxPositionJump_{};
  bool enableSoftBraking_{false};

  // object PIO
  bool enable_{DEFAULT_ENABLE};
  bool disable_{DEFAULT_DISABLE};
  bool isEnabled_{DEFAULT_ENABLE && !DEFAULT_DISABLE};
  ConstraintDirection constraintDirection_{STAY_INSIDE};
  double constraintBoundary_{};
  double constrainedPositionConverted_{};
  mcx::math::Vector6D jumpCorrectedPose_{};
  bool isBraking_{false};
  bool isViolating_{false};
  bool isLimiting_{false};
  mcx::math::Vector3D limitingVector_{};
  bool isInBrakingArea_{false};

  // constraint parameters
  mcx::math::Vector6D referencePose_{};
  mcx::math::Vector6D rawPose_{};
  mcx::math::Vector6D constrainedPose_{};

  // local variables
  std::string constraintName_{"cartPoseConstraint"};
  bool brakingInitialized_{false};
  double brakingPoint_{};
};

#endif /* CTRL_CARTIANPOSECONSTRAINT_H */
