/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef CTRL_CARTESIANPLANECONSTRAINT_H
#define CTRL_CARTESIANPLANECONSTRAINT_H

#include "ctrl_cartesianposeconstraint.h"

class CartesianPlaneConstraint final : public CartesianPoseConstraint {
public:
  CartesianPlaneConstraint() = default;

  explicit CartesianPlaneConstraint(const std::string& constraintName);

  ~CartesianPlaneConstraint() override = default;

protected:
  void calculateConstraint() override;

  void calculateLimitingVector() override;

  void calculateConvertedConstrainedPosition() override;

  bool initPhase1_() override;
};

#endif /* CTRL_CARTESIANPLANECONSTRAINT_H */
