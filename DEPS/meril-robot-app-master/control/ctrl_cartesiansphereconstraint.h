/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef CTRL_CARTESIANSPHERECONSTRAINT_H
#define CTRL_CARTESIANSPHERECONSTRAINT_H

#include "ctrl_cartesianposeconstraint.h"
#include <mcx/core.h>

class CartesianSphereConstraint : public CartesianPoseConstraint {
public:
  CartesianSphereConstraint() = default;

  explicit CartesianSphereConstraint(const std::string& constraintName);

  ~CartesianSphereConstraint() override = default;

private:
  void calculateConstraint() final;

  void calculateLimitingVector() final;

  void calculateConvertedConstrainedPosition() final;

  bool initPhase1_() override;
};

#endif /* CTRL_CARTESIANSPHERECONSTRAINT_H */
