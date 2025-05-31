/*
 * All rights reserved. Copyright (c) 2014-2022 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef MOTORCORTEX_ROBOT_CONTROL_CTRL_TYPES_H
#define MOTORCORTEX_ROBOT_CONTROL_CTRL_TYPES_H

#include <mcx/math.h>
#include <mcx/mechanics.h>

namespace control {

template <unsigned int rows>
using Vector = mcx::math::Vector<rows>;
using Quaternion = mcx::math::Quaternion;
using CartPose6 = mcx::math::CartPose6;
using Twist = mcx::math::Twist;
using Wrench = mcx::math::Wrench;

using JointPositions = mcx::mechanics::JointPositions;
using JointVelocities = mcx::mechanics::JointVelocities;
using JointAccelerations = mcx::mechanics::JointAccelerations;
using JointTorques = mcx::mechanics::JointTorques;
using Wrenches = mcx::mechanics::Wrenches;

using JointPVAIn = mcx::mechanics::JointPVAIn;
using JointPVAOut = mcx::mechanics::JointPVAIn;
using CartPoseOut = mcx::mechanics::CartPoseOut;
using JointTorqueOut = mcx::mechanics::JointTorqueOut;

inline JointPositions getSyncJointOutput(const std::valarray<bool>& isOpenLoop, const bool syncPositionLoop,  const JointPositions& actual,
                                         const JointPositions& target) {
  JointPositions syncJointOutput(isOpenLoop.size());
  for (unsigned int i = 0; i < isOpenLoop.size(); i++) {
    syncJointOutput[i] = isOpenLoop[i] || syncPositionLoop ? actual[i] : target[i];
  }
  return syncJointOutput;
}

inline bool isAnyPositionLoopOpen(const std::valarray<bool>& isOpenLoop) {
  return std::ranges::any_of(isOpenLoop, [](bool isOpen) { return isOpen; });
}

} // namespace control

#endif // MOTORCORTEX_ROBOT_CONTROL_CTRL_TYPES_H
