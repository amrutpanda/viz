/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef CTRL_FULCRUMTEACHING_H
#define CTRL_FULCRUMTEACHING_H

#include "mcx/math/math_pose.h"
#include <mcx/core.h>

/**
 * @brief Manages the fulcrum teaching process, focusing on the calculation and tracking
 *        of the fulcrum point based on manipulator movements and local link poses.
 *
 * The FulcrumTeaching class is a module designed to iteratively determine the fulcrum pose
 * for a manipulator by analyzing the positional and orientational changes over time. It
 * incorporates functionality to set and update manipulator and link poses, tracks calculation
 * progress, and outputs the final fulcrum position and orientation upon completion.
 *
 * The main responsibilities of the class include:
 * - Storing and updating the current and previous poses of the manipulator.
 * - Setting and tracking the local pose of a fixed link of the manipulator.
 * - Iteratively calculating valid fulcrum points based on manipulator motion.
 * - Managing internal states and parameters for computing and finalizing a consistent fulcrum pose.
 */
class FulcrumTeaching final : public mcx::container::Module {
public:
  FulcrumTeaching() = default;

  ~FulcrumTeaching() override = default;

  /**
   * @brief Updates the pose of the manipulator in the six-dimensional space.
   *
   * This method assigns a new pose, represented as a 6D vector, to the manipulator.
   * The pose typically includes positional and orientational components necessary
   * for defining the manipulator's configuration in space.
   *
   * @param manipulatorPoseNew A 6D vector representing the new pose of the manipulator.
   */
  void setNewManipulatorPose(const mcx::math::Vector6D& manipulatorPoseNew) {
    manipulatorPoseNew_ = manipulatorPoseNew;
  };

  /**
   * @brief Sets the local pose of a link relative to its parent frame.
   *
   * This method assigns a new pose, described as a 3D vector, to the selected link in the local coordinate system.
   * The local pose is critical in expressing the position and orientation of the link in relation to its parent.
   *
   * @param linkPoseLocal The 3D vector representing the local pose of the link.
   */
  void setLinkPoseLocal(const mcx::math::Vector3D& linkPoseLocal) { linkPoseLocal_ = linkPoseLocal; }

private:
  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override;

  bool initPhase1_() override;

  bool initPhase2_() override;

  bool startOp_() override;

  bool stopOp_() override;

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override;

  /**
   * @brief A boolean flag that determines whether the fulcrum teaching algorithm is engaged.
   *
   * When set to true, the system actively performs calculations in each iteration to determine
   * the fulcrum pose based on manipulator motion data. When the necessary computations are completed
   * or a certain threshold of calculated points is reached, the flag is reset to false.
   *
   * This variable is particularly important in controlling the behavior of the fulcrum teaching process,
   * ensuring the system only performs the required operations when actively engaged.
   *
   * Default value: false
   */
  bool engage_{false};
  /**
   * @brief Stores the number of valid calculated fulcrum points during the fulcrum teaching process.
   *
   * This variable keeps track of the number of valid calculations made while determining
   * the fulcrum position and pose. A valid point is considered when specific conditions
   * on distances and proportions are met during the iterative calculation process.
   *
   * - Incremented each time a valid fulcrum point is computed.
   * - Reset to 0 once the number of calculated points reaches a certain threshold (e.g., 10000),
   *   after which the computed fulcrum pose is finalized and other relevant internal states are reset.
   *
   * Used internally within the iterative operation process (`FulcrumTeaching::iterateOp_`)
   * to evaluate the state and completion criteria of the fulcrum teaching computation.
   */
  int numberOfCalculatedPoints_{};
  /**
   * @brief Represents the calculated fulcrum pose in a 6D vector format.
   *
   * This variable stores the pose of the calculated fulcrum, including its position and orientation.
   * It is updated during the iterative operation phase of the fulcrum teaching process and holds
   * the latest calculated fulcrum position and orientation once certain conditions are met.
   *
   * @details
   * - The pose is calculated based on the movement of the manipulator in previous and current states,
   *   combined with input parameters such as instrument link pose and vectors derived from
   *   manipulator's rotation matrices.
   * - The calculation is performed only if significant movement of the manipulator is detected
   *   (i.e., the norm of the movement increment exceeds a defined threshold `EPS`).
   * - The calculation ensures the derived point lies within a valid range based on predefined constraints.
   * - Once the number of calculated points reaches a threshold (e.g., 10,000), the fulcrum pose is
   *   finalized and recorded in this variable.
   *
   * This variable is also registered as an output parameter during the initialization phase and can
   * be accessed as part of the fulcrum teaching module's output.
   */
  mcx::math::Vector6D calculatedFulcrumPose_{};
  /**
   * @brief Represents the local position of a link in the manipulator's coordinate system.
   *
   * The vector `linkPoseLocal_` is used to retain the position of a fixed link
   * with respect to the manipulator's local coordinate frame. It is essential in
   * determining the spatial relationship between the link and the manipulator's
   * motion in the inertial reference frame during iterative operations.
   *
   * This variable plays a critical role in computing the incremental changes
   * in position of the manipulator and is used to assess the movement of the
   * link over subsequent operations, especially in scenarios involving position
   * adjustments and fulcrum calculations.
   */
  mcx::math::Vector3D linkPoseLocal_{};

  /**
   * @brief Accumulates the cumulative calculated positions for determining the fulcrum point
   *        in the fulcrum teaching process.
   *
   * This variable is used to sum the position vectors derived from valid data points during
   * the fulcrum teaching operation. The accumulated value, along with the total number of
   * valid data points, is later used to compute the average position, representing the
   * estimated fulcrum point. It is reset when the calculation is complete or invalidated.
   *
   * @see FulcrumTeaching::iterateOp_
   */
  mcx::math::Vector3D sumCalculatedPosition_{};
  /**
   * @brief Stores the current pose of a manipulator.
   *
   * This variable represents the six degrees of freedom (position and orientation)
   * of a manipulator in terms of a 6-dimensional vector. The position is represented
   * by the first three elements (x, y, z), and the orientation is represented
   * by the remaining three elements (phi, theta, psi or similar).
   *
   * It is used for calculating the movement of the manipulator and comparing its
   * new position and orientation to previous states for operations such as fulcrum
   * teaching and precise trajectory adjustments.
   */
  mcx::math::Vector6D manipulatorPoseNew_{};
  /**
   * @brief Stores the previous pose of the manipulator in a 6-dimensional vector.
   *
   * This variable is used to record the pose of the manipulator from the previous iteration in
   * operations involving manipulator motion tracking, such as fulcrum teaching.
   * It consists of 3 translational and 3 rotational components, with the specific convention
   * depending on the implementation of the `math::Vector6D` type.
   *
   * The translational components (indices 0, 1, and 2) represent the manipulator's position in
   * inertial coordinates.
   * The rotational components (indices 3, 4, and 5) encode the orientation of the manipulator,
   * typically in the form of Euler angles or another rotational representation defined by the
   * `math::Rotation` type.
   *
   * During the iterative operation, the current manipulator pose is compared to the previously
   * stored pose to compute motion increments and perform calculations for fulcrum positioning.
   *
   * This variable is updated at the end of each iteration to reflect the latest manipulator pose
   * for subsequent comparisons.
   */
  mcx::math::Vector6D manipulatorPoseOld_{};
};

#endif /* CTRL_FULCRUMTEACHING_H */
