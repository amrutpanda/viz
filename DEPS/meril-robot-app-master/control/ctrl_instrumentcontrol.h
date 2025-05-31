/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef CTRL_INSTRUMENTCONTROL_H
#define CTRL_INSTRUMENTCONTROL_H

#include "ctrl_cartesianplaneconstraint.h"
#include <mcx/control3.h>
#include <mcx/core.h>

namespace control {
class InstrumentControl final : public mcx::container::Module {
public:
  static constexpr double INSTRUMENT_STRAIGHTEN_THRESHOLD = 1e-3;
  static constexpr double DEFAULT_MAX_INSERTION_DEPTH = 0.38; // [m]
  static constexpr double DEFAULT_MIN_INSERTION_DEPTH = 0.05;
  static constexpr double INSTRUMENT_INSERTION_SPEED_THRESHOLD = 1e-6; // per sample, corresponds to 1e-3m/s at 1000Hz
  static constexpr size_t NR_OF_INSTRUMENT_JOINTS = 4;
  static constexpr uint PITCH_JOINT = 0;
  static constexpr uint YAW_JOINT = 1;
  static constexpr uint PINCH_JOINT = 2;
  static constexpr uint ROLL_JOINT = 3;

  enum fulcrumTeachSetting { MANUAL_INPUT = 0, LINK1TIP = 1, LINK2TIP = 2, LINK3TIP = 3};

  InstrumentControl();

  ~InstrumentControl() override = default;

  /**
   * @brief Updates the inverse kinematics for the instrument control system.
   *
   * Computes and updates the instrument's joint positions and corrected tool pose
   * based on the desired target pose, while ensuring the tool pose of the manipulator
   * aligns with the adjusted instrument pose.
   *
   * @param instrumentToolPoseTarget The desired target pose for the instrument's tool.
   * @param manipulatorToolPoseTarget Reference to the tool pose of the manipulator to be updated.
   * @param instrumentJointPositionsTarget Reference to the instrument's joint positions to be updated.
   * @param instrumentToolPoseCorrectedTarget Reference to hold the corrected target pose for the instrument's tool after
   * applying constraints.
   */
  void updateInverseKin(const mcx::math::CartPose6& instrumentToolPoseTarget,
                        mcx::math::CartPose6& manipulatorToolPoseTarget,
                        mcx::control3::JointPositions& instrumentJointPositionsTarget,
                        mcx::math::CartPose6& instrumentToolPoseCorrectedTarget);

  /**
   * @brief Updates the forward kinematics for the instrument tool.
   *
   * Computes and updates the actual pose of the instrument tool based on the specified
   * manipulator tool pose and the current joint positions of the instrument.
   *
   * @param manipulatorToolPoseActual The current pose of the manipulator's tool, expressed as a 6D Cartesian pose.
   * @param instrumentJointPositionsActual The current joint positions of the instrument.
   * @param instrumentToolPoseActual The resulting pose of the instrument's tool, expressed as a 6D Cartesian pose,
   * updated to reflect the latest calculations.
   */
  void updateForwardKin(const mcx::math::CartPose6& manipulatorToolPoseActual,
                        const mcx::control3::JointPositions& instrumentJointPositionsActual, // inputs
                        mcx::math::CartPose6& instrumentToolPoseActual);

  /**
   * @brief Checks if the fulcrum position is valid.
   *
   * Determines whether the current fulcrum position is considered valid based on
   * the internal state of the system.
   *
   * @return True if the fulcrum position is valid, false otherwise.
   */
  [[nodiscard]] bool isFulcrumValid() const { return fulcrumValid_; };

  /**
   * @brief Enables or disables the teach fulcrum functionality.
   *
   * Sets the teach fulcrum state based on the provided boolean value, adjusting
   * the internal state accordingly.
   *
   * @param teachFulcrum A boolean value indicating whether to enable (true) or
   * disable (false) the teach fulcrum.
   */
  void setTeachFulcrum(const bool teachFulcrum) { teachFulcrum_ = teachFulcrum; };

  /**
   * @brief Retrieves the pose of the fulcrum.
   *
   * Returns the current 6D Cartesian pose representing the fulcrum's position and orientation.
   *
   * @return The fulcrum pose as a 6D Cartesian pose.
   */
  [[nodiscard]] mcx::math::CartPose6 getFulcrumPose() const { return fulcrumPose_; };

  /**
   * Sets the global position of the fulcrum port.
   *
   * This method assigns a given position to the fulcrum port in global coordinates.
   *
   * @param position The global position to set for the fulcrum port.
   */
  void setFulcrumPortPosition(mcx::math::Position position) { fulcrumPortPosition_ = position; };

  /**
   * Retrieves the fulcrum position delta.
   *
   * @return The difference in position associated with the fulcrum port as a mcx::math::Position object.
   */
  [[nodiscard]] mcx::math::Position getFulcrumPositionOffset() const { return fulcrumPortPositionOffset_; };

  /**
   * Retrieves the position delta of a fulcrum at the specified index.
   *
   * This method is marked as [[nodiscard]], indicating that the returned value
   * should not be discarded when the method is called.
   *
   * @param index The index of the fulcrum whose position delta is to be retrieved.
   * @return The position delta of the fulcrum at the given index.
   * @throws std::out_of_range if the index is invalid or out of bounds.
   */
  [[nodiscard]] double getFulcrumPositionOffset(unsigned int index) const { return fulcrumPortPositionOffset_[index]; };

  /**
   * @brief Calculates the error in the fulcrum position for the current configuration.
   *
   * Determines the deviation of the current fulcrum position from its desired or expected value.
   * This is used to assess and correct the alignment of the fulcrum during operation.
   *
   * @return The calculated error in the fulcrum position.
   */
  [[nodiscard]] double getFulcrumPositionError() const { return fulcrumPositionError_; };

  /**
   * @brief Checks if the fulcrum state is stored.
   *
   * Determines whether the fulcrum state has been stored in the system.
   *
   * @return True if the fulcrum state is stored, otherwise false.
   */
  [[nodiscard]] bool getFulcrumIsStored() const { return fulcrumIsStored_; };

  /**
   * @brief Sets the joint positions for the manipulator.
   *
   * Updates the internal joint positions to the specified values, reflecting
   * the desired state of the manipulator's joints.
   *
   * @param jointPositions The new joint positions provided as a mcx::control3::JointPositions object.
   */
  void setJointPositions(const mcx::control3::JointPositions& jointPositions) { jointPositions_ = jointPositions; };

  /**
   * @brief Sets the joint position limits for the system.
   *
   * Configures the upper and lower bounds for the joint positions,
   * ensuring that the joints operate within the specified range during motion.
   *
   * @param upperLimits The upper bounds for the joint positions.
   * @param lowerLimits The lower bounds for the joint positions.
   */
  void setJointPositionsLimit(const mcx::control3::JointPositions& upperLimits,
                              const mcx::control3::JointPositions& lowerLimits) {
    jointPositionsUpperLimit_ = upperLimits;
    jointPositionsLowerLimit_ = lowerLimits;
  };

  /**
   * @brief Sets the pose of the manipulator's tool.
   *
   * Updates the manipulator's tool pose with the specified pose, modifying
   * the internal state to reflect the new position and orientation.
   *
   * @param manipulatorToolPose The new pose for the manipulator's tool, expressed as a 6D Cartesian pose.
   */
  void setManipulatorToolPose(const mcx::math::CartPose6& manipulatorToolPose) {
    manipulatorToolPose_ = manipulatorToolPose;
  }

  /**
   * @brief Determines whether any cartesian pose constraint is currently being violated.
   *
   * This function iterates through all registered cartesian pose constraints associated
   * with the instrument and checks their individual violation state. If at least one
   * constraint is found to be violated, the function returns true; otherwise, it
   * returns false.
   *
   * @return True if any cartesian pose constraint is violated, false otherwise.
   */
  [[nodiscard]] bool isConstraintViolated() const;

  /**
   * @brief Determines whether the instrument is inserting.
   *
   * This function evaluates whether the instrument insertion depth is increasing.
   *
   * @return True if the insertion depth is increasing, false otherwise.
   */
  [[nodiscard]] bool isInstrumentInserting() const { return isInstrumentInserting_; };

  /**
   * @brief Determines whether the instrument is retracting.
   *
   * This function evaluates whether the instrument insertion depth is decreasing.
   *
   * @return True if the insertion depth is decreasing, false otherwise.
   */
  [[nodiscard]] bool isInstrumentRetracting() const { return isInstrumentRetracting_; };

  /**
   * @brief Checks if any cartesian pose constraint is currently limiting.
   *
   * This function evaluates each cartesian pose constraint associated with the instrument
   * to determine if it is actively limiting the system's behavior. If at least one constraint
   * is in a limiting state, the function returns true; otherwise, it returns false.
   *
   * @return True if any cartesian pose constraint is limiting, false otherwise.
   */
  [[nodiscard]] bool isConstraintLimiting() const;

  /**
   * @brief Checks if the tip of the instrument is located on the pitch joint.
   *
   * This function evaluates whether the instrument tip is positioned directly
   * on the pitch joint based on the current kinematic decoupling status.
   *
   * @return True if the tip is on the pitch joint, false otherwise.
   */
  [[nodiscard]] bool decoupledKinematicsEnabled() const { return decoupleInstrumentKinematics_; };

  /**
   * @brief Checks if the minimum insertion depth limit has been reached.
   *
   * This function indicates whether the current state has crossed the predefined
   * minimum insertion depth threshold.
   *
   * @return True if the minimum insertion depth limit has been reached, false otherwise.
   */
  [[nodiscard]] bool minInsertionDepthLimitReached() const { return minInsertionDepthLimitReached_; };

  /**
   * @brief Checks if the maximum insertion depth limit has been reached.
   *
   * This function determines whether the insertion depth has exceeded the pre-defined maximum limit.
   * It evaluates the current state and returns true if the limit has been reached; otherwise, it returns false.
   *
   * @return True if the maximum insertion depth limit is reached, false otherwise.
   */
  [[nodiscard]] bool maxInsertionDepthLimitReached() const { return maxInsertionDepthLimitReached_; };

  /**
   * @brief Checks if the insertion depth limit has been reached.
   *
   * This function returns the current state indicating whether the predefined
   * insertion depth limit has been reached or exceeded.
   *
   * @return True if the insertion depth limit has been reached, false otherwise.
   */
  [[nodiscard]] bool insertionDepthLimitReached() const { return insertionDepthLimitReached_; };

  /**
   * @brief Checks if the fulcrum port boundary has been exceeded.
   *
   * This method determines whether the current state indicates being outside
   * of the predefined fulcrum port. The result is based on the internal state
   * maintained by the object.
   *
   * @return True if outside the fulcrum port, false otherwise.
   */
  [[nodiscard]] bool isOutsideFulcrumPort() const { return outsideFulcrumPort_; };

  /**
   * @brief Checks if the maximum insertion depth limit is enabled.
   *
   * This method returns whether maximum insertion depth constraint is enabled
   *
   * @return True if outside the maximum insertion depth limit is enabled, false otherwise.
   */
  [[nodiscard]] bool isMaximumInsertionDepthEnabled() const {
    return cartPoseConstraints_.find("maxInsertionDepth")->second->isEnabled();
  };

  /**
   * @brief Retrieves the maximum allowable insertion depth for the cartesian pose.
   *
   * This function identifies and accesses the "maxInsertionDepth" cartesian pose constraint,
   * returning the boundary value defined for this constraint.
   *
   * @return The maximum insertion depth specified by the constraint.
   */
  [[nodiscard]] double getMaximumInsertionDepth() const {
    return cartPoseConstraints_.find("maxInsertionDepth")->second->getConstraintBoundary();
  }

  /**
   * @brief Retrieves the minimum insertion depth constraint boundary.
   *
   * This function accesses the cartesian pose constraints identified by the
   * key "minInsertionDepth" and returns the defined boundary for this constraint.
   *
   * @return The minimum insertion depth constraint boundary as a double.
   */
  [[nodiscard]] double getMinimumInsertionDepth() const {
    return cartPoseConstraints_.find("minInsertionDepth")->second->getConstraintBoundary();
  }

  /**
   * @brief Retrieves the current insertion depth of the instrument.
   *
   * This function returns the value representing the depth to which the instrument
   * is inserted, measured in the appropriate units. The insertion depth is maintained
   * internally and reflects the current position.
   *
   * @return The current insertion depth of the instrument.
   */
  [[nodiscard]] double getInsertionDepth() const { return instrumentInsertionDepth_; }

  /**
   * @brief Retrieves the length of the first link of the instrument.
   *
   * This method calculates and returns the magnitude of the local pose vector associated
   * with the first link of the instrument, providing its effective length.
   *
   * @return The length of the first link as a double.
   */
  [[nodiscard]] double getLink1Length() const;

  /**
   * @brief Retrieves the length of the second link in the instrument.
   *
   * This function calculates and returns the length of the second link
   * based on its local pose data. The value represents the norm of the
   * pose vector associated with the second link.
   *
   * @return The length of the second link as a double value.
   */
  [[nodiscard]] double getLink2Length() const;

  /**
   * @brief Retrieves the length of the third link in the instrument's mechanism.
   *
   * This function calculates and returns the magnitude of the vector defining
   * the local position of the third link's endpoint relative to its base.
   *
   * @return The length of the third link as a double.
   */
  [[nodiscard]] double getLink3Length() const;

  /**
   * Resets the fulcrum pose to a specified value or a default value.
   *
   * The method updates the stored fulcrumPose_ value to the given fulcrumPoseReset
   * and marks the fulcrum as not stored.
   *
   * @param fulcrumPoseReset The new pose to set as the fulcrum.
   *                         Defaults to a zero-initialized CartPose6.
   */
  void
  resetFulcrumPose(const mcx::math::CartPose6& fulcrumPoseReset = mcx::math::CartPose6({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

  /**
   * @brief Calculates the Jacobian matrix for a manipulator at a given pose and joint configuration.
   *
   * This function computes the 6x6 Jacobian matrix for a manipulator based on its
   * cartesian pose and joint positions. The Jacobian represents the relationship
   * between joint velocities and end-effector linear and angular velocities,
   * and is essential for robotic motion control and dynamics calculations.
   *
   * @param manipulatorPose The 6D cartesian pose of the manipulator in the inertial frame,
   *                        expressed as a vector of position and orientation components.
   * @param jointPositions  The current joint positions (angles) of the manipulator,
   *                        represented as a vector of joint values.
   * @return A 6x6 Jacobian matrix combining linear and angular velocity components.
   */
  [[nodiscard]] mcx::math::Matrix6x6 getJacobian(const mcx::math::CartPose6& manipulatorPose,
                                                 const mcx::control3::JointPositions& jointPositions) const;

  /**
   * @brief Retrieves the current cartesian constraint force vector.
   *
   * This function returns the force vector associated with the cartesian constraint
   * applied to the instrument. The vector represents the magnitude and direction
   * of the constraint force currently being enforced.
   *
   * @return The cartesian constraint force vector as a mcx::math::Vector3D.
   */
  [[nodiscard]] mcx::math::Vector3D getConstraintForce() const { return cartConstraintForce_; };

  /**
   * @brief Retrieves whether the point of interest is at the instrument tip.
   *
   * This function retrieves whether the point of interest for the coupled kinematics is
   * at the instrument tip.
   *
   * @return true if the POI is at the instrument tip, false if the POI is at the yaw joint
   */
  [[nodiscard]] bool pointOfInterestAtTip() const { return pointOfInterestAtTip_; };

private:
  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override;

  bool initPhase1_() override;

  bool initPhase2_() override;

  bool startOp_() override;

  bool stopOp_() override;

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override;

  /**
   * @brief Sets a new maximum insertion depth for the instrument.
   *
   * This function updates the maximum insertion depth constraint for the instrument.
   * If the provided maximum insertion depth is less than or equal to the already defined
   * minimum insertion depth, the maximum insertion depth will automatically be set to match
   * the minimum insertion depth to ensure valid constraint boundaries.
   *
   * @param maxInsertionLimit The desired maximum insertion depth to be set.
   */
  void teachMaxInsertionDepth(double maxInsertionLimit);

  /**
   * @brief Updates the cartesian pose constraints applied to the instrument target pose.
   *
   * This function iterates through all registered cartesian pose constraints and updates
   * the provided target pose accordingly to ensure compliance with the constraints.
   *
   * @param instrumentPoseTarget The initial target pose of the instrument, represented as a cartesian pose.
   * @return The modified target pose of the instrument after applying the updates from all constraints.
   */
  [[nodiscard]] mcx::math::CartPose6 updateInstrumentConstraints(mcx::math::CartPose6 instrumentPoseTarget) const;

  /**
   * @brief Computes the inverse kinematics for the instrument's motion.
   *
   * This function calculates the required instrument joint positions based on
   * the desired instrument tool pose and manipulator tool pose in the Cartesian space.
   *
   * @param instrumentToolPoseTarget The target pose of the instrument tool in Cartesian coordinates.
   * @param manipulatorToolPoseTarget The target pose of the manipulator tool in Cartesian coordinates.
   * @param instrumentJointPositionsTarget A reference to store the computed joint positions of the instrument.
   */
  void calculateInverseKin(mcx::math::CartPose6& instrumentToolPoseTarget,
                           mcx::math::CartPose6& manipulatorToolPoseTarget,
                           mcx::control3::JointPositions& instrumentJointPositionsTarget);

  // void checkStraightened();

  /**
   * @brief Calculates the total constraint force resulting from all active cartesian pose constraints.
   *
   * This function iterates through all registered cartesian pose constraints and determines
   * the combined constraint force to be applied. The force is computed based on the constraint's
   * stiffness coefficient, braking range, and whether the current position resides in the braking area.
   * The individual forces from all active constraints are accumulated and returned.
   *
   * @return A vector representing the calculated total constraint force based on the active cartesian pose constraints.
   */
  [[nodiscard]] mcx::math::Vector3D calculateConstraintForce() const;

  /**
   * @brief A boolean variable that controls whether the current fulcrum position
   * should be stored and certain positional constraints should be applied.
   *
   * When set to true, the fulcrum position is recorded, relevant constraints for
   * insertion depth are updated, and the fulcrum reference frame is stored. After
   * the operation is performed, the variable is reset to false to avoid repeated
   * storage in subsequent updates.
   *
   * Used primarily during the kinematic update process for instruments where a
   * fulcrum-centered constraint is required, such as in surgical robotics or
   * other precise control systems.
   */
  bool teachFulcrum_{false};
  /**
   * @brief Indicates whether the fulcrum pose has been stored.
   *
   * This boolean flag is used to determine if the fulcrum pose
   * has been updated and stored in the system. It is set to `true`
   * when the fulcrum pose is updated and `false` when the fulcrum
   * pose is reset or cleared.
   *
   * Usage in:
   * - `InstrumentControl::updateForwardKin`: Sets the flag to `true`
   *   when the fulcrum pose is updated.
   * - `InstrumentControl::resetFulcrumPose`: Resets the flag to `false`
   *   when the fulcrum pose is cleared.
   */
  bool fulcrumIsStored_{false};

  /**
   * @brief Indicates whether the roll-straightening feature of the instrument is enabled.
   *
   * This boolean variable represents the state of the roll-straightening functionality
   * in an instrument or system. If set to `true`, the feature is enabled, allowing the
   * system to automatically straighten the roll angle. If set to `false`, the feature is
   * disabled, and the roll angle will not be adjusted automatically.
   *
   * The default value is `false`.
   */
  bool instrumentRollStraightenEnable_{false};

  /**
   * Represents a Cartesian pose with six degrees of freedom (translation and rotation)
   * for the fulcrum, which is utilized in instrument kinematics and position control.
   * This member is updated within the forward kinematics implementation, specifically
   * in the context of handling fulcrum reference adjustments.
   */
  mcx::math::CartPose6 fulcrumPose_{};

  mcx::math::Position fulcrumPortPosition_{};
  mcx::math::Position fulcrumPortPositionOffset_{};
  /**
   * @brief Represents the fulcrum reference frame in the local coordinate system.
   *
   * This variable defines the local pose of the fulcrum reference point, relative to the robot or instrument base
   * frame. It is used to calculate the fulcrum's position and orientation in the inertial reference frame during
   * kinematic transformations.
   *
   * In applications involving robotic manipulation, such as laparoscopic surgery or remote-controlled instruments,
   * the fulcrum serves as a fixed pivot point about which the instrument operates. This local pose is utilized
   * in conjunction with other transformations to maintain accurate kinematic behavior and to compute the
   * fulcrum's contribution to the instrument's spatial orientation.
   *
   * Type: math::CartPose6
   */
  mcx::math::CartPose6 fulcrumReferenceLocal_{};
  /**
   * @brief Represents the position of the first link's tip in the local coordinate frame.
   *
   * This variable stores the local pose information of the tip of the first link
   * in a robotic system. It is used to compute the sequential forward kinematics
   * of the robot by transforming it into the inertial reference frame. The variable
   * plays a critical role in determining the overall pose of the tip in the global
   * coordinate system by chaining transformations with other components like roll,
   * wrist, tool, and other link poses. Used specifically in updating forward kinematics
   * calculations.
   */
  mcx::math::Position link1TipPoseLocal_{};
  /**
   * @brief Represents the relative pose of the second link to the tip in the local frame.
   *
   * This variable stores the position and orientation offset (pose) of the second link with
   * respect to the tip in the local coordinate system. It is used in kinematic calculations
   * for determining the forward kinematics of the instrument, particularly in context of
   * determining the tool and tip poses.
   *
   * The value is updated as part of the robot's kinematics when computing transformations
   * in the inertial reference frame.
   */
  mcx::math::Position link2TipPoseLocal_{};
  /**
   * @brief Represents the pose of the tip of link 3 in the local reference frame.
   *
   * This variable is used in robotic kinematics calculations to describe the
   * relative position and orientation of the tip of the third link of the
   * manipulator with respect to its local coordinate frame.
   *
   * link3TipPoseLocal_ is employed during forward kinematics computation, particularly
   * in the determination of the tool's pose with respect to the manipulator base or
   * other reference frames as part of the robotic transformation chain.
   */
  mcx::math::Position link3TipPoseLocal_{};

  /**
   * @brief A variable representing the external length of the fulcrum port.
   *
   * This variable holds a floating-point value that indicates the external length
   * of the fulcrum port component. It is initialized to zero by default.
   */
  double fulcrumPortExternalLength_{};
  /**
   * @brief Indicates whether the teaching operation should enforce an insertion depth limit.
   *
   * The variable determines if a constraint is applied during a teaching
   * operation to limit the depth of insertion. If set to `true`, the teaching
   * process will respect a predefined insertion depth constraint. If set to
   * `false`, no such constraint will be applied, allowing unrestricted insertion
   * depth during the operation.
   *
   * @details This flag may be used in scenarios where controlling or capping the
   * depth of structure or data insertion is required, enhancing performance or
   * ensuring operational accuracy.
   */
  bool teachInsertionDepthLimit_{false};
  /**
   * @brief Flag indicating whether the insertion depth limit should be reset.
   *
   * This variable is used to control whether the depth limit for an insertion operation
   * within a specific context needs to be reset or not. If set to `true`, the depth limit
   * is reset; if `false`, the depth limit remains unchanged.
   */
  bool resetInsertionDepthLimit_{false};
  /**
   * @brief Represents the minimum required depth for instrument insertion.
   *
   * This variable is used to define the minimum depth value that an
   * instrument must be inserted to ensure proper functionality or operation.
   * It is measured in appropriate depth units (e.g., millimeters).
   *
   * The value ensures safe and effective use of the instrument by setting
   * a lower bound for insertion depth, preventing issues such as improper
   * placement or insufficient engagement.
   */
  double minInstrumentInsertionDepth_{};
  /**
   * @brief Calibration offset for the instrument insertion depth.
   *
   * Represents the adjustment value applied to the instrument's
   * insertion depth during calibration to account for system-specific
   * or environmental variations. This value ensures accurate positioning
   * of the instrument relative to the target location.
   */
  double instrumentInsertionDepthCalibrationOffset_{};
  /**
   * @brief Represents the insertion depth of an instrument in a specific context.
   *
   * This variable is used to store the depth (likely in a specific unit of measurement,
   * such as millimeters or inches) to which an instrument is inserted. It is typically
   * associated with applications that involve medical instruments, probes, or other tools
   * requiring precise depth measurement.
   */
  double instrumentInsertionDepth_{};
  /**
   * @brief Represents the tolerance level for determining the validity of a fulcrum.
   *
   * This variable defines a numerical threshold value used to assess the
   * acceptability or validity of a fulcrum within a specific context.
   * The tolerance value helps in performing calculations where exact
   * precision is not practical, allowing for minor deviations to be considered
   * valid.
   *
   * @note The exact purpose and usage of this variable may depend on the
   * overarching implementation context and the associated algorithms.
   */
  double fulcrumValidityTolerance_{};
  /**
   * @brief Indicates the validity of the fulcrum position based on calculated errors and a defined tolerance.
   *
   * This variable is used to determine whether the current fulcrum position is within acceptable limits.
   * It is set to `true` if the calculated fulcrum position error is within a specific tolerance threshold,
   * and `false` otherwise.
   *
   * The validity is assessed during operations such as manipulator behavior control and instrument
   * insertion depth management. When false, it may impact additional constraints or conditions
   * related to fulcrum-centered operations.
   */
  bool fulcrumValid_{};
  /**
   * @brief Represents the magnitude of the positional error of the fulcrum in a system.
   *
   * The variable `fulcrumPositionError_` is used to store the calculated magnitude of
   * the fulcrum's positional error. This value is typically updated dynamically as part
   * of the system's output parameters and serves as a measure of how far the fulcrum
   * deviates from its desired or ideal position. It can be utilized for diagnostic
   * purposes or for driving corrective actions within the system.
   *
   * @note This variable is used as an output parameter in the system for reporting
   * fulcrum error magnitudes.
   */
  double fulcrumPositionError_{};
  /**
   * @brief Represents the error in the x-coordinate of the fulcrum position.
   *
   * This variable stores the deviation or error in the x-dimension of the
   * fulcrum position. It is primarily used in scenarios where precise
   * positioning of a fulcrum is critical, and the x-axis error needs to
   * be tracked or compensated for in calculations or adjustments.
   *
   * The value is represented as a double-precision floating-point number.
   */
  double fulcrumPositionErrorX_{};
  /**
   * @brief Represents the error or deviation in the Y-coordinate of a fulcrum's position.
   *
   * This variable stores the difference or discrepancy in the estimated or observed Y-coordinate
   * value of a fulcrum, typically used in the context of mechanics, robotics, or structural analysis.
   * The error value can be positive or negative based on the measurement deviation.
   */
  double fulcrumPositionErrorY_{};
  /**
   * @brief Represents the stiffness coefficient used in the calculation of constraint forces.
   *
   * This variable is utilized within the control logic to determine the scaling factor
   * for calculating constraint forces based on the delta position and braking range.
   *
   * The value of this variable affects the magnitude of the applied constraint force,
   * with higher values resulting in stronger forces being applied to correct deviations.
   */
  double constraintForceStiffness_{};
  /**
   * @brief Controls whether the instrument kinematics should be decoupled from the manipulator kinematics.
   *
   * When set to true, the forward and inverse kinematic calculations for the instrument
   * are performed independently from the manipulator kinematics. This can be useful in scenarios
   * where the instrument's motion needs to be treated separately from the manipulator,
   * such as when implementing specialized control strategies or simulating kinematic behavior without direct coupling.
   *
   * If set to false, the instrument kinematics remain coupled with the manipulator kinematics,
   * allowing standard motion and pose updates that rely on their interdependence.
   *
   * Default value is false.
   */
  bool decoupleInstrumentKinematics_{false};

  /**
   * @brief Represents the current actual position and orientation of the fulcrum reference in a 6D Cartesian space.
   *
   * This variable stores the fulcrum's pose in the inertial reference frame. It is updated within kinematic
   * computations and represents the real-time state of the fulcrum reference in terms of its position and orientation.
   *
   * The value is derived by transforming the local fulcrum reference frame, `fulcrumReferenceLocal_`, using the
   * manipulator's global pose. It is expressed as a 6D vector (position in 3D space and orientation in roll, pitch,
   * yaw).
   *
   * This variable is essential to defining constraints and relationships for instrument control during
   * kinematics calculations, such as setting insertion depth boundaries or teaching the fulcrum position.
   */
  mcx::math::CartPose6 fulcrumReferenceActual_{};
  /**
   * @brief Represents the pose of a manipulator's tool in Cartesian coordinates.
   *
   * This variable holds a Cartesian pose with six degrees of freedom
   * (position and orientation) that defines the location and orientation
   * of the manipulator's end-effector/tool.
   *
   * @details The data type `mcx::math::CartPose6` typically encapsulates
   * information regarding the 3D position (x, y, z) and orientation
   * (roll, pitch, yaw or equivalent representation) of the manipulator tool.
   * It is a crucial property for robotic systems requiring precise
   * manipulation tasks.
   */
  mcx::math::CartPose6 manipulatorToolPose_{};
  /**
   * @brief Represents the previous pose of a manipulator tool.
   *
   * This variable stores the Cartesian pose of the manipulator tool
   * at a prior state or timestep. It is used to track the tool's
   * position and orientation over time for tasks requiring historical
   * pose data.
   *
   * The data is represented using the `mcx::math::CartPose6` type,
   * which encodes the 6 degrees of freedom (3 for position and 3 for
   * orientation) of the manipulator tool in Cartesian space.
   */
  mcx::math::CartPose6 manipulatorToolPosePrevious_{};
  /**
   * @brief Represents the current joint positions of a robotic instrument.
   *
   * This variable holds the positions of each joint in the robotic instrument. It is
   * initialized with a pre-determined number of joints and is used in various calculations,
   * including inverse kinematics, to determine or manipulate the state of the instrument.
   * The positions are updated or retrieved as part of control logic. Appropriate constraints
   * or limits (upper and lower) are applied to validate the joint angles during operations.
   */
  mcx::control3::JointPositions jointPositions_{};
  /**
   * @brief Represents the upper limit for joint positions of an instrument.
   *
   * This variable holds the maximum allowed joint positions for the instrument's joints.
   * It is used to ensure the joint positions remain within valid operational limits.
   * The size of this data structure is dynamically set to match the number of instrument joints.
   */
  mcx::control3::JointPositions jointPositionsUpperLimit_{};
  /**
   * @brief Represents the lower limits for joint positions in the control system.
   *
   * This variable stores the minimum permissible joint positions for all instrument joints.
   * It is resized dynamically during the initialization of the control system to fit the
   * total number of joints as defined by NR_OF_INSTRUMENT_JOINTS. These limits are used
   * to enforce constraints and ensure the joints operate within safe bounds.
   *
   * @note This variable is utilized in conjunction with jointPositions_ and
   *       jointPositionsUpperLimit_ to define the operational range of the joints.
   */
  mcx::control3::JointPositions jointPositionsLowerLimit_{};

  /**
   * @brief Indicates whether the minimum insertion depth limit has been reached.
   *
   * The variable represents a flag that is set to `true` when the insertion
   * process has reached the predefined minimum depth threshold. It is
   * initialized as `false` and can be updated dynamically during the
   * operation where the depth limit is a concern.
   *
   * This flag is useful in scenarios where controlling or limiting
   * the depth of data insertion or processing is required to ensure
   * optimal performance or to prevent exceeding resource constraints.
   */
  bool minInsertionDepthLimitReached_{false};
  /**
   * @brief Indicates whether the maximum insertion depth limit has been reached.
   *
   * This flag is set to true if the manipulator exceeds the maximum allowable insertion
   * depth as defined by the constraint "maxInsertionDepth". It is evaluated during
   * the operation of the system, based on the current insertion depth and the
   * constraints applied. If the insertion depth exceeds the defined limit and the
   * manipulator tip is not outside the fulcrum port, this flag is set to true.
   * Otherwise, it remains false. This variable is also used in conjunction with
   * other depth limit flags to determine overall insertion depth violations.
   */
  bool maxInsertionDepthLimitReached_{false};
  /**
   * @brief Indicates whether the insertion depth limit has been reached.
   *
   * This flag is set to `true` if either the minimum insertion depth or the maximum insertion depth limit
   * has been reached or violated during the operation of an instrument. It consolidates checks for both
   * depth constraints to determine if any boundary condition related to insertion depth has been met.
   *
   * @details
   * - The flag is updated during execution based on the state of minimum and maximum insertion depth checks.
   * - `true` signifies that the insertion depth restrictions have been triggered (either minimum or maximum).
   * - `false` indicates normal operation within permissible insertion depth limits.
   *
   * @note
   * This variable is critical for validating safe operating boundaries of the instrument and ensuring compliance
   * with constraints defined for its movement relative to the fulcrum or system configuration.
   */
  bool insertionDepthLimitReached_{false};
  /**
   * @brief Indicates whether the context or entity is outside the fulcrum port boundary.
   *
   * This boolean variable is used to determine the location status in relation to the fulcrum port.
   * A value of `true` signifies that the entity is outside the fulcrum port, while `false` indicates
   * that it is inside or within the fulcrum port.
   */
  bool outsideFulcrumPort_{false};
  /**
   * @brief Represents the Cartesian constraint force in 3D space.
   *
   * This variable is used to store the force applied as a constraint
   * in Cartesian coordinates. It is represented as a 3D vector and is
   * initialized to default values.
   */
  mcx::math::Vector3D cartConstraintForce_{};

  /**
   * @brief A container that holds Cartesian pose constraints for the instrument control system.
   *
   * This variable is a map where the keys are strings representing the names of the constraints
   * and the values are unique pointers to instances of `CartesianPoseConstraint`, which define
   * the specific constraints to be applied in Cartesian space.
   *
   * The constraints are utilized to manage and enforce spatial limitations or allowable regions
   * for the instrument's positioning. Examples of constraints include cylindrical constraints,
   * spherical constraints, and planar constraints.
   *
   * Key-value pairs in this map correspond to different types of constraints, and their objects
   * are dynamically allocated during the initialization of the control system.
   */
  std::map<std::string, std::unique_ptr<CartesianPoseConstraint>> cartPoseConstraints_;

  /**
   * @brief Stores the target pose of the instrument tool in Cartesian coordinates.
   *
   * This variable represents the target pose of the instrument tool, including position
   * (x, y, z) and orientation (roll, pitch, yaw) in a six-dimensional Cartesian space.
   * It is updated during the kinematic calculations to align the instrument tool to
   * the desired position and orientation relative to its control requirements.
   *
   * The pose values include:
   * - Position: x, y, z coordinates.
   * - Orientation: roll, pitch, yaw angles.
   *
   * @note This variable plays a key role in the inverse kinematics calculation, where
   *       it is used to compute joint positions and motion dynamics based on desired
   *       instrument movements.
   *
   * @see InstrumentControl::calculateInverseKin
   */
  mcx::math::CartPose6 instrumentToolPoseTarget_{};

  /**
   * @brief A vector representing the configuration for reversing body rotation directions.
   *
   * This variable is used to determine if the rotational directions of specific body axes
   * should be inverted during calculations. It is particularly applied when computing
   * the joint angles or rates for a manipulator or instrument, considering rotations in roll,
   * pitch, and yaw.
   *
   * Each boolean value in the vector corresponds to an axis of rotation:
   * - true indicates the rotation direction for that axis should be reversed.
   * - false indicates the default rotation direction is maintained.
   *
   * The size of the vector aligns with the three-dimensional space (roll, pitch, yaw).
   * It is typically referenced when finalizing transformations or adjustments
   * in kinematic calculations.
   */
  mcx::math::Vector<3, bool> reverseBodyRotation_{};

  /**
   * @brief Indicates whether the roll motion for the instrument is enabled.
   *
   * When set to true, roll motion calculations for the instrument are included
   * in the inverse kinematics computations, affecting the corresponding joint
   * angles. If false, roll motion is excluded from these calculations.
   *
   * This variable is used in contexts like calculating the orientation and
   * joint position of instruments, which are influenced by the roll component
   * based on this flag's value.
   *
   * Default value: false.
   */
  bool enableInstrumentRoll_{false};

  /**
   * @brief Indicates whether the instrument is inserting.
   *
   * The variable represents a flag that is set to `true` when the instrument
   * insertion depth is increasing. It is initialized as `false` and is updated
   * dynamically during the operation.
   *
   * This flag is useful in scenarios where actions are based on the
   * insertion direction of the instrument.
   *
   * Default value: false.
   */
  bool isInstrumentInserting_{false};

  /**
   * @brief Indicates whether the instrument is retracting.
   *
   * The variable represents a flag that is set to `true` when the instrument
   * insertion depth is decreasing. It is initialized as `false` and is updated
   * dynamically during the operation.
   *
   * This flag is useful in scenarios where actions are based on the
   * insertion direction of the instrument.
   *
   * Default value: false.
   */
  bool isInstrumentRetracting_{false};

  /**
   * @brief Specifies whether the point of interest is at the instrument tip.
   *
   * The variable specifies the point of interest for the coupled kinematics, if
   * set to true, the instrument tip is used as the point of interest, else the
   * POI is at J3 (yaw joint). It is initialized as `false` and is updated
   * dynamically during the operation.
   *
   * This flag is useful in scenarios where the point of interest needs to be changed.
   *
   * Default value: false.
   */
  bool pointOfInterestAtTip_{false};

  unsigned int fulcrumTeachSetting_{fulcrumTeachSetting::MANUAL_INPUT};
};

} // end namespace control

#endif /* CTRL_INSTRUMENTCONTROL_H */
