/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef CTRL_JOINTPOSITIONMODULE_H
#define CTRL_JOINTPOSITIONMODULE_H

#include "mcx/control3/ctrl_joints.h"
#include "mcx/control3/ctrl_setpointgenerator.h"
#include <mcx/core.h>

/**
 * @class SymbolicPositionModule
 * @brief Implements functionality for managing and controlling symbolic positions of joints.
 *
 * The SymbolicPositionModule is used to define, manipulate, and control symbolic joint positions
 * across multiple channels. It provides methods for teaching symbolic positions, checking if
 * joints are at a specified symbolic position, synchronizing joint positions, and moving joints
 * to their target symbolic positions.
 *
 * @param numberOfActuators Number of actuators (or joints) controlled by this module. Default is 1.
 * @param symbolicNames Vector containing the names for symbolic positions, allowing for custom labels.
 *
 * @note This class is designed for robotic control systems where symbolic positions are used to
 *       map abstract or predefined positions to real joint configurations.
 *
 * Members:
 * - **SYMBOLIC_POSITION_TOLERANCE**: The threshold used for determining whether a joint has reached
 *   a symbolic position. Default is 1e-2.
 * - **jointSymbolicPositions_**: A vector storing symbolic positions for each joint channel.
 * - **syncJointPositions_**: Stores synchronized positions of all joints to ensure consistency across operations.
 * - **numberOfJoints_**: The total number of joints managed by the module.
 * - **timeScaleFactor_**: A multiplier used to scale the time for setpoint generation, helping with press-and-move
 * operations.
 * - **startMove_**: A flag indicating whether movement to a symbolic position has been started.
 * - **selectMove_**: The currently selected symbolic move index.
 */
class SymbolicPositionModule final : public mcx::container::Module {
  //  using SetpointGenerators = std::vector<mcx::control3::SetpointGenerator()>;

  struct SymbolicPosition {
    unsigned int number{};
    std::string name;
    double tolerance{};
    bool teach{};
    bool done{};
    std::valarray<uint8_t> isBusy;
    std::valarray<uint8_t> isAtPosition;
    std::valarray<uint8_t> moveToPosition;
    std::valarray<uint8_t> moveOrder;
    mcx::control3::JointPositions position;
  };

public:
  /// Straighten threshold
  static constexpr double SYMBOLIC_POSITION_TOLERANCE = 1e-2;

  explicit SymbolicPositionModule(unsigned int numberOfActuators = 1,
                                  std::vector<std::string> symbolicNames = {"sympos01"});

  ~SymbolicPositionModule() override = default;

  /**
   * @brief Sets the state to initiate or stop the movement towards a symbolic position.
   *
   * This method updates the flag that controls whether the system should begin or halt
   * the movement to a predefined symbolic position.
   *
   * @param newValue The new state for the movement flag. Setting it to true initiates the movement,
   *                 while setting it to false stops it.
   */
  void setMoveToSymbolicPosition(bool newValue) { startMove_ = newValue; }
  /**
   * @brief Sets the move-to-symbolic-position status for a specific joint channel and index.
   *
   * This method updates the status that determines whether a specific joint channel should move to
   * a target symbolic position defined by the given index. It modifies the internal control state
   * for the specified channel and symbolic position.
   *
   * @param newValue The new status indicating whether to move to the symbolic position (true to enable, false to
   * disable).
   * @param channel The joint channel index to be updated.
   * @param index The symbolic position index within the specified channel to update the status for.
   */
  void setMoveToSymbolicPositionChannel(bool newValue, unsigned int channel, unsigned int index) {
    jointSymbolicPositions_[channel].moveToPosition[index] = newValue;
  }

  /**
   * @brief Sets a new time scale factor for scaling the time used in setpoint generation.
   *
   * The time scale factor is used to control the timing behavior of operations, such as
   * press-and-move, by applying a multiplier to time-based calculations.
   *
   * @param newValue The new value to set for the time scale factor. Must be a non-negative double.
   */
  void setTimeScaleFactor(double newValue) { timeScaleFactor_ = newValue; }

  /**
   * @brief Teaches a new symbolic position for a specified joint channel.
   *
   * The method assigns a new symbolic position to the given channel by updating
   * the joint's symbolic position with the provided value.
   *
   * @param newValue JointPositions object representing the desired symbolic position to be taught.
   * @param channel The index of the joint channel for which the symbolic position is being updated.
   *
   * @note Ensure the specified channel exists within the bounds of the system's configuration.
   */
  void teachSymbolicPosition(mcx::control3::JointPositions& newValue, unsigned int channel) {
    jointSymbolicPositions_[channel].position = newValue;
  }

  /**
   * @brief Sets a symbolic position value for a specific joint channel and index.
   *
   * This method assigns a new symbolic position value to a given index within a specific channel.
   * It is commonly used to update or define symbolic positions for controlling robotic joint
   * configurations.
   *
   * @param newValue The new symbolic position value to be assigned.
   * @param channel The index of the joint channel where the symbolic position is being updated.
   * @param index The specific index within the channel for the symbolic position to be updated.
   *
   * @note It is assumed that the corresponding channel and index are valid and have been properly
   * initialized before calling this function.
   */
  void teachSymbolicPosition(double newValue, unsigned int channel, unsigned int index) {
    jointSymbolicPositions_[channel].position[index] = newValue;
  }

  /**
   * @brief Checks if the current joint positions match the symbolic position of the selected move.
   *
   * This function compares the actual joint positions with the target symbolic position for the
   * currently selected move index. It determines whether the joints have reached their target
   * symbolic position based on predefined criteria.
   *
   * @param jointPositionsActual The actual joint positions to be checked against the target symbolic position.
   * @return True if the actual joint positions match the target symbolic position for the selected move, false
   * otherwise.
   */
  bool checkAtSymbolicPosition(const mcx::control3::JointPositions& jointPositionsActual) {
    return checkAtSymbolicPosition(jointSymbolicPositions_[selectMove_].position, jointPositionsActual, selectMove_);
  }

  /**
   * @brief Checks if the specified channel is at its designated symbolic position.
   *
   * This method verifies whether the actual joint positions for a given channel
   * match the predefined symbolic position for that channel. It is used to ensure
   * that the physical joint positions align with the expected symbolic target.
   *
   * @param jointPositionsActual The current actual joint positions of the system.
   * @param channel The channel representing the specific symbolic position to check.
   * @return True if the actual joint positions for the specified channel match the symbolic position, false otherwise.
   */
  bool checkAtSymbolicPosition(const mcx::control3::JointPositions& jointPositionsActual, unsigned int channel) {
    return checkAtSymbolicPosition(jointSymbolicPositions_[channel].position, jointPositionsActual, channel);
  }

  /**
   * @brief Checks whether the joints of a specified channel are at their target symbolic positions.
   *
   * This method verifies if the actual joint positions match the target symbolic positions within
   * a defined tolerance for a given channel. It evaluates the status of each joint and determines
   * if all joints in the specified channel have reached their symbolic positions. It updates the
   * internal state of whether the joints are at positions for the given channel.
   *
   * @param jointPositionsTarget The target positions of the joints for the specified channel.
   * @param jointPositionsActual The current actual positions of the joints for the specified channel.
   * @param channel The specific channel index for which the check is performed.
   * @return A boolean value indicating if all joints in the specified channel have reached their
   *         target symbolic positions. Returns true if all joints are at their target positions;
   *         otherwise, false.
   */
  bool checkAtSymbolicPosition(const mcx::control3::JointPositions& jointPositionsTarget,
                               const mcx::control3::JointPositions& jointPositionsActual, unsigned int channel);

  /**
   * @brief Checks if any symbolic positions for a given joint channel are still busy.
   *
   * This method determines whether any symbolic position within the specified joint channel
   * is currently marked as "busy," indicating ongoing motion or processing.
   *
   * @param channel The index of the joint channel to check.
   * @return True if any symbolic position in the given joint channel is currently "busy"; false otherwise.
   */
  [[nodiscard]] bool getIsBusy(const unsigned int channel) const {
    return std::ranges::any_of(jointSymbolicPositions_[channel].isBusy, [](const auto& el) { return el; });
  }

  /**
   * @brief Checks whether a symbolic position movement has started.
   *
   * This method determines if the movement to a symbolic position is ongoing by
   * evaluating the current state of the time scale factor.
   *
   * @return True if a symbolic position movement has started; otherwise, false.
   */
  [[nodiscard]] bool getIsStarted() const { return static_cast<bool>(timeScaleFactor_); }

  /**
   * @brief Retrieves the name of the currently selected symbolic move.
   *
   * This method returns the symbolic name associated with the currently selected move index
   * in the `jointSymbolicPositions_` vector. The name corresponds to the symbolic move being
   * executed or selected.
   *
   * @return A reference to the string representing the symbolic name of the selected move.
   */
  [[nodiscard]] const std::string& getSelectedMoveName() const { return jointSymbolicPositions_[selectMove_].name; }

  /**
   * @brief Teaches or updates symbolic joint positions based on the current actual joint positions.
   *
   * This method is used to either teach new symbolic positions or update existing ones by using
   * the provided actual joint positions as reference. The teach parameter determines whether
   * the teaching process is active.
   *
   * @param jointPositionsActual The actual joint positions provided as a reference for teaching symbolic positions.
   * @param teach A boolean flag indicating whether teaching mode is active. If true, the method will teach or update
   * symbolic positions.
   */
  void teachSymbolicPosition(const mcx::control3::JointPositions& jointPositionsActual, bool teach) {}

  /**
   * @brief Sets the selected move index for symbolic position control.
   *
   * This method updates the index of the currently selected symbolic move, which determines
   * the specific symbolic position targeted by the motion control system.
   *
   * @param newValue The new symbolic move index to set. Represents the selected move for
   *                 controlling joint symbolic positions.
   */
  void setSelectMove(unsigned int newValue) { selectMove_ = newValue; };

  /**
   * @brief Sets the synchronized joint positions to a new value.
   *
   * This method updates the internally stored synchronized joint positions with the provided value.
   * It ensures that all joint positions are consistent during operations.
   *
   * @param newValue The new joint positions to be set, represented as a JointPositions object.
   *                 This should contain the updated positions for all joints.
   */
  // void setSyncJointPositions(mcx::control3::JointPositions newValue) { syncJointPositions_ = newValue; };

  /**
   * @brief Synchronizes joint positions with a new set of values and resets joint setpoint generators if necessary.
   *
   * This method updates the synchronized joint positions with the provided joint position values.
   * It also iterates through the joint setpoint generators and resets them to the updated positions
   * if no symbolic movement is currently in progress (indicated by the `startMove_` flag).
   *
   * @param newValue The new joint position values of type `mcx::control3::JointPositions` to synchronize with.
   *
   * @note This function ensures joint positions remain consistent and resets setpoint generators
   *       to reflect the updated state, unless a movement operation has already started.
   */
  void syncJointPositions(const mcx::control3::JointPositions& newValue) {
    syncJointPositions_ = newValue;
    unsigned int cnt = 0;
    for (const auto& generator : jointSetpointGenerators_) {
      if (!startMove_) {
        generator->resetTo(syncJointPositions_[cnt]);
      }
      cnt++;
    }
  }

  /**
   * @brief Moves the specified joint positions to their corresponding symbolic positions.
   *
   * This method adjusts the target joint positions to align with the symbolic positions defined
   * for a specific channel, based on the actual joint positions provided.
   *
   * @param jointPositionsTarget Reference to the target joint positions that will be adjusted to match
   *                             the desired symbolic positions.
   * @param jointPositionsActual Reference to the actual joint positions of the system, used to determine
   *                             the current state of the joints for the symbolic move calculation.
   * @param channel Specifies the channel for which the symbolic positions are to be applied.
   *
   * @return A reference to the updated target joint positions after applying the symbolic adjustments.
   */
  mcx::control3::JointPositions& moveToSymbolicPosition(mcx::control3::JointPositions& jointPositionsTarget,
                                                        const mcx::control3::JointPositions& jointPositionsActual,
                                                        unsigned int channel) {
    return moveToSymbolicPositions(jointPositionsTarget, jointPositionsActual, channel, startMove_);
  }

  /**
   * @brief Moves joints to their specified symbolic positions.
   *
   * This method initiates the movement of joints to their target symbolic positions by utilizing
   * the provided target and actual joint positions. The movement is governed by pre-defined
   * symbolic move parameters and state flags.
   *
   * @param jointPositionsTarget Reference to the target positions of joints, which are symbolic positions
   *                              to be achieved by the control system.
   * @param jointPositionsActual Constant reference to the actual current positions of the joints.
   *                              Used for comparison and to determine adjustments required to reach target positions.
   * @return A reference to the modified joint positions target after processing the symbolic move.
   *
   * @note This function requires symbolic move and start move parameters to be set correctly to
   *       execute the movement operation.
   */
  mcx::control3::JointPositions& moveToSymbolicPositions(mcx::control3::JointPositions& jointPositionsTarget,
                                                         const mcx::control3::JointPositions& jointPositionsActual) {
    return moveToSymbolicPositions(jointPositionsTarget, jointPositionsActual, selectMove_, startMove_);
  }

  /**
   * @brief Moves joints to specified symbolic positions.
   *
   * This method updates the target joint positions to move towards symbolic positions
   * while considering the actual joint positions and movement initiation flags.
   *
   * @param jointPositionsTarget Reference to the target joint positions that need to be updated.
   * @param jointPositionsActual Constant reference to the actual joint positions for comparison and adjustment.
   * @param startMove Flag indicating whether the symbolic movement process should be initiated.
   * @return Reference to the updated target joint positions after the move towards the symbolic positions is processed.
   */
  mcx::control3::JointPositions& moveToSymbolicPositions(mcx::control3::JointPositions& jointPositionsTarget,
                                                         const mcx::control3::JointPositions& jointPositionsActual,
                                                         bool startMove) {
    return moveToSymbolicPositions(jointPositionsTarget, jointPositionsActual, selectMove_, startMove);
  }

  /**
   * @brief Moves joints to their target symbolic positions based on a specified symbolic channel.
   *
   * This method calculates the target positions for all joints by following the predefined
   * symbolic position sequences for a specified channel. Movement progresses in order, defined
   * by the move orders assigned to joints, ensuring synchronized and stepwise transitions.
   *
   * If the startMove flag is set to true, the movement process starts, initiating the transition
   * from the current actual positions to the configured symbolic positions.
   *
   * @param jointPositionsTarget Reference to the target joint positions, updated with setpoint values.
   * @param jointPositionsActual Current actual positions of the joints.
   * @param channel The symbolic position channel used to determine the target configuration.
   * @param startMove Flag indicating whether movement to the symbolic positions should start.
   *
   * @return Reference to the target joint positions after modification.
   */
  mcx::control3::JointPositions& moveToSymbolicPositions(mcx::control3::JointPositions& jointPositionsTarget,
                                                         const mcx::control3::JointPositions& jointPositionsActual,
                                                         unsigned int channel, bool startMove);

  /**
   * @brief Checks if a joint on a specific channel is at a specified symbolic position.
   *
   * This method determines whether the joint on the provided channel is currently at
   * the symbolic position specified by the given index. The symbolic position is
   * based on the predefined set of positions for that joint channel.
   *
   * @param channel The channel index representing the specific joint within the system.
   * @param index The symbolic position index to check for the joint on the given channel.
   * @return Returns true if the joint on the specified channel is at the symbolic position index, false otherwise.
   */
  [[nodiscard]] bool getJointAtSymbolicPosition(unsigned int channel, unsigned int index) const {
    return jointSymbolicPositions_[channel].isAtPosition[index];
  };

  /**
   * @brief Checks if a joint at the specified channel has reached a symbolic position.
   *
   * This method returns whether the joint associated with the given channel
   * has completed movement to its target symbolic position. The symbolic position
   * is considered "done" when the joint has arrived at the desired configuration
   * within the defined tolerance.
   *
   * @param channel The index of the joint channel to check for symbolic position completion.
   * @return True if the specified joint channel has reached its symbolic position; otherwise, false.
   */
  [[nodiscard]] bool getAtSymbolicPosition(unsigned int channel) const {
    return jointSymbolicPositions_[channel].done;
  };

  [[nodiscard]] double getOutputPosition(const unsigned int channel) const {
    return outputPositions_[channel];
  };

private:
  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override;

  bool initPhase1_() override;

  bool initPhase2_() override;

  bool startOp_() override;

  bool stopOp_() override;

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override;

  /**
   * @var numberOfJoints_
   * @brief Represents the total number of joints managed or controlled.
   *
   * This variable holds the number of joints or actuators associated with a specific module
   * or system. It is used to define and manage the length of arrays or vectors for joint-specific
   * configurations and operations.
   *
   * @note The value of this variable is expected to be set during initialization to ensure
   *       proper handling of joint operations.
   */
  unsigned int numberOfJoints_{};
  /**
   * @brief Represents the total number of communication or data channels.
   *
   * The `numberOfChannels_` variable holds the count of channels available or in use for
   * communication, data processing, or other related functionalities. It is used to manage
   * and configure channel-specific operations such as data transfer, signal processing, or
   * resource allocation.
   *
   * Typical use cases include:
   * - Managing the number of input/output channels in a system.
   * - Defining channel-specific configurations or behaviors.
   * - Monitoring active or allocated channels in a system.
   *
   * @note The value is set to an unsigned integer and should be initialized appropriately to
   *       match the intended application.
   */
  unsigned int numberOfChannels_{};

  /**
   * @var currentMoveOrder_
   * @brief Represents the current order or sequence of a move in a series of operations.
   *
   * This variable is used to track the order in which moves are performed, enabling
   * sequential execution or identification of specific steps in a move sequence.
   * It is initialized with a default value of 1, indicating the starting point of
   * the move order.
   *
   * @note This variable is critical in systems where move operations are performed
   *       in a specific sequence or need to be reliably identified by their order.
   */
  unsigned int currentMoveOrder_ = 1;

  /**
   * @brief A container for managing setpoint generators for joint control.
   *
   * The `jointSetpointGenerators_` variable holds a collection of unique pointers to
   * `SetpointGenerator` objects, which are responsible for generating setpoints for
   * individual joint movements. Each generator encapsulates the logic to compute
   * desired joint states, such as positions, velocities, or accelerations, over time.
   *
   * Members of this container correspond to individual joints or actuators in a robotic system.
   * The size of the vector typically matches the number of joints being controlled,
   * ensuring that each joint can have an independently defined setpoint generator.
   *
   * @note The use of `std::unique_ptr` ensures exclusive ownership and prevents duplication
   *       of the setpoint generators, aligning with the need for efficient and safe memory management.
   */
  std::vector<std::unique_ptr<mcx::control3::SetpointGenerator>> jointSetpointGenerators_;

  /**
   * @variable gotoJointMoveToSymbolicPositionPrevious_
   * @brief Stores previous states of joint movements towards symbolic positions.
   *
   * This variable is used to track the prior movement states for each joint to symbolic positions,
   * represented as a valarray of uint8_t values. It can be used to compare with current states or
   * determine changes in motion or target positions.
   *
   * @note Each element in the valarray corresponds to a specific joint, with the value encoding
   *       whether the movement is complete, active, or in a previous state.
   */
  std::valarray<uint8_t> gotoJointMoveToSymbolicPositionPrevious_{};

  /**
   * @var jointSymbolicPosition_
   * @brief Represents the symbolic positions assigned to joints within the system.
   *
   * This variable stores the current symbolic positions for the joints, enabling easy mapping of
   * predefined or abstract positions to actual joint configurations. It ensures joint states can
   * be managed and synchronized with symbolic labels or predefined states.
   *
   * @note The symbolic positions are crucial for operations requiring standardized or repeatable
   *       joint configurations, often used in robotics and motion control systems.
   */
  mcx::control3::JointPositions jointSymbolicPosition_{};
  /**
   * @var jointSymbolicPositions_
   * @brief Stores symbolic positions for each joint in the system.
   *
   * This vector contains instances of `SymbolicPosition`, representing abstract or predefined
   * positions for the joints managed by the module. Each element corresponds to a specific joint channel,
   * allowing for customizable symbolic positions to be assigned and used in various robotic operations.
   *
   * @note Symbolic positions are used to map high-level or named configurations to exact
   *       joint positions for simplified control and repeatability.
   *
   * @see SymbolicPosition
   */
  std::vector<SymbolicPosition> jointSymbolicPositions_;
  /**
   * @var syncJointPositions_
   * @brief Stores the synchronized positions of all joints to ensure consistency during operations.
   *
   * This variable is responsible for holding the current synchronized joint positions as used in the system.
   * It plays a crucial role in maintaining coordinated movements across multiple joints, ensuring that
   * positional data is consistent and accurate throughout the control process.
   *
   * @note Used internally within systems where precise synchronization of joint positions is required,
   *       particularly in robotic or motion control scenarios.
   */
  mcx::control3::JointPositions syncJointPositions_{};

  /**
   * @brief Flag indicating the initiation of motion towards a target or symbolic position.
   *
   * The `startMove_` variable is used to determine whether the movement process
   * has been initiated. It acts as a trigger to start the motion execution in
   * systems that control positional or symbolic movement of joints or actuators.
   *
   * @note Default value is `false`, meaning no motion has been started upon initialization.
   */
  bool startMove_{false};
  /**
   * @brief Represents the currently selected symbolic move index.
   *
   * The `selectMove_` variable is used to store the index corresponding to a symbolic move
   * that has been selected for execution or evaluation. This index typically references
   * a position or state defined in the context of symbolic control or movement.
   *
   * @note This variable is integral in determining the active move or operation within
   *       a system utilizing symbolic or predefined states for motion control.
   */
  unsigned int selectMove_{};

  /**
   * @brief A multiplier used to scale the time for motion or setpoint generation.
   *
   * The `timeScaleFactor_` affects the timing and duration of motion operations, allowing
   * for     adjustments to the speed at which movements or transitions are performed. Setting
   * this     factor to a value greater than 1.0 slows down the motion, while values less
   * than 1.0     accelerate it.
   *
   * @default 1.0
   *
   * @note This variable is particularly useful in systems that require dynamic adjustment
   * of motion speeds, such as robotic control systems or animations.
   */
  double timeScaleFactor_{1.0};
  double timeScaleFactorPrev_{1.0}; // default is on...

  /**
   * @brief Represents the output joint positions in a control system.
   *
   * This member variable is used to store and manage the positions of joints, which are controlled
   * or calculated within the control framework. It acts as a container for the current or target
   * state of the joints in the system.
   *
   * @note This variable is typically updated during control cycles and used as an output for further
   *       processing or actuation in the system.
   */
  mcx::control3::JointPositions outputPositions_{};
};

#endif /* CTRL_JOINTPOSITIONMODULE_H */
