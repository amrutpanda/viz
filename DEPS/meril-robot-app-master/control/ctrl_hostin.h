/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef MCX_CTRL_HOST_IN
#define MCX_CTRL_HOST_IN

#include <mcx/control3/ctrl_joints.h>
#include <mcx/control3/ctrl_watchdog.h>
#include <mcx/core.h>
#include <mcx/math.h>

namespace control {

template <typename T>
struct Filter {
  unsigned int order{2};
  double beta{M_SQRT1_2}; // 0.707 [-]
  double omega{5 * M_PI}; // 2.5 [Hz]
  std::array<T, 2> output{};
};

struct HostInWatchdog {
  static constexpr double DEFAULT_TIMEOUT_SEC{0.01}; // 10 [ms]
  HostInWatchdog() { wd.timeoutSec = DEFAULT_TIMEOUT_SEC; }
  mcx::parameter_server::ParamHandle handle;
  mcx::control3::Watchdog<bool> wd;
  bool enable{true};
};

struct HostInEnableFader {
  static constexpr double DEFAULT_FADE_TIME_SEC{0.1}; // 100 [ms]
  double inTime{DEFAULT_FADE_TIME_SEC};
  double outTime{DEFAULT_FADE_TIME_SEC};
  double value{};
};

struct HostInTracking {
  bool relative{true};       // input
  bool global{true};         // input
  double factor{1};          // input
  bool currentEnable{};      // input
  bool previousEnable{};     // internal
  bool relativeRisingEdge{}; // internal
  bool absoluteRisingEdge{}; // internal
  bool isTracking{};         // output
  HostInWatchdog watchdog;
  HostInEnableFader enableFader;
};

struct HostInPosition {
  explicit HostInPosition(size_t nChannels)
      : sync(nChannels), input(nChannels), inputCopy(nChannels), previousInput(nChannels), output(nChannels) {
    for (auto& out : filter.output) {
      out.resize(nChannels);
    }
  }
  HostInTracking track{};

  mcx::control3::JointPositions sync;
  mcx::control3::JointPositions input;
  mcx::control3::JointPositions inputCopy;
  mcx::control3::JointPositions previousInput;
  mcx::control3::JointPositions output;

  bool reset{};
  bool enable{};
  bool disable{false};
  bool enabled{};
  bool reverseDirection{false};

  Filter<mcx::control3::JointPositions> filter;
};

struct HostInOrientation {
  HostInTracking track{};

  HostInOrientation() : sync(3), input(3), output(3), offsetEulerZYX0(3), offsetEulerZYX1(3), deltaEuler(3) {}

  // Used for Euler:
  mcx::control3::JointPositions sync{};
  mcx::control3::JointPositions input{};
  mcx::control3::JointPositions output{};
  mcx::control3::JointPositions offsetEulerZYX0{};
  mcx::control3::JointPositions offsetEulerZYX1{};
  mcx::control3::JointPositions deltaEuler{};
  mcx::parameter_server::ParamHandle offsetEulerHandle0;
  mcx::parameter_server::ParamHandle offsetEulerHandle1;

  /* math::Quaternion quaternionSync{}; */
  mcx::math::Quaternion inputQ{};
  mcx::math::Quaternion previousInputQ{};
  mcx::math::Quaternion outputQ{};
  bool enableInputQ{false}; // parameter
  bool previousEnableInputQ{false};

  bool reset{};
  bool enable{};
  bool disable{false};
  bool enabled{};
  bool enableHandcontrollerOrientationOffset{false};
  double rzOrientationOffsetTotal{};
  double rzOrientationOffsetExternal{};
  bool reverseDirection{false};

  Filter<mcx::math::Quaternion> filter;
};

/**
 * @brief The HostInModule Control Object multiplies the inputs by the gain parameter and then adds the offset
 * parameter.
 *
 *
 * @copydetails HostInModule()
 *
 * @dummy
 * @param[in] input[·] - the input signals
 * @param[out] output[·] - the output signals
 * @param gain - the gain the input signals are multiplied with
 * @param offset - the offset applied to the output
 *
 * First the input is multiplied by the gain, then the offset is added.
 */

template <typename T = HostInPosition>
class HostInModule final : public mcx::container::Module {
public:
  /**
   * @brief Creates a new HostInModule Object
   * @param number_of_channels - number of channels (N)
   */
  HostInModule(size_t numberOfChannels = 1);

  ~HostInModule() override = default;

  /**
   * @brief Sets the input.
   * @param input - new value of the input.
   */
  inline void setReset(bool input) { hostIn_.reset = input; }

  /**
   * @brief Sets the input.
   * @param input - new value of the input.
   */
  inline void setInput(mcx::utils::span<const double> input) { mcx::utils::span<double>{hostIn_.input} = input; }

  /**
   * @brief Sets the input of a specific channel
   * @param input - new value of the input.
   * @param channel - the channel to set the input of (default = 0)
   */
  inline void setInput(double input, size_t channel = 0) {
    log_assert((channel < numberOfChannels_),
               "HostInModule: setInput, channel out of bounds!") if (channel < numberOfChannels_) {
      hostIn_.input[channel] = input;
    }
  };

  /**
   * @brief Sets the input.
   * @param input - new value of the input.
   */
  inline void setReference(mcx::utils::span<const double> input) { mcx::utils::span<double>{hostIn_.sync} = input; }

  /**
   * @brief Sets the input of a specific channel
   * @param input - new value of the input.
   * @param channel - the channel to set the input of (default = 0)
   */
  inline void setReference(double input, size_t channel = 0) {
    log_assert((channel < numberOfChannels_),
               "HostInModule: setInput, channel out of bounds!") if (channel < numberOfChannels_) {
      hostIn_.sync[channel] = input;
    }
  };

  /**
   * @brief Sets the enable.
   * @param input - new value of enable.
   */
  inline void setEnable(bool input) { hostIn_.enable = input; }

  /**
   * @brief Returns the current output.
   * @return actual value of the output.
   */
  inline mcx::utils::span<const double> getOutput() const { return hostIn_.output; };

  inline bool getIsTracking() const { return hostIn_.track.isTracking; }

  inline bool getRelativeRisingEdge() const { return hostIn_.track.relativeRisingEdge; }

  inline bool getAbsoluteRisingEdge() const { return hostIn_.track.absoluteRisingEdge; }

  // Implemented for HostInOrientation only:
  mcx::math::Quaternion getOutputQuaternion() const;

  void setRzOrientationOffsetExternal(double angle);

protected:
  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override {};

  bool initPhase1_() override;

  bool initPhase2_() override { return true; }

  bool startOp_() override {
    hostIn_.track.watchdog.wd.init(getDtSec(), false);
    return true;
  }

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override;

  bool stopOp_() override { return true; }

protected:
  size_t numberOfChannels_;

  T hostIn_;
};

} // namespace control

#endif /* MCX_CTRL_HOST_IN */
