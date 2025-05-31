/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "ctrl_hostin.h"

namespace control {

template <>
HostInModule<HostInPosition>::HostInModule(size_t numberOfChannels)
    : numberOfChannels_(numberOfChannels), hostIn_(numberOfChannels) {}

template <>
HostInModule<HostInOrientation>::HostInModule(size_t numberOfChannels) : numberOfChannels_(6) {}

template <>
bool HostInModule<HostInPosition>::initPhase1_() {
  using ParamType = mcx::parameter_server::ParameterType;

  addParameterVec("input", ParamType::INPUT, hostIn_.input);
  addParameterVec("output", ParamType::OUTPUT, hostIn_.output);
  addParameter("enable", ParamType::INPUT, &hostIn_.enable);
  addParameter("disable", ParamType::PARAMETER, &hostIn_.disable);
  addParameter("reset", ParamType::OUTPUT, &hostIn_.reset);
  addParameter("enabled", ParamType::OUTPUT, &hostIn_.enabled);
  addParameter("reverseDirection", ParamType::PARAMETER, &hostIn_.reverseDirection);
  hostIn_.track.watchdog.handle = addParameter("track/enable", ParamType::INPUT, &hostIn_.track.currentEnable);
  addParameter("track/watchdog/timeout", ParamType::PARAMETER, &hostIn_.track.watchdog.wd.timeoutSec);
  addParameter("track/watchdog/enable", ParamType::PARAMETER, &hostIn_.track.watchdog.enable);
  addParameter("track/enableFader/inTime", ParamType::PARAMETER, &hostIn_.track.enableFader.inTime);
  addParameter("track/enableFader/outTime", ParamType::PARAMETER, &hostIn_.track.enableFader.outTime);
  addParameter("track/enableFader/value", ParamType::OUTPUT, &hostIn_.track.enableFader.value);
  addParameter("track/relative", ParamType::INPUT, &hostIn_.track.relative);
  addParameter("track/factor", ParamType::INPUT, &hostIn_.track.factor);
  addParameter("track/isTracking", ParamType::OUTPUT, &hostIn_.track.isTracking);
  addParameter("filter/order", ParamType::PARAMETER, &hostIn_.filter.order);
  addParameter("filter/omega", ParamType::PARAMETER, &hostIn_.filter.omega);
  addParameter("filter/beta", ParamType::PARAMETER, &hostIn_.filter.beta);
  addParameterVec("filter/output", ParamType::OUTPUT, hostIn_.filter.output[0]);

  return true;
}

template <>
bool HostInModule<HostInOrientation>::initPhase1_() {
  using ParamType = mcx::parameter_server::ParameterType;

  addParameterVec("eulerZYX/input", ParamType::INPUT, hostIn_.input);
  hostIn_.offsetEulerHandle0 = addParameterVec("eulerZYX/offset0", ParamType::PARAMETER, hostIn_.offsetEulerZYX0);
  hostIn_.offsetEulerHandle1 = addParameterVec("eulerZYX/offset1", ParamType::PARAMETER, hostIn_.offsetEulerZYX1);
  addParameterVec("eulerZYX/output", ParamType::OUTPUT, hostIn_.output);
  addParameterVec("quaternion/input", ParamType::INPUT, hostIn_.inputQ);
  addParameterVec("quaternion/output", ParamType::OUTPUT, hostIn_.outputQ);
  addParameter("enable", ParamType::INPUT, &hostIn_.enable);
  addParameter("disable", ParamType::PARAMETER, &hostIn_.disable);
  addParameter("reset", ParamType::OUTPUT, &hostIn_.reset);
  addParameter("enabled", ParamType::OUTPUT, &hostIn_.enabled);
  addParameter("reverseDirection", ParamType::PARAMETER, &hostIn_.reverseDirection);
  addParameter("enableHandcontrollerOrientationOffset", ParamType::PARAMETER,
               &hostIn_.enableHandcontrollerOrientationOffset);
  addParameter("rzOrientationOffsetTotal", ParamType::OUTPUT, &hostIn_.rzOrientationOffsetTotal);
  addParameter("rzOrientationOffsetExternal", ParamType::OUTPUT, &hostIn_.rzOrientationOffsetExternal);
  addParameter("quaternion/enable", ParamType::PARAMETER, &hostIn_.enableInputQ);
  hostIn_.track.watchdog.handle = addParameter("track/enable", ParamType::INPUT, &hostIn_.track.currentEnable);
  addParameter("track/watchdog/timeout", ParamType::PARAMETER, &hostIn_.track.watchdog.wd.timeoutSec);
  addParameter("track/watchdog/enable", ParamType::PARAMETER, &hostIn_.track.watchdog.enable);
  addParameter("track/enableFader/inTime", ParamType::PARAMETER, &hostIn_.track.enableFader.inTime);
  addParameter("track/enableFader/outTime", ParamType::PARAMETER, &hostIn_.track.enableFader.outTime);
  addParameter("track/enableFader/value", ParamType::OUTPUT, &hostIn_.track.enableFader.value);
  addParameter("track/relative", ParamType::PARAMETER, &hostIn_.track.relative);
  addParameter("track/factor", ParamType::PARAMETER, &hostIn_.track.factor);
  addParameter("track/isTracking", ParamType::OUTPUT, &hostIn_.track.isTracking);
  addParameter("filter/order", ParamType::PARAMETER, &hostIn_.filter.order);
  addParameter("filter/omega", ParamType::PARAMETER, &hostIn_.filter.omega);
  addParameter("filter/beta", ParamType::PARAMETER, &hostIn_.filter.beta);
  addParameterVec("filter/output", ParamType::OUTPUT, hostIn_.filter.output[0]);

  return true;
}

template <>
bool HostInModule<HostInPosition>::iterateOp_(const mcx::container::TaskTime& systemTime,
                                              mcx::container::UserTime* userTime) {
  hostIn_.track.isTracking = false;
  hostIn_.track.relativeRisingEdge = false;
  hostIn_.track.absoluteRisingEdge = false;

  hostIn_.inputCopy = hostIn_.input; // make sure is copy
  auto& filter = hostIn_.filter;
  // Filter orientation input
  filter.beta = (filter.beta < 0) ? -filter.beta : filter.beta;
  filter.omega = (filter.omega < 0) ? -filter.omega : filter.omega;
  const double dt = getDtSec();
  switch (filter.order) {
  case 1: {
    // 1st order: yk = yk-1 + alfa * (xk - yk-1);
    const double alfa = filter.omega * dt / (1 + filter.omega * dt);
    filter.output[1] = filter.output[0];
    filter.output[0] += alfa * (hostIn_.inputCopy - filter.output[0]);
    break;
  }
  case 2: {
    // 2nd order
    const double n0 = dt * dt * filter.omega * filter.omega;
    const double d0 = 1 + 2 * filter.beta * dt * filter.omega + n0;
    // y(k) = y(k-1) + n0 / d0 * (x(k) - y(k-1)) + 1/d0 * (y(k-1) - y(k-2));
    // In the following expression do not use keyword auto!!!
    // Ensure the correct type for following (not _Expr<_BinClos ....>) so calculation is performed correctly:
    const decltype(filter.output)::value_type dyy = filter.output[0] - filter.output[1];
    filter.output[1] = filter.output[0];
    filter.output[0] += (n0 / d0) * (hostIn_.inputCopy - filter.output[0]) + (1 / d0) * dyy;
    break;
  }
  default:
    filter.order = 0;
    filter.output[1] = filter.output[0];
    filter.output[0] = hostIn_.inputCopy;
    break;
  }
  hostIn_.inputCopy = filter.output[0];

  if (hostIn_.reverseDirection) {
    hostIn_.inputCopy *= -1;
  }

  if (auto& watchdog = hostIn_.track.watchdog; watchdog.enable) {
    watchdog.wd.check(watchdog.handle, hostIn_.track.currentEnable);
  }
  hostIn_.enabled = hostIn_.enable & hostIn_.track.currentEnable & !hostIn_.disable;

  hostIn_.track.factor = std::max(0.0, hostIn_.track.factor);

  auto& fader = hostIn_.track.enableFader;
  fader.inTime = std::max(0.0, fader.inTime);
  fader.outTime = std::max(0.0, fader.outTime);
  const double fadeDelta = hostIn_.enabled ? (fader.inTime > 0 ? getDtSec() / fader.inTime : 1)
                                           : (fader.outTime > 0 ? -getDtSec() / fader.outTime : -1);
  fader.value = std::min(std::max(fader.value + fadeDelta, 0.0), 1.0);
  const bool enable = fader.value > 0;

  if (hostIn_.track.relative) { // Relative
    if (hostIn_.reset) {
      // Sync trajectory
      hostIn_.output = hostIn_.sync;
    }
    if (enable) {
      hostIn_.track.isTracking = true;
      hostIn_.track.relativeRisingEdge = !hostIn_.track.previousEnable;
      if (hostIn_.track.relativeRisingEdge) { // Rising edge of "enable"
        hostIn_.previousInput = hostIn_.inputCopy;
      }
      for (unsigned int n = 0; n < numberOfChannels_; ++n) {
        hostIn_.output[n] += fader.value * hostIn_.track.factor * (hostIn_.inputCopy[n] - hostIn_.previousInput[n]);
      }
    }
  } else { // Direct
    if (hostIn_.reset) {
      // Sync trajectory if not outputEngaged
      hostIn_.output = hostIn_.sync;
    } else if (enable) {
      hostIn_.track.absoluteRisingEdge = !hostIn_.track.previousEnable;
      hostIn_.output = hostIn_.inputCopy;
    }
  }

  hostIn_.track.previousEnable = enable;
  hostIn_.previousInput = hostIn_.inputCopy;

  return true;
}

template <>
bool HostInModule<HostInOrientation>::iterateOp_(const mcx::container::TaskTime& systemTime,
                                                 mcx::container::UserTime* userTime) {
  hostIn_.track.isTracking = false;
  hostIn_.track.relativeRisingEdge = false;
  hostIn_.track.absoluteRisingEdge = false;
  hostIn_.track.factor = std::max(0.0, hostIn_.track.factor);

  const auto eulerSync = hostIn_.sync;
  const auto eulerInput = hostIn_.input;
  auto& eulerOutput = hostIn_.output;
  auto& filter = hostIn_.filter;

  auto& outputQ = hostIn_.outputQ;
  const auto& previousInputQ = hostIn_.previousInputQ;

  if (auto& watchdog = hostIn_.track.watchdog; watchdog.enable) {
    watchdog.wd.check(watchdog.handle, hostIn_.track.currentEnable);
  }
  hostIn_.enabled = hostIn_.enable & hostIn_.track.currentEnable & !hostIn_.disable;
  // Switched input between euler and quaternion.
  // Need to reset for the (special) case they are a different orientation
  const bool forceRisingEdge = hostIn_.offsetEulerHandle0.isUpdated() || hostIn_.offsetEulerHandle1.isUpdated() ||
                               (hostIn_.previousEnableInputQ != hostIn_.enableInputQ);
  hostIn_.previousEnableInputQ = hostIn_.enableInputQ;

  mcx::math::Quaternion inputQ(eulerInput[0], eulerInput[1], eulerInput[2]);
  if (hostIn_.enableInputQ) {
    inputQ = hostIn_.inputQ;
  }
  inputQ.normalize();

  if (forceRisingEdge) {
    // Reset filter
    for (auto& out : filter.output) {
      out = inputQ;
    }
  }

  // Filter orientation input
  filter.beta = (filter.beta < 0) ? -filter.beta : filter.beta;
  filter.omega = (filter.omega < 0) ? -filter.omega : filter.omega;
  const double dt = getDtSec();
  switch (filter.order) {
  case 1: {
    // 1st order: yk = yk-1 + alfa * (xk - yk-1);
    const double alfa = filter.omega * dt / (1 + filter.omega * dt);
    filter.output[1] = filter.output[0];
    filter.output[0] = filter.output[0].slerp(inputQ, alfa);
    if ((filter.output[1] * filter.output[0].transpose()).w() < 0) {
      filter.output[0] *= -1;
    }
    break;
  }
  case 2: {
    // 2nd order: y(k) = y(k-1) + n0 / d0 * (x(k) - y(k-1)) + 1/d0 * (y(k-1) - y(k-2));
    const double n0 = dt * dt * filter.omega * filter.omega;
    const double d0 = 1 + 2 * filter.beta * dt * filter.omega + n0;
    auto dxy = inputQ * filter.output[0].transpose();
    dxy.scaleAngle(n0 / d0);
    auto dyy = filter.output[0] * filter.output[1].transpose();
    dyy.scaleAngle(1 / d0);

    filter.output[1] = filter.output[0];
    filter.output[0] = dyy * dxy * filter.output[0];
    if ((filter.output[1] * filter.output[0].transpose()).w() < 0) {
      filter.output[0] *= -1;
    }
    break;
  }
  default:
    filter.order = 0;
    filter.output[1] = filter.output[0];
    filter.output[0] = inputQ;
    break;
  }

  const mcx::math::Quaternion offsetQ0(hostIn_.offsetEulerZYX0[0], hostIn_.offsetEulerZYX0[1],
                                       hostIn_.offsetEulerZYX0[2]);
  const mcx::math::Quaternion offsetQ1(hostIn_.offsetEulerZYX1[0], hostIn_.offsetEulerZYX1[1],
                                       hostIn_.offsetEulerZYX1[2]);
  // out = Global rotation * in * Body rotation
  inputQ = offsetQ1.transpose() * filter.output[0] * offsetQ1;
  inputQ = offsetQ0 * inputQ;

  auto& fader = hostIn_.track.enableFader;
  fader.inTime = std::max(0.0, fader.inTime);
  fader.outTime = std::max(0.0, fader.outTime);
  const double fadeDelta = hostIn_.enabled ? (fader.inTime > 0 ? getDtSec() / fader.inTime : 1)
                                           : (fader.outTime > 0 ? -getDtSec() / fader.outTime : -1);
  fader.value = std::min(std::max(fader.value + fadeDelta, 0.0), 1.0);

  bool enable = fader.value > 0;
  if (forceRisingEdge) {
    enable = false;
  }

  if (hostIn_.track.relative) { // Relative
    if (hostIn_.reset) {
      // Reset
      eulerOutput = eulerSync;
      outputQ.set(eulerSync[0], eulerSync[1], eulerSync[2]);
    }
    if (enable) {
      hostIn_.track.isTracking = true;
      hostIn_.track.relativeRisingEdge = !hostIn_.track.previousEnable;

      if (hostIn_.track.relativeRisingEdge) { // Rising edge of "enable"
        hostIn_.previousInputQ = inputQ;
      }

      // calculate the delta input
      auto qD = previousInputQ.transpose() * inputQ;
      qD.normalize();
      const double angle = mcx::math::normalizeAngle(qD.getAngle() * hostIn_.track.factor);
      qD.setAngle(angle);

      if (hostIn_.enableHandcontrollerOrientationOffset) {
        // adjust the body Rx and Ry for the deviation in Rz such that they are aligned with the input again
        hostIn_.rzOrientationOffsetTotal = hostIn_.deltaEuler[0] + hostIn_.rzOrientationOffsetExternal;
        const mcx::math::Quaternion offsetRz(hostIn_.rzOrientationOffsetTotal, 0.0, 0.0);
        qD = offsetRz.transpose() * qD * offsetRz;
      }

      if (hostIn_.reverseDirection) {
        qD = qD.transpose();
      }

      // calculate the output
      const mcx::math::Quaternion qF = outputQ * qD;
      outputQ = outputQ.slerp(qF, fader.value);
      outputQ.normalize();

      // for consistency:
      outputQ.getEuler(eulerOutput[0], eulerOutput[1], eulerOutput[2]);
    }
  } else { // Direct
    if (hostIn_.reset) {
      // Reset
      eulerOutput = eulerSync;
      outputQ.set(eulerSync[0], eulerSync[1], eulerSync[2]);
    } else if (enable) {
      hostIn_.track.absoluteRisingEdge = !hostIn_.track.previousEnable;
      eulerOutput = eulerInput;
      outputQ.set(eulerInput[0], eulerInput[1], eulerInput[2]);
    }
  }

  const mcx::math::Quaternion deltaQ = inputQ.transpose() * outputQ;
  deltaQ.getEuler(hostIn_.deltaEuler[0], hostIn_.deltaEuler[1], hostIn_.deltaEuler[2]);

  hostIn_.track.previousEnable = enable;
  hostIn_.previousInputQ = inputQ;

  return true;
}

template <>
mcx::math::Quaternion HostInModule<HostInOrientation>::getOutputQuaternion() const {
  return hostIn_.outputQ;
}

template <>
void HostInModule<HostInOrientation>::setRzOrientationOffsetExternal(double angle) {
  hostIn_.rzOrientationOffsetExternal = angle;
}

template class HostInModule<HostInPosition>;

template class HostInModule<HostInOrientation>;

} // namespace control
