/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef MOTORCORTEX_ROBOT_APP_FEATURE_H
#define MOTORCORTEX_ROBOT_APP_FEATURE_H

#include <mcx/core.h>
#include <string>

namespace feature {

struct Manipulator {
  BEGIN_VISITABLES(Manipulator);
  VISITABLE(bool, enabled);
  VISITABLE(int, userFrames);
  VISITABLE(bool, tool6DofAdmittance);
  VISITABLE(unsigned int, numberOfJoints);
  END_VISITABLES;
};

struct Machine {
  BEGIN_VISITABLES(Machine);
  VISITABLE(bool, enabled);
  VISITABLE(unsigned int, numberOfJoints);
  END_VISITABLES;
};

struct Axes {
  BEGIN_VISITABLES(Axes);
  VISITABLE(bool, enabled);
  VISITABLE(unsigned int, numberOfAxes);
  VISITABLE(unsigned int, numberOfActuators);
  END_VISITABLES;
};

struct SafetyChannelSelector {
  BEGIN_VISITABLES(SafetyChannelSelector);
  VISITABLE(bool, enabled);
  VISITABLE(unsigned int, numberOfChannels);
  END_VISITABLES;
};

struct App {
  BEGIN_VISITABLES(App);
  VISITABLE(int, signalGenerators);
  VISITABLE(int, setpointGenerators);
  VISITABLE(int, joysticks);
  VISITABLE(int, udp);
  VISITABLE(Axes, axes);
  VISITABLE(Axes, instrumentAxes);
  VISITABLE(Machine, machine);
  VISITABLE(Manipulator, manipulator);
  VISITABLE(SafetyChannelSelector, safetyChannelSelector);
  END_VISITABLES;
};

inline int getAmount(const mcx::cmd_line::Config& config, const std::string& feature,
                     const std::string& keyWord = "Amount") {
  auto configAmount = config.get<int>(fmt::format("/{}/{}", feature, keyWord));
  log_info("App Feature: adding {} {}", configAmount, feature);
  return configAmount;
}

template <typename T>
std::tuple<std::unique_ptr<T[]>, int> create(mcx::parameter_server::Parameter& paramServer, mcx::container::Task& task,
                                             const mcx::cmd_line::Config& config, const std::string& feature,
                                             const std::string& folder) {
  std::unique_ptr<T[]> modules;
  int amount = getAmount(config, feature);
  if (amount > 0) {
    modules = std::make_unique<T[]>(amount);
    auto path = paramServer.createPath(folder.c_str());
    for (int cnt = 0; cnt < amount; cnt++) {
      modules[cnt].create(fmt::format("{}{:02d}", feature.c_str(), cnt + 1).c_str(), path);
      task.add(&modules[cnt]);
    }
  }
  return {std::move(modules), amount};
}

inline Manipulator getManipulator(const mcx::cmd_line::Config& config, const std::string& feature) {
  Manipulator manipulator{};
  const auto configEnabled = config.get<bool>(fmt::format("/{}/Enable", feature));
  manipulator.enabled = configEnabled;
  manipulator.tool6DofAdmittance = getAmount(config, "Tool6DofAdmittance") > 0,
  manipulator.numberOfJoints = getAmount(config, "ManipulatorControl", "NumberOfJoints"),
  manipulator.userFrames = getAmount(config, "UserFrame");
  return manipulator;
}

inline Machine getMachine(const mcx::cmd_line::Config& config, const std::string& feature) {
  const auto amount = getAmount(config, "MachineControl", "NumberOfJoints");
  return {.enabled = amount > 0, .numberOfJoints = static_cast<unsigned int>(amount)};
}

inline Axes getAxes(const mcx::cmd_line::Config& config, const std::string& name = "AxesControl") {
  const auto enabled = config.get<bool>(fmt::format("/{}/Enable", name), mcx::cmd_line::Field::Optional, true);
  const auto amountAxes = getAmount(config, name, "NumberOfAxes");
  auto amountActuators = getAmount(config, name, "NumberOfActuators");
  if (amountActuators == 0) {
    amountActuators = amountAxes;
  }
  return {.enabled = amountAxes > 0 && enabled,
          .numberOfAxes = static_cast<unsigned int>(amountAxes),
          .numberOfActuators = static_cast<unsigned int>(amountActuators)};
}

inline SafetyChannelSelector getSafetyChannelSelector(const mcx::cmd_line::Config& config, const std::string& feature) {
  const auto configEnabled = config.get<bool>(fmt::format("/{}/Enable", feature));
  const auto amount = getAmount(config, feature, "NumberOfChannels");
  return {.enabled = configEnabled && (amount > 0), .numberOfChannels = static_cast<unsigned int>(amount)};
}

} // namespace feature

#endif // MOTORCORTEX_ROBOT_APP_FEATURE_H
