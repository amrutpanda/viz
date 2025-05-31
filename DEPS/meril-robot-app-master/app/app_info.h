/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef MOTORCORTEX_ROBOT_APPINFO_H
#define MOTORCORTEX_ROBOT_APPINFO_H

#include "app_feature.h"
#include "app_version.h"
#include <mcx/control3.h>
#include <mcx/core.h>
#include <mcx/math.h>

class AppInfo final : public mcx::container::Module {
public:
  AppInfo() = delete;

  AppInfo(const feature::App& features, mcx::cmd_line::SystemMode mode, bool licenseValid)
      : appFeatures_{features}, mode_{mode == mcx::cmd_line::SystemMode::SIMULATION ? "Simulation" : "Production"},
        licenseValid_{licenseValid} {}

  ~AppInfo() override = default;

private:
  void create_(const char* name, mcx::parameter_server::Parameter* parameterServer, uint64_t dtMicroS) override {}

  bool initPhase1_() override {
    addParameter("mode", mcx::parameter_server::ParameterType::OUTPUT, mode_.data(), mode_.size());
    addParameter("licenseValid", mcx::parameter_server::ParameterType::OUTPUT, &licenseValid_);
    addParameter("features", mcx::parameter_server::ParameterType::OUTPUT, &appFeatures_);
    addParameter("versions/app", mcx::parameter_server::ParameterType::OUTPUT, (char*)mcx::meril_robot::version(),
                 strlen(mcx::meril_robot::version()));
    addParameter("versions/core", mcx::parameter_server::ParameterType::OUTPUT, (char*)mcx::utils::version(),
                 strlen(mcx::utils::version()));
    addParameter("versions/math", mcx::parameter_server::ParameterType::OUTPUT, (char*)mcx::math::version(),
                 strlen(mcx::math::version()));
    addParameter("versions/control", mcx::parameter_server::ParameterType::OUTPUT, (char*)mcx::control3::version(),
                 strlen(mcx::control3::version()));
    return true;
  }

  bool initPhase2_() override { return true; }

  bool startOp_() override { return true; }

  bool stopOp_() override { return true; }

  bool iterateOp_(const mcx::container::TaskTime& systemTime, mcx::container::UserTime* userTime) override {
    return true;
  }

  feature::App appFeatures_;
  std::string mode_;
  bool licenseValid_{false};
};

#endif /* MOTORCORTEX_ROBOT_APPINFO_H */