/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#include "instrument_description.h"
#include <fstream>
#include <map>
#include <mcx/core.h>

namespace {
using namespace mcx::cmd_line;
const std::map<std::string, FieldDesc> FIELDS = {
    {"categories", {"categories", Field::Required, Container::Single, DataType::Object}},
    {"id", {"id", Field::Required, Container::Single, DataType::Number}},
    {"visualizationNumber", {"visualizationNumber", Field::Optional, Container::Single, DataType::Number}},
    {"pitchScaleFactor", {"pitchScaleFactor", Field::Required, Container::Single, DataType::Number}},
    {"yawScaleFactor", {"yawScaleFactor", Field::Required, Container::Single, DataType::Number}},
    {"rollScaleFactor", {"rollScaleFactor", Field::Required, Container::Single, DataType::Number}},
    {"pinchGain", {"pinchGain", Field::Required, Container::Single, DataType::Number}},
    {"pinchOffset", {"pinchOffset", Field::Required, Container::Single, DataType::Number}},
    {"pitchLength", {"pitchLength", Field::Required, Container::Single, DataType::Number}},
    {"shaftLength", {"shaftLength", Field::Required, Container::Single, DataType::Number}},
    {"yawLength", {"yawLength", Field::Required, Container::Single, DataType::Number}},
    {"totalInstrumentLength", {"totalInstrumentLength", Field::Required, Container::Single, DataType::Number}},
    {"type", {"type", Field::Required, Container::Single, DataType::String}},
    {"name", {"name", Field::Required, Container::Single, DataType::String}},
    {"pinchLimits", {"pinchLimits", Field::Required, Container::Array, DataType::Number}},
    {"pitchLimits", {"pitchLimits", Field::Required, Container::Array, DataType::Number}},
    {"currentLimits", {"currentLimits", Field::Required, Container::Array, DataType::Number}},
    {"backlashCompensation", {"backlashCompensation", Field::Required, Container::Array, DataType::Number}},
    {"transformationMatrix_4x4", {"transformationMatrix_4x4", Field::Required, Container::Matrix, DataType::Number}},
    {"inverseTransformationMatrix_4x4",
     {"inverseTransformationMatrix_4x4", Field::Required, Container::Matrix, DataType::Number}},
};

std::tuple<nlohmann::json, bool> loadFile(const std::string& filename) {
  using namespace mcx;
  nlohmann::json instrumentConfig;
  auto path = utils::trim(filename);
  std::ifstream f(path);
  if (f.fail()) {
    log_error("Failed to find instrument configuration file: {}", path);
    return {};
  }
  try {
    f >> instrumentConfig;
  } catch (...) {
    log_error("Configuration file is broken: {}", path);
    return {};
  }
  return {std::move(instrumentConfig), true};
}

using Matrix4x4 = std::array<std::array<double, 4>, 4>;

template <unsigned int ROW, unsigned COL>
std::array<double, ROW * COL> matrixToVector(const std::array<std::array<double, COL>, ROW> matrix) {
  std::array<double, ROW * COL> vector{};
  auto itr = vector.begin();
  for (const auto& row : matrix) {
    std::copy(row.begin(), row.end(), itr);
    itr += COL;
  }
  return vector;
}

} // namespace

InstrumentData loadInstrument(const auto& el) {
  using namespace mcx::cmd_line;
  InstrumentData instrumentData{};
  instrumentData.id = get<decltype(instrumentData.id)>(el, FIELDS.at("id"), "");
  instrumentData.visualizationNumber =
      get<decltype(instrumentData.visualizationNumber)>(el, FIELDS.at("visualizationNumber"), "");
  fmt::format_to(instrumentData.name, "{}", get<std::string>(el, FIELDS.at("name"), ""));
  instrumentData.pitchScaleFactor =
      get<decltype(instrumentData.pitchScaleFactor)>(el, FIELDS.at("pitchScaleFactor"), "");
  instrumentData.yawScaleFactor = get<decltype(instrumentData.yawScaleFactor)>(el, FIELDS.at("yawScaleFactor"), "");
  instrumentData.rollScaleFactor = get<decltype(instrumentData.rollScaleFactor)>(el, FIELDS.at("rollScaleFactor"), "");
  instrumentData.pinchGain = get<decltype(instrumentData.pinchGain)>(el, FIELDS.at("pinchGain"), "");
  instrumentData.pinchOffset = get<decltype(instrumentData.pinchOffset)>(el, FIELDS.at("pinchOffset"), "");
  instrumentData.pitchLength = get<decltype(instrumentData.pitchLength)>(el, FIELDS.at("pitchLength"), "");
  instrumentData.shaftLength = get<decltype(instrumentData.shaftLength)>(el, FIELDS.at("shaftLength"), "");
  instrumentData.yawLength = get<decltype(instrumentData.yawLength)>(el, FIELDS.at("yawLength"), "");
  instrumentData.totalInstrumentLength =
      get<decltype(instrumentData.totalInstrumentLength)>(el, FIELDS.at("totalInstrumentLength"), "");
  instrumentData.pinchLimits = get<decltype(instrumentData.pinchLimits)>(el, FIELDS.at("pinchLimits"), "");
  instrumentData.pitchLimits = get<decltype(instrumentData.pitchLimits)>(el, FIELDS.at("pitchLimits"), "");
  instrumentData.currentLimits = get<decltype(instrumentData.currentLimits)>(el, FIELDS.at("currentLimits"), "");
  instrumentData.transformationMatrix_4x4 =
      matrixToVector<4, 4>(get<Matrix4x4>(el, FIELDS.at("transformationMatrix_4x4"), ""));
  instrumentData.inverseTransformationMatrix_4x4 =
      matrixToVector<4, 4>(get<Matrix4x4>(el, FIELDS.at("inverseTransformationMatrix_4x4"), ""));

  return instrumentData;
}

std::tuple<InstrumentsMap, bool> loadInstruments(const std::string& filename) {
  auto [json, is_ok] = loadFile(filename);
  if (!is_ok) {
    return {};
  }

  auto categories = get<nlohmann::json>(json, FIELDS.at("categories"), filename);
  InstrumentsMap instruments;
  for (const auto& categoryEl : categories.items()) {
    if (categoryEl.value().is_array()) {
      for (const auto& instrumentEl : categoryEl.value()) {
        auto instrument = loadInstrument(instrumentEl);
        instrument.category = categoryEl.key();
        instruments[instrument.id] = std::move(instrument);
      }
    }
  }

  return {instruments, true};
}

bool instrumentsEnabled(const Config& config) { return config.get<bool>("/Instruments/Enable", Field::Optional, true); }

std::tuple<AvailableInstruments, bool> loadInstruments(const Config& config) {
  AvailableInstruments availableInstruments;
  auto paths = config.paths("/Instruments/Path");
  std::ranges::sort(paths);
  paths.erase(std::ranges::unique(paths).begin(), paths.end());
  availableInstruments.displayInTheTree = config.get<bool>("/Instruments/DisplayInTheTree", Field::Optional, {});
  bool isAllOk = true;
  for (auto path : paths) {
    if (auto [instrumentsMap, isOk] = loadInstruments(path); isOk) {
      if (availableInstruments.instrumentsMap.empty()) {
        availableInstruments.instrumentsMap = std::move(instrumentsMap);
      } else {
        availableInstruments.instrumentsMap.merge(instrumentsMap);
      }
    } else {
      isAllOk = false;
    }
  }
  return {std::move(availableInstruments), isAllOk};
}
