/*
 * All rights reserved. Copyright (c) 2014-2024 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef INSTRUMENT_DESCRIPTION_H
#define INSTRUMENT_DESCRIPTION_H

#include <array>
#include <cstdint>
#include <map>
#include <mcx/core/visit_struct_intrusive.h>
#include <string>

using INSTRUMENT_ID_TYPE = uint16_t;
static constexpr INSTRUMENT_ID_TYPE NO_INSTRUMENT_ID = -1;

using Vec2 = std::array<double, 2>;
using Vec4 = std::array<double, 4>;
using Vec16 = std::array<double, 16>;
using Char256 = char[256];
struct InstrumentData {
  BEGIN_VISITABLES(InstrumentData);
  VISITABLE(INSTRUMENT_ID_TYPE, id);
  VISITABLE(INSTRUMENT_ID_TYPE, visualizationNumber);
  VISITABLE(double, pitchScaleFactor);
  VISITABLE(double, yawScaleFactor);
  VISITABLE(double, rollScaleFactor);
  VISITABLE(double, pinchGain);
  VISITABLE(double, pinchOffset);
  VISITABLE(double, pitchLength);
  VISITABLE(double, shaftLength);
  VISITABLE(double, yawLength);
  VISITABLE(double, totalInstrumentLength);
  VISITABLE(Vec2, pinchLimits);
  VISITABLE(Vec2, pitchLimits);
  VISITABLE(Vec4, currentLimits);
  VISITABLE(Vec4, backlashCompensation);
  VISITABLE(Vec16, transformationMatrix_4x4);
  VISITABLE(Vec16, inverseTransformationMatrix_4x4);
  VISITABLE(Char256, name);
  END_VISITABLES;
  std::string category;
};

using InstrumentsMap = std::map<unsigned int, InstrumentData>;

struct AvailableInstruments {
  InstrumentsMap instrumentsMap{};
  bool displayInTheTree{};
};

std::tuple<InstrumentsMap, bool> loadInstruments(const std::string& filename);

namespace mcx::cmd_line {
class Config;
}

bool instrumentsEnabled(const mcx::cmd_line::Config& config);

std::tuple<AvailableInstruments, bool> loadInstruments(const mcx::cmd_line::Config& config);

#endif // INSTRUMENT_DESCRIPTION_H
