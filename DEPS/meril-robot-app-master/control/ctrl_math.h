/*
 * All rights reserved. Copyright (c) 2014-2023 VECTIONEER B.V.
 *
 * This is proprietary software.
 * Modification, duplication, creation of derivative works, (re-)distribution are strictly prohibited
 * unless explicitly permitted in writing.
 *
 * This header must be left in place with the code at all times.
 */

#ifndef MOTORCORTEX_ROBOT_CTRL_MATH_H
#define MOTORCORTEX_ROBOT_CTRL_MATH_H

#include "mcx/core/utl_span.h"
#include <valarray>

namespace control {
template <typename Operator>
double operation(double arg1) {
  return arg1;
}

template <typename Operator, typename... Args>
double operation(double arg1, Args... args) {
  constexpr auto OP = Operator{};
  return OP(arg1, operation<Operator>(args...));
}

template <typename Operator, typename... Args>
void apply_v(std::valarray<double>& result, const mcx::utils::span<const double>& span1, const Args&... spanN) {
  const auto size = result.size();
  for (unsigned int i = 0; i < size; i++) {
    result[i] = operation<Operator>(span1[i], spanN[i]...);
  }
}

template <typename... Args>
void plus(std::valarray<double>& result, const mcx::utils::span<const double>& span1, const Args&... spanN) {
  apply_v<std::plus<double>>(result, span1, spanN...);
}

template <typename... Args>
void minus(std::valarray<double>& result, const mcx::utils::span<const double>& span1, const Args&... spanN) {
  apply_v<std::plus<double>>(result, spanN...);
  apply_v<std::minus<double>>(result, span1, result);
}
} // namespace control

#endif // MOTORCORTEX_ROBOT_CTRL_MATH_H
