/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/cost_wrapper.h"
#include "utils/exception.h"
#include "utils/helpers.h"
#include <cmath>

namespace vroom {

namespace {

double checked_speed_factor(double speed_factor) {
  // Written to also reject NaN values.
  if (!(0 < speed_factor && speed_factor <= MAX_SPEED_FACTOR)) {
    throw InputException(std::format("Invalid speed factor: {}", speed_factor));
  }
  return speed_factor;
}

template <typename T> T checked_factor(double factor) {
  // Make sure factors are exactly representable and leave enough
  // room for multiplications with matrix values.
  constexpr double max_factor = static_cast<double>(1ULL << 53);
  if (!(factor < max_factor)) {
    throw InputException("Too high values for vehicle costs.");
  }
  return static_cast<T>(std::round(factor));
}

} // namespace

CostWrapper::CostWrapper(double speed_factor, Cost per_hour, Cost per_km)
  : _per_hour(per_hour),
    _per_km(per_km),
    discrete_duration_factor(checked_factor<Duration>(
      1 / checked_speed_factor(speed_factor) * DURATION_FACTOR)),
    discrete_duration_cost_factor(checked_factor<Cost>(
      1 / speed_factor * DURATION_FACTOR * static_cast<double>(per_hour))),
    discrete_distance_cost_factor(
      checked_factor<Cost>(static_cast<double>(DISTANCE_FACTOR * per_km))) {
}

void CostWrapper::set_durations_matrix(const Matrix<UserDuration>* matrix) {
  duration_matrix_size = matrix->size();
  duration_data = (*matrix)[0];
}

void CostWrapper::set_distances_matrix(const Matrix<UserDistance>* matrix) {
  distance_matrix_size = matrix->size();
  distance_data = (*matrix)[0];
}

void CostWrapper::set_costs_matrix(const Matrix<UserCost>* matrix,
                                   bool reset_cost_factor) {
  cost_matrix_size = matrix->size();
  cost_data = (*matrix)[0];

  if (reset_cost_factor) {
    discrete_duration_cost_factor = DURATION_FACTOR * COST_FACTOR;
    discrete_distance_cost_factor = 0;
    _cost_based_on_metrics = false;
  }
}

UserCost CostWrapper::user_cost_from_user_metrics(UserDuration d,
                                                  UserDistance m) const {
  assert(_cost_based_on_metrics);
  constexpr auto SECONDS_PER_HOUR = 3600;
  constexpr auto M_PER_KM = 1000;
  return utils::round<UserCost>(static_cast<double>(d * _per_hour) /
                                  SECONDS_PER_HOUR +
                                static_cast<double>(m * _per_km) / M_PER_KM);
}

} // namespace vroom
