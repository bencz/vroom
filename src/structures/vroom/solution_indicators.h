#ifndef SOLUTION_INDICATORS_H
#define SOLUTION_INDICATORS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <array>
#include <tuple>

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "utils/helpers.h"

namespace vroom::utils {

struct SolutionIndicators {
  // priority_counts[p] is the number of assigned tasks with priority
  // p. Priorities are hierarchical: more assigned tasks with a given
  // priority always beats any number of assigned tasks with lower
  // priorities.
  std::array<unsigned, MAX_PRIORITY + 1> priority_counts{};
  unsigned assigned{0};
  Eval eval;
  unsigned used_vehicles{0};
  // Hash based on the ordered sizes of routes in the solution.
  uint32_t routes_hash{0};

  SolutionIndicators() = default;

  template <class Route>
  SolutionIndicators(const Input& input, const std::vector<Route>& sol)
    : SolutionIndicators() {
    Index v_rank = 0;
    for (const auto& r : sol) {
      for (const auto j : r.route) {
        ++priority_counts[input.jobs[j].priority];
      }
      assigned += r.route.size();

      eval += utils::route_eval_for_vehicle(input, v_rank, r.route);
      ++v_rank;

      if (!r.empty()) {
        used_vehicles += 1;
      }
    }

    std::vector<uint32_t> routes_sizes;
    routes_sizes.reserve(sol.size());
    std::ranges::transform(sol,
                           std::back_inserter(routes_sizes),
                           [](const auto& r) { return std::size(r); });
    std::ranges::sort(routes_sizes);
    routes_hash = get_vector_hash(routes_sizes);
  }

  // Compare assigned tasks based on priorities only: -1 if lhs is
  // better, 1 if rhs is better, 0 if equivalent.
  static int compare_priorities(const SolutionIndicators& lhs,
                                const SolutionIndicators& rhs) {
    for (auto p = static_cast<int>(MAX_PRIORITY); p >= 0; --p) {
      if (lhs.priority_counts[p] != rhs.priority_counts[p]) {
        return (lhs.priority_counts[p] > rhs.priority_counts[p]) ? -1 : 1;
      }
    }
    return 0;
  }

  friend bool operator<(const SolutionIndicators& lhs,
                        const SolutionIndicators& rhs) {
    if (const auto c = compare_priorities(lhs, rhs); c != 0) {
      return c < 0;
    }
    return std::tie(rhs.assigned,
                    lhs.eval.cost,
                    lhs.used_vehicles,
                    lhs.eval.duration,
                    lhs.eval.distance,
                    lhs.routes_hash) < std::tie(lhs.assigned,
                                                rhs.eval.cost,
                                                rhs.used_vehicles,
                                                rhs.eval.duration,
                                                rhs.eval.distance,
                                                rhs.routes_hash);
  }
};

} // namespace vroom::utils

#endif
