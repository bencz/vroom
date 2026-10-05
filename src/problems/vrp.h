#ifndef VRP_H
#define VRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <atomic>
#include <mutex>
#include <numeric>
#include <ranges>
#include <set>
#include <thread>

#include "algorithms/heuristics/heuristics.h"
#include "algorithms/local_search/local_search.h"
#include "structures/vroom/eval.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution/solution.h"

namespace vroom {

template <class Route>
std::vector<Route> set_init_sol(const Input& input,
                                std::unordered_set<Index>& init_assigned) {
  std::vector<Route> init_sol;
  init_sol.reserve(input.vehicles.size());

  for (Index v = 0; v < input.vehicles.size(); ++v) {
    init_sol.emplace_back(input, v, input.zero_amount().size());
  }

  if (input.has_initial_routes()) {
    heuristics::set_initial_routes<Route>(input, init_sol, init_assigned);
  }

  return init_sol;
}

template <class Route> struct SolvingContext {
  std::unordered_set<Index> init_assigned;
  const std::vector<Route> init_sol;
  std::set<Index> unassigned;
  std::vector<Index> vehicles_ranks;
  std::vector<std::vector<Route>> solutions;
  std::vector<utils::SolutionIndicators> sol_indicators;

  // Time spent in heuristic for each search.
  std::vector<std::chrono::milliseconds> heuristic_times;

  SolvingContext(const Input& input, unsigned nb_searches)
    : init_sol(set_init_sol<Route>(input, init_assigned)),
      vehicles_ranks(input.vehicles.size()),
      solutions(nb_searches, init_sol),
      sol_indicators(nb_searches),
      heuristic_times(nb_searches) {

    // Deduce unassigned jobs from initial solution.
    std::ranges::copy_if(std::views::iota(0u, input.jobs.size()),
                         std::inserter(unassigned, unassigned.begin()),
                         [this](const Index j) {
                           return !init_assigned.contains(j);
                         });

    // Heuristics will operate on all vehicles.
    std::iota(vehicles_ranks.begin(), vehicles_ranks.end(), 0);
  }

  // A heuristic solution is considered a duplicate if it has already
  // been found for a lower rank. This only depends on searches
  // parameters, not on threads scheduling.
  bool heuristic_solution_already_found(unsigned rank) const {
    assert(rank < sol_indicators.size());
    return std::ranges::any_of(std::views::iota(0u, rank),
                               [&](const unsigned other_rank) {
                                 return !(sol_indicators[other_rank] <
                                          sol_indicators[rank]) &&
                                        !(sol_indicators[rank] <
                                          sol_indicators[other_rank]);
                               });
  }
};

template <class Route>
void run_heuristic(const Input& input,
                   const HeuristicParameters& p,
                   const unsigned rank,
                   SolvingContext<Route>& context) {
  const auto heuristic_start = utils::now();

  switch (p.heuristic) {
  case HEURISTIC::BASIC:
    heuristics::basic<Route>(input,
                             context.solutions[rank],
                             context.unassigned,
                             context.vehicles_ranks,
                             p.init,
                             p.regret_coeff,
                             p.sort);
    break;
  case HEURISTIC::DYNAMIC:
    heuristics::dynamic_vehicle_choice<Route>(input,
                                              context.solutions[rank],
                                              context.unassigned,
                                              context.vehicles_ranks,
                                              p.init,
                                              p.regret_coeff,
                                              p.sort);
    break;
  }

  if (!input.has_homogeneous_costs() && p.sort == SORT::AVAILABILITY) {
    // Worth trying another vehicle ordering scheme in
    // heuristic.
    std::vector<Route> other_sol = context.init_sol;

    switch (p.heuristic) {
    case HEURISTIC::BASIC:
      heuristics::basic<Route>(input,
                               other_sol,
                               context.unassigned,
                               context.vehicles_ranks,
                               p.init,
                               p.regret_coeff,
                               SORT::COST);
      break;
    case HEURISTIC::DYNAMIC:
      heuristics::dynamic_vehicle_choice<Route>(input,
                                                other_sol,
                                                context.unassigned,
                                                context.vehicles_ranks,
                                                p.init,
                                                p.regret_coeff,
                                                SORT::COST);
      break;
    }

    // Compare whole solutions, accounting for assigned jobs and
    // priorities, not only cost.
    if (utils::SolutionIndicators(input, other_sol) <
        utils::SolutionIndicators(input, context.solutions[rank])) {
      context.solutions[rank] = std::move(other_sol);
    }
  }

  // Store heuristic solution indicators.
  context.sol_indicators[rank] =
    utils::SolutionIndicators(input, context.solutions[rank]);

  context.heuristic_times[rank] =
    std::chrono::duration_cast<std::chrono::milliseconds>(utils::now() -
                                                          heuristic_start);
}

template <class Route, class LocalSearch>
void run_local_search(const Input& input,
                      const unsigned rank,
                      const unsigned depth,
                      const Timeout& search_time,
                      SolvingContext<Route>& context) {
  Timeout ls_search_time;
  if (search_time.has_value()) {
    const auto heuristic_time = context.heuristic_times[rank];

    if (search_time.value() <= heuristic_time) {
      // No time left for local search!
      return;
    }

    ls_search_time = search_time.value() - heuristic_time;
  }

  // Local search phase.
  LocalSearch ls(input, context.solutions[rank], depth, ls_search_time);
  ls.run();

  // Store solution indicators.
  context.sol_indicators[rank] = ls.indicators();
}

class VRP {
  // Abstract class describing a VRP (vehicle routing problem).
protected:
  const Input& _input;

  template <class Route, class LocalSearch>
  Solution solve(
    unsigned nb_searches,
    const unsigned depth,
    const unsigned nb_threads,
    const Timeout& timeout,
    const std::vector<HeuristicParameters>& homogeneous_parameters,
    const std::vector<HeuristicParameters>& heterogeneous_parameters) const {
    const auto& parameters = (_input.has_homogeneous_locations())
                               ? homogeneous_parameters
                               : heterogeneous_parameters;
    assert(nb_searches != 0);
    nb_searches =
      std::min(nb_searches, static_cast<unsigned>(parameters.size()));

    SolvingContext<Route> context(_input, nb_searches);

    const auto actual_nb_threads = std::min(nb_searches, nb_threads);
    assert(actual_nb_threads > 0);

    Timeout search_time;
    if (timeout.has_value()) {
      // Max number of solving per thread.
      const auto dv = std::div(static_cast<long>(nb_searches),
                               static_cast<long>(actual_nb_threads));
      const unsigned max_solving_number = dv.quot + ((dv.rem == 0) ? 0 : 1);
      search_time = timeout.value() / max_solving_number;
    }

    // Run f on all provided ranks using a pool of threads.
    auto run_on_ranks = [actual_nb_threads](const std::vector<unsigned>& ranks,
                                            const auto& f) {
      std::atomic<std::size_t> next_rank_index{0};
      std::exception_ptr ep = nullptr;
      std::mutex ep_m;

      auto worker = [&]() {
        for (auto i = next_rank_index++; i < ranks.size();
             i = next_rank_index++) {
          try {
            f(ranks[i]);
          } catch (...) {
            const std::scoped_lock<std::mutex> lock(ep_m);
            ep = std::current_exception();
          }
        }
      };

      {
        std::vector<std::jthread> threads;
        const auto nb_workers =
          std::min(static_cast<std::size_t>(actual_nb_threads), ranks.size());
        threads.reserve(nb_workers);
        for (std::size_t t = 0; t < nb_workers; ++t) {
          threads.emplace_back(worker);
        }
      }

      if (ep != nullptr) {
        std::rethrow_exception(ep);
      }
    };

    // Heuristics phase.
    std::vector<unsigned> all_ranks(nb_searches);
    std::iota(all_ranks.begin(), all_ranks.end(), 0);

    run_on_ranks(all_ranks, [&](const unsigned rank) {
      run_heuristic<Route>(_input, parameters[rank], rank, context);
    });

    // Skip local search for duplicate heuristic solutions. This is
    // done after all heuristics are computed so that the outcome does
    // not depend on threads scheduling.
    std::vector<unsigned> ls_ranks;
    std::ranges::copy_if(all_ranks,
                         std::back_inserter(ls_ranks),
                         [&](const unsigned rank) {
                           return !context.heuristic_solution_already_found(
                             rank);
                         });

    // Local search phase.
    run_on_ranks(ls_ranks, [&](const unsigned rank) {
      run_local_search<Route, LocalSearch>(_input,
                                           rank,
                                           depth,
                                           search_time,
                                           context);
    });

    auto best_indic = std::min_element(context.sol_indicators.cbegin(),
                                       context.sol_indicators.cend());

    return utils::
      format_solution(_input,
                      context.solutions[std::distance(context.sol_indicators
                                                        .cbegin(),
                                                      best_indic)]);
  }

public:
  explicit VRP(const Input& input);

  virtual ~VRP();

  virtual Solution solve(unsigned nb_searches,
                         unsigned depth,
                         unsigned nb_threads,
                         const Timeout& timeout) const = 0;
};

} // namespace vroom

#endif
