#ifndef LOCAL_SEARCH_H
#define LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <random>

#include "structures/vroom/solution_indicators.h"
#include "structures/vroom/solution_state.h"

namespace vroom::ls {

template <class Route,
          class UnassignedExchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class IntraTwoOpt,
          class PDShift,
          class RouteExchange,
          class SwapStar,
          class RouteSplit,
          class PriorityReplace,
          class TSPFix>
class LocalSearch {
private:
  const Input& _input;
  const std::size_t _nb_vehicles;

  const unsigned _depth;
  const Deadline _deadline;

  std::optional<unsigned> _completed_depth;
  std::vector<Index> _all_routes;

  utils::SolutionState _sol_state;

  std::vector<Route> _sol;

  std::vector<Route>& _best_sol;
  utils::SolutionIndicators _best_sol_indicators;

  std::unordered_set<Index> try_job_additions(const std::vector<Index>& routes,
                                              double regret_coeff);

  void run_ls_step();

  // _close_routes[v1][v2] is true if a job in route for vehicle v1 is
  // among closest neighbors of a job in route for vehicle v2 (or the
  // other way around). Only used when some locations have no
  // coordinates, otherwise route bounding boxes are used.
  std::vector<std::vector<unsigned char>> _close_routes;
  std::vector<Index> _job_vehicle;
  void update_close_routes();

  // Decide whether moves between routes for source and target are
  // worth checking, based on routes proximity.
  bool routes_may_interact(Index source, Index target) const;

  // Compute "cost" between route at rank v_target and job with rank r
  // in route at rank v. Relies on
  // _sol_state.cheapest_job_rank_in_routes_* being up to date.
  Eval job_route_cost(Index v_target, Index v, Index r);

  // Compute lower bound for the cost of relocating job at rank r
  // (resp. jobs at rank r1 and r2) in route v to any other
  // (compatible) route.
  Eval relocate_cost_lower_bound(Index v, Index r);
  Eval relocate_cost_lower_bound(Index v, Index r1, Index r2);

  void remove_from_routes();

  // Try to assign unassigned jobs by removing lower-priority jobs
  // from a route, for a net priority gain. Return true if solution
  // has been modified.
  bool try_priority_improvements();

  // Update all solution state data for route v.
  void update_route_state(Index v);

  // Used once the regular perturbation scheme is exhausted while
  // there is still time left: randomly remove strings of consecutive
  // jobs in a few close routes.
  std::mt19937 _rng;
  void remove_random_strings();

public:
  LocalSearch(const Input& input,
              std::vector<Route>& tw_sol,
              unsigned depth,
              const Timeout& timeout);

  utils::SolutionIndicators indicators() const;

  void run();
};

} // namespace vroom::ls

#endif
