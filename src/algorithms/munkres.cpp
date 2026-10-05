/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <cassert>
#include <limits>
#include <tuple>
#include <vector>

#include "algorithms/munkres.h"

namespace vroom::utils {

template <class T>
std::map<Index, Index> minimum_weight_perfect_matching(const Matrix<T>& m) {
  // Hungarian algorithm in O(n^3) using potentials (rows: x, columns:
  // y). Signed 64 bits values are used to avoid any overflow with
  // big (e.g. infinite on diagonal) unsigned weights. Indices below
  // are 1-based, 0 being used as a sentinel column.
  const std::size_t n = m.size();
  constexpr Cost INF = std::numeric_limits<Cost>::max() / 4;

  std::vector<Cost> u(n + 1, 0);
  std::vector<Cost> v(n + 1, 0);
  // p[j] is the row matched with column j.
  std::vector<std::size_t> p(n + 1, 0);
  std::vector<std::size_t> way(n + 1, 0);
  std::vector<Cost> min_v(n + 1);
  std::vector<unsigned char> used(n + 1);

  for (std::size_t i = 1; i <= n; ++i) {
    p[0] = i;
    std::size_t j0 = 0;
    std::ranges::fill(min_v, INF);
    std::ranges::fill(used, false);

    do {
      used[j0] = true;
      const std::size_t i0 = p[j0];
      Cost delta = INF;
      std::size_t j1 = 0;

      for (std::size_t j = 1; j <= n; ++j) {
        if (used[j]) {
          continue;
        }
        const Cost current = static_cast<Cost>(m[i0 - 1][j - 1]) - u[i0] - v[j];
        if (current < min_v[j]) {
          min_v[j] = current;
          way[j] = j0;
        }
        if (min_v[j] < delta) {
          delta = min_v[j];
          j1 = j;
        }
      }

      for (std::size_t j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          min_v[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    // Augmenting path.
    do {
      const std::size_t j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::map<Index, Index> matching_xy;
  for (std::size_t j = 1; j <= n; ++j) {
    matching_xy.emplace(static_cast<Index>(p[j] - 1),
                        static_cast<Index>(j - 1));
  }
  return matching_xy;
}

template <class T>
std::map<Index, Index> greedy_symmetric_approx_mwpm(const Matrix<T>& m) {
  // Fast greedy algorithm for finding a symmetric perfect matching,
  // choosing always smaller possible value, no minimality
  // assured. Matrix size should be even!
  assert(m.size() % 2 == 0);

  // Sorting all candidate pairs once by (weight, i, j), then picking
  // pairs with both vertices still available yields the same
  // matching as repeatedly searching for the smallest weight among
  // remaining vertices.
  struct Candidate {
    T weight;
    Index i;
    Index j;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(m.size() * (m.size() - 1) / 2);
  for (Index i = 0; i < m.size(); ++i) {
    for (Index j = i + 1; j < m.size(); ++j) {
      candidates.push_back({m[i][j], i, j});
    }
  }
  std::ranges::sort(candidates, [](const auto& lhs, const auto& rhs) {
    return std::tie(lhs.weight, lhs.i, lhs.j) <
           std::tie(rhs.weight, rhs.i, rhs.j);
  });

  std::map<Index, Index> matching;
  std::vector<unsigned char> matched(m.size(), false);
  for (const auto& c : candidates) {
    if (!matched[c.i] && !matched[c.j]) {
      matching.emplace(c.i, c.j);
      matched[c.i] = true;
      matched[c.j] = true;
    }
  }

  return matching;
}

template std::map<Index, Index>
minimum_weight_perfect_matching(const Matrix<UserCost>& m);

template std::map<Index, Index>
greedy_symmetric_approx_mwpm(const Matrix<UserCost>& m);

} // namespace vroom::utils
