/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>
#include <limits>

#include "algorithms/kruskal.h"

namespace vroom::utils {

template <class T>
UndirectedGraph<T> minimum_spanning_tree(const Matrix<T>& m) {
  // Prim algorithm on a dense symmetric matrix, in O(n^2) time and
  // O(n) additional memory, with no need to build and sort all
  // edges.
  const std::size_t n = m.size();

  std::vector<Edge<T>> mst;
  if (n < 2) {
    return UndirectedGraph<T>(std::move(mst));
  }
  mst.reserve(n - 1);

  // key[v] is the lowest weight of an edge between v and the
  // current tree, parent[v] is the matching tree vertex.
  std::vector<T> key(n, std::numeric_limits<T>::max());
  std::vector<Index> parent(n, 0);
  std::vector<unsigned char> in_tree(n, false);

  Index current = 0;
  for (std::size_t step = 0; step < n; ++step) {
    in_tree[current] = true;
    if (step > 0) {
      mst.emplace_back(parent[current], current, m[parent[current]][current]);
    }

    // Update keys based on new tree vertex and spot next vertex.
    T best_key = std::numeric_limits<T>::max();
    Index next = current;
    for (std::size_t v = 0; v < n; ++v) {
      if (in_tree[v]) {
        continue;
      }
      if (m[current][v] < key[v]) {
        key[v] = m[current][v];
        parent[v] = current;
      }
      if (key[v] < best_key) {
        best_key = key[v];
        next = static_cast<Index>(v);
      }
    }
    current = next;
  }
  assert(mst.size() == n - 1);

  return UndirectedGraph<T>(std::move(mst));
}

template UndirectedGraph<UserCost>
minimum_spanning_tree(const Matrix<UserCost>& m);

} // namespace vroom::utils
