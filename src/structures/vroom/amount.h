#ifndef AMOUNT_H
#define AMOUNT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <array>
#include <cassert>
#include <memory>

#include "structures/typedefs.h"

namespace vroom {

template <typename E> class AmountExpression {
public:
  Capacity operator[](size_t i) const {
    return static_cast<const E&>(*this)[i];
  };
  std::size_t size() const {
    return static_cast<const E&>(*this).size();
  };
  bool empty() const {
    return size() == 0;
  };
};

// Lexicographical comparison, useful for situations where a total
// order is required.
template <typename E1, typename E2>
bool operator<(const AmountExpression<E1>& lhs,
               const AmountExpression<E2>& rhs) {
  assert(lhs.size() == rhs.size());
  if (lhs.empty()) {
    return false;
  }
  auto last_rank = lhs.size() - 1;
  for (std::size_t i = 0; i < last_rank; ++i) {
    if (lhs[i] < rhs[i]) {
      return true;
    }
    if (lhs[i] > rhs[i]) {
      return false;
    }
  }
  return lhs[last_rank] < rhs[last_rank];
}

template <typename E1, typename E2>
bool operator<=(const AmountExpression<E1>& lhs,
                const AmountExpression<E2>& rhs) {
  bool is_inf = true;
  assert(lhs.size() == rhs.size());
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i] > rhs[i]) {
      is_inf = false;
      break;
    }
  }

  return is_inf;
}

template <typename E1, typename E2>
bool operator==(const AmountExpression<E1>& lhs,
                const AmountExpression<E2>& rhs) {
  bool is_equal = true;
  assert(lhs.size() == rhs.size());
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i] != rhs[i]) {
      is_equal = false;
      break;
    }
  }

  return is_equal;
}

class Amount : public AmountExpression<Amount> {
  // Values are stored inline for usual small sizes in order to avoid
  // heap allocations in hot code paths, with heap storage as a
  // fallback for bigger sizes.
  static constexpr std::size_t INLINE_CAPACITY = 4;

  std::size_t _size{0};
  std::size_t _capacity{INLINE_CAPACITY};
  std::array<Capacity, INLINE_CAPACITY> _inline_elems{};
  std::unique_ptr<Capacity[]> _heap_elems;

  Capacity* data() {
    return _heap_elems ? _heap_elems.get() : _inline_elems.data();
  }

  const Capacity* data() const {
    return _heap_elems ? _heap_elems.get() : _inline_elems.data();
  }

  // Resize, discarding current values.
  void resize_for_overwrite(std::size_t size) {
    if (size > _capacity) {
      _heap_elems = std::make_unique_for_overwrite<Capacity[]>(size);
      _capacity = size;
    }
    _size = size;
  }

public:
  Amount() = default;

  explicit Amount(std::size_t size) {
    resize_for_overwrite(size);
    std::fill_n(data(), size, 0);
  }

  Amount(const Amount& other) {
    resize_for_overwrite(other._size);
    std::copy_n(other.data(), other._size, data());
  }

  Amount(Amount&& other) noexcept
    : _size(other._size),
      _capacity(other._capacity),
      _inline_elems(other._inline_elems),
      _heap_elems(std::move(other._heap_elems)) {
    other._size = 0;
    other._capacity = INLINE_CAPACITY;
  }

  template <typename E> Amount(const AmountExpression<E>& u) {
    resize_for_overwrite(u.size());
    auto* elems = data();
    for (std::size_t i = 0; i < _size; ++i) {
      elems[i] = u[i];
    }
  }

  ~Amount() = default;

  Amount& operator=(const Amount& other) {
    if (this != &other) {
      resize_for_overwrite(other._size);
      std::copy_n(other.data(), other._size, data());
    }
    return *this;
  }

  Amount& operator=(Amount&& other) noexcept {
    if (this != &other) {
      _size = other._size;
      _capacity = other._capacity;
      _inline_elems = other._inline_elems;
      _heap_elems = std::move(other._heap_elems);
      other._size = 0;
      other._capacity = INLINE_CAPACITY;
    }
    return *this;
  }

  // Evaluate expression in place, reusing existing storage. Safe
  // with expressions involving *this as each value only depends on
  // values at the same rank.
  template <typename E> Amount& operator=(const AmountExpression<E>& u) {
    const auto size = u.size();
    if (size > _capacity) {
      // Can't evaluate in place without overwriting values that may
      // be used by u.
      *this = Amount(u);
      return *this;
    }
    _size = size;
    auto* elems = data();
    for (std::size_t i = 0; i < size; ++i) {
      elems[i] = u[i];
    }
    return *this;
  }

  void push_back(Capacity c) {
    if (_size == _capacity) {
      const auto new_capacity = 2 * _capacity;
      auto new_elems = std::make_unique_for_overwrite<Capacity[]>(new_capacity);
      std::copy_n(data(), _size, new_elems.get());
      _heap_elems = std::move(new_elems);
      _capacity = new_capacity;
    }
    data()[_size] = c;
    ++_size;
  }

  Capacity operator[](std::size_t i) const {
    assert(i < _size);
    return data()[i];
  }

  Capacity& operator[](std::size_t i) {
    assert(i < _size);
    return data()[i];
  }

  std::size_t size() const {
    return _size;
  }

  Amount& operator+=(const Amount& rhs) {
    assert(this->size() == rhs.size());
    auto* elems = data();
    const auto* rhs_elems = rhs.data();
    for (std::size_t i = 0; i < _size; ++i) {
      elems[i] += rhs_elems[i];
    }
    return *this;
  }

  Amount& operator-=(const Amount& rhs) {
    assert(this->size() == rhs.size());
    auto* elems = data();
    const auto* rhs_elems = rhs.data();
    for (std::size_t i = 0; i < _size; ++i) {
      elems[i] -= rhs_elems[i];
    }
    return *this;
  }

#if USE_PYTHON_BINDINGS
  Capacity* get_data() {
    return data();
  };
#endif

  template <class AmountExpression>
  Amount& operator+=(const AmountExpression& rhs) {
    assert(this->size() == rhs.size());
    for (std::size_t i = 0; i < this->size(); ++i) {
      (*this)[i] += rhs[i];
    }
    return *this;
  }

  template <class AmountExpression>
  Amount& operator-=(const AmountExpression& rhs) {
    assert(this->size() == rhs.size());
    for (std::size_t i = 0; i < this->size(); ++i) {
      (*this)[i] -= rhs[i];
    }
    return *this;
  }
};

template <typename E1, typename E2>
class AmountSum : public AmountExpression<AmountSum<E1, E2>> {
  const E1& lhs;
  const E2& rhs;

public:
  AmountSum(const E1& a, const E2& b) : lhs(a), rhs(b) {
    assert(a.size() == b.size());
  }

  Capacity operator[](std::size_t i) const {
    return lhs[i] + rhs[i];
  }

  std::size_t size() const {
    return lhs.size();
  }
};

template <typename E1, typename E2>
auto operator+(const AmountExpression<E1>& u, const AmountExpression<E2>& v)
  -> AmountSum<AmountExpression<E1>, AmountExpression<E2>> {
  return {u, v};
}

template <typename E1, typename E2>
class AmountDiff : public AmountExpression<AmountDiff<E1, E2>> {
  const E1& lhs;
  const E2& rhs;

public:
  AmountDiff(const E1& a, const E2& b) : lhs(a), rhs(b) {
    assert(a.size() == b.size());
  }

  Capacity operator[](std::size_t i) const {
    return lhs[i] - rhs[i];
  }

  std::size_t size() const {
    return lhs.size();
  }
};

template <typename E1, typename E2>
auto operator-(const AmountExpression<E1>& lhs, const AmountExpression<E2>& rhs)
  -> AmountDiff<AmountExpression<E1>, AmountExpression<E2>> {
  return {lhs, rhs};
}

} // namespace vroom

#endif
