#pragma once

#include <algorithm>
#include <cassert>
#include <map>
#include <vector>

namespace ds {
namespace queue {

/**
 * @brief A priority queue implementation with a uniqueness guarantee.
 *
 * Each element in the queue is the pair of a key and a value (the lower, the better).
 * There are two cases when a key-value pair is pushed into the queue.
 * If the key is not in the queue, the push operation works the same as
 * the standard priority queue does. If the key already exists in the queue, then
 * the value associated with the existing key will be updated, and no new element will be inserted.
 *
 * @tparam T type of comparable values
 */
template <typename T>
class AdaptivePriorityQueue {
 private:
  int const NOT_FOUND = -1;
  int n_;
  std::vector<int> location_;  // mapping from Key to Index
  std::vector<T> values_;      // mapping from Key to Value
  std::vector<int> data_;      // stores a binary tree of Keys

 public:
  AdaptivePriorityQueue(int n) : n_(n), location_(n, NOT_FOUND), values_(n) {}

  std::size_t size() const { return data_.size(); }

  bool empty() const { return size() == 0; }

  void clear() {
    std::fill(location_.begin(), location_.end(), NOT_FOUND);
    std::fill(values_.begin(), values_.end(), T());
    data_.clear();
  }

  void push(int key, T value) {
    assert(0 <= key && key < n_);

    int index = location_[key];
    if (index == NOT_FOUND) {
      // insert
      index = size();
      data_.push_back(key);
      location_[key] = index;
    } else if (values_[key] == value) {
      return;  // same value; early return
    }
    // update value
    values_[key] = value;

    // sort nodes
    if (index > 0 && values_[data_[parent(index)]] > value) {
      bubble_up(index);
    } else {
      bubble_down(index);
    }
  }

  std::pair<int, T> top() const {
    if (empty()) throw std::invalid_argument("empty queue");
    auto key = data_[0];
    return {key, values_[key]};
  }

  void pop() {
    if (empty()) throw std::invalid_argument("empty queue");

    auto key = data_[0];
    if (size() > 1UL) {
      // move the last element to the root
      int j = static_cast<int>(size()) - 1;
      data_[0] = data_[j];
      location_[data_[j]] = 0;
    }
    data_.pop_back();
    location_[key] = NOT_FOUND;

    if (!empty()) bubble_down(0);
  }

 private:
  inline int parent(int index) const { return (index - 1) / 2; }
  inline int left_child(int index) const { return index * 2 + 1; }
  inline int right_child(int index) const { return index * 2 + 2; }

  void swap(int i, int j) {
    if (i == j) return;
    auto x = data_[i];
    auto y = data_[j];
    data_[i] = y;
    data_[j] = x;
    location_[x] = j;
    location_[y] = i;
  }

  void bubble_up(int i) {
    while (i > 0) {
      int j = parent(i);
      if (values_[data_[j]] <= values_[data_[i]]) return;  // done
      swap(i, j);
      i = j;
    }
  }

  void bubble_down(int i) {
    int n = size();
    auto p = values_[data_[i]];

    while (i < n) {
      int j1 = left_child(i);
      int j2 = right_child(i);
      auto p1 = (j1 < n) ? values_[data_[j1]] : 0;
      auto p2 = (j2 < n) ? values_[data_[j2]] : 0;

      if ((j1 >= n || p <= p1) && (j2 >= n || p <= p2)) return;  // done

      // @note j2 < n implies j1 < n because j1 < j2
      int swap_with = (j2 < n && p1 > p2) ? j2 : j1;
      swap(i, swap_with);
      i = swap_with;
    }
  }
};
}  // namespace queue
}  // namespace ds
