#pragma once
#include <vector>

#include "util/Random.hpp"
#include "util/util.hpp"

namespace ds {
namespace queue {
/**
 * @brief Specialized adaptive priority queue where
 * all keys and values are integers between 0 and n-1, inclusive.
 */
class BucketQueue {
 private:
  int n_;
  int cursor_;  // cursor_ = n_ iff the queue is empty; otherwise, cursor_ must be a valid Index

  /** Mapping from keys to their values. */
  std::vector<int> value_;  // Key -> Value (many-to-one)

  /** Mapping from keys to their positions in the queue. */
  std::vector<int> perm_;  // Key -> Index (one-to-one)

  /** Mapping from positions in the queue to keys. */
  std::vector<int> perm_inv_;  // Index -> Key (one-to-one)

  /** Starting index for each value. */
  std::vector<int> value_idx_;  // Value -> Index (one-to-one)

  /** Stores the number of valid elements for each value. */
  std::vector<int> value_size_;  // Value -> number of valid elements

  /** Tracks which index has been removed. */
  std::vector<bool> removed_;  // Index -> true if the Index is invalid

 public:
  /**
   * @brief Construct a new Bucket Queue object.
   *
   * @param values list of size n; i-th element is the value of i
   */
  BucketQueue(std::vector<int> const& values)
      : n_(values.size()), cursor_(0), value_(values), perm_(n_), value_idx_(n_ + 1), value_size_(n_), removed_(n_) {
    // initialization
    auto n = values.size();
    std::vector<std::vector<int>> bucket(n);
    for (std::size_t i = 0; i < n; ++i) bucket[values[i]].push_back(i);

    for (std::size_t i = 0; i < n; ++i) {
      value_size_[i] = bucket[i].size();
      value_idx_[i] = perm_inv_.size();
      for (std::size_t j = 0; j < bucket[i].size(); ++j) {
        perm_[bucket[i][j]] = perm_inv_.size();  // set perm before growing perm_inv
        perm_inv_.push_back(bucket[i][j]);
      }
    }
    value_idx_[n] = n;
  }

  std::size_t size() const { return n_ - cursor_; }

  bool empty() const { return size() == 0; }

  /**
   *
   * @brief Returns the minimum value in the queue.
   *
   * @return int minimum value
   *
   * Time complexity: O(1)
   */
  int min_value() const {
    if (cursor_ == n_) throw std::invalid_argument("empty queue");
    return value_[perm_inv_[cursor_]];
  }

  /**
   * @brief Returns the value associated with the given key.
   *
   * @param key key
   * @return int value
   *
   * Time complexity: O(1)
   */
  int get_value(int key) const {
    if (perm_[key] < cursor_) throw std::invalid_argument("element already removed");
    return value_[key];
  }

  /**
   * @brief Returns a key with the minimum value.
   *
   * @return int key having the minimum value; if there are ties, returns one of them
   *
   * Time complexity: O(1)
   */
  int get_key_with_min_value() const {
    if (cursor_ == n_) throw std::invalid_argument("empty queue");
    return perm_inv_[cursor_];
  }

  /**
   * @brief Returns a uniformly random key with the minimum value.
   *
   * @return int key having the minimum value
   *
   * Time complexity: O(1)
   */
  int get_random_key_with_min_value(util::Random& rand) const {
    if (cursor_ == n_) throw std::invalid_argument("empty queue");
    return perm_inv_[rand.randint(cursor_, cursor_ + value_size_[min_value()] - 1)];
  }

  /**
   * @brief Removes an element in the queue.
   *
   * @param key key to remove
   * @return int value of the removed key
   */
  int remove(int key) {
    int value = value_[key];
    int index = perm_[key];
    if (removed_[index]) return value;  // do nothing

    int last_index = get_last_index(value);
    swap_perm(index, last_index);
    remove_index(last_index, value);

    for (int v = value; v >= 0 && value_size_[v] == 0; --v) value_idx_[v] = value_idx_[v + 1];
    return value;
  }

  /**
   * @brief Removes a key with the minimum value.
   *
   * @return removed key
   *
   * Time complexity: O(1)
   */
  int remove_min() {
    if (cursor_ == n_) throw std::invalid_argument("empty queue");
    assert(!removed_[cursor_]);

    int key = perm_inv_[cursor_];
    int value = value_[key];
    remove_index(cursor_, value);

    return key;
  }

  /**
   * @brief Updates the value associated with a specific key.
   *
   * @param key key to update its value
   * @param diff amount of (new value - current value)
   *
   * Time complexity: O(diff)
   */
  void update(int key, int diff) {
    assert(!removed_[cursor_]);
    int p = value_[key];
    int idx = perm_[key];
    if (removed_[idx]) throw std::invalid_argument("element already removed");
    if (p + diff >= n_) throw std::invalid_argument("new value too high");
    if (p + diff < 0) throw std::invalid_argument("new value too low");

    // update value
    value_[key] += diff;

    while (diff != 0) {
      if (diff > 0) {
        int new_idx = --value_idx_[p + 1];
        swap_perm(idx, new_idx);

        if (removed_[idx]) {  // there may be a gap; perform another swap
          swap_perm(idx, get_last_index(p));
        }
        idx = new_idx;
        --value_size_[p];
        ++value_size_[p + 1];
        if (value_size_[p] == 0) value_idx_[p] = value_idx_[p + 1];
        ++p;
        --diff;
      } else {
        if (value_idx_[p] < cursor_) value_idx_[p] = cursor_;

        int new_idx = std::max(cursor_, value_idx_[p - 1]) + value_size_[p - 1];
        swap_perm(idx, new_idx);

        if (removed_[idx]) {  // there may be a gap; perform another swap
          swap_perm(idx, value_idx_[p]);
        }
        idx = new_idx;
        ++value_idx_[p];
        --value_size_[p];
        ++value_size_[p - 1];
        --p;
        ++diff;
      }
    }
    forward_cursor();
  }

 private:
  int get_last_index(int value) { return std::max(cursor_, value_idx_[value]) + value_size_[value] - 1; }

  void forward_cursor() {
    while (cursor_ < n_ && removed_[cursor_]) ++cursor_;
  }

  void remove_index(int index, int value) {
    removed_[index] = true;
    --value_size_[value];
    forward_cursor();
  }

  int swap_perm(int index, int swap_with) {
    if (index == swap_with) return index;
    int k1 = perm_inv_[index];
    int k2 = perm_inv_[swap_with];
    perm_[k1] = swap_with;
    perm_[k2] = index;
    perm_inv_[index] = k2;
    perm_inv_[swap_with] = k1;

    // swap removed
    if (removed_[index] != removed_[swap_with]) {
      removed_[index] = !removed_[index];
      removed_[swap_with] = !removed_[swap_with];
    }
    return swap_with;
  }
};
}  // namespace queue
}  // namespace ds
