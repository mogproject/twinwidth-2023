#pragma once
#include <cassert>
#include <vector>

namespace ds {
namespace tree {

/**
 * @brief Complete binary tree to dynamically compute the max element.
 *
 */
class MaxElementTree {
 private:
  int n_;
  std::vector<int> data_;
  std::vector<int> max_element_;

 public:
  MaxElementTree(int n) : n_(n), data_(n + 1), max_element_(n + 1) { assert(n > 0); }

  /**
   * @brief Returns the max element in the tree.
   *
   * @return int max element
   *
   * Time complexity: O(1)
   */
  int max_element() const { return max_element_[1]; }

  /**
   * @brief Updates an element in the tree.
   *
   * @param index index of the element
   * @param value new value of the element
   *
   * Time complexity: O(log n)
   */
  void update(int index, int value) {
    assert(value >= 0);
    int i = index + 1;
    if (data_[i] == value) return;  // no change

    data_[i] = value;  // update value

    for (; i > 0; i /= 2) {
      int v = data_[i];
      // compare with left and right subtrees
      for (int c = i * 2; c <= std::min(i * 2 + 1, n_); ++c) v = std::max(v, max_element_[c]);
      max_element_[i] = v;
    }
  }
};
}  // namespace tree
}  // namespace ds
