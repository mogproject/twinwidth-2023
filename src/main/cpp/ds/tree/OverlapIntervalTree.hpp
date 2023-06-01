#pragma once
#include <cassert>
#include <vector>

namespace ds {
namespace tree {

/**
 * @brief Complete binary tree to dynamically compute the maximum number of overlaps
 * given a multiset of intervals.
 *
 */
class OverlapIntervalTree {
 private:
  int n_;
  std::vector<int> node_id_;
  std::vector<int> node_id_inv_;
  std::vector<int> delta_;
  std::vector<int> total_;
  std::vector<int> max_overlap_;

 public:
  OverlapIntervalTree(int n)
      : n_(n), node_id_(n + 1), node_id_inv_(n + 2), delta_(n + 2), total_(n + 2), max_overlap_(n + 2) {
    assert(n > 0);

    // in-order traversal to construct a mapping: value [0,n] -> node id [1,n+1]
    int value = 0;
    std::vector<int> st = {1};

    while (!st.empty()) {
      int x = st.back();
      st.pop_back();

      if (x > 0) {
        int left = 2 * x;
        int right = 2 * x + 1;
        if (right < n + 2) st.push_back(right);
        st.push_back(-x);
        if (left < n + 2) st.push_back(left);
      } else {
        node_id_[value] = -x;
        node_id_inv_[-x] = value;
        ++value;
      }
    }
  }

  /**
   * @brief Returns n.
   * 
   * @return int n
   */
  int n() const { return n_; }

  /**
   * @brief Returns the max number of overlaps.
   *
   * @return int max overlaps
   *
   * Time complexity: O(1)
   */
  int max_overlap() const { return max_overlap_[1]; }

  /**
   * @brief Returns the number of overlaps at the given time.
   *
   * @param t point to check
   * @return int number of overlaps
   */
  int get_overlap(int t) const {
    assert(0 <= t && t < n_);

    int ret = 0;
    for (int x = node_id_[t]; x > 0; x /= 2) {
      if (node_id_inv_[x] <= t) {
        int left = x * 2;
        int t_left = (left < n_ + 2) ? total_[left] : 0;
        ret += t_left + delta_[x];
      }
    }
    return ret;
  }

  /**
   * @brief Adds the interval [a,b) to the current multiset.
   *
   * @param a a
   * @param b b
   *
   * Time complexity: O(log n)
   */
  void add_interval(int a, int b) { update_interval(a, b, 1); }

  /**
   * @brief Removes the interval [a,b) from the current multiset.
   *
   * @param a a
   * @param b b
   *
   * Time complexity: O(log n)
   */
  void remove_interval(int a, int b) { update_interval(a, b, -1); }

 private:
  void update_interval(int a, int b, int diff) {
    assert(0 <= a && a < b && b <= n_);

    std::vector<int> xs = {node_id_[a], node_id_[b]};
    delta_[xs[0]] += diff;
    delta_[xs[1]] -= diff;

    for (auto x : xs) {
      while (x > 0) {
        int left = x * 2;
        int right = x * 2 + 1;

        // update the sum of delta in the subtree
        int t_left = (left < n_ + 2) ? total_[left] : 0;
        int t_right = (right < n_ + 2) ? total_[right] : 0;
        total_[x] = delta_[x] + t_left + t_right;

        // update max overlap
        int m_left = (left < n_ + 2) ? max_overlap_[left] : 0;
        int m_right = (right < n_ + 2) ? max_overlap_[right] : 0;
        max_overlap_[x] = std::max(m_left, std::max(t_left + delta_[x], t_left + delta_[x] + m_right));

        x /= 2;
      }
    }
  }
};
}  // namespace tree
}  // namespace ds
