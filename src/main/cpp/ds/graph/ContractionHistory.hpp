#pragma once

#include <unordered_map>
#include <vector>

namespace ds {
namespace graph {
/**
 * @brief Information used for the inverse operation of the contraction.
 */
class ContractionHistory {
 public:
  int merge;                                 // vertex to keep (j)
  int merged;                                // vertex to remove (i)
  std::vector<int> removed_black;            // set of i's old neighbors k such that edge ik was black
  std::vector<int> removed_red;              // set of i's old neighbors k such that edge ik was red
  std::vector<int> new_neighbors;            // set of j's new neighbors
  std::vector<int> recolored;                // set of j's neighbors k such that edge jk turns red from black
  std::unordered_map<int, int> ncn_updated;  // changes in the number of common neighbors (ncn)
  std::unordered_map<int, int> ncr_updated;  // changes in the number of common red neighbors (ncr)

  ContractionHistory(int n) : n_(n) {}

  void update_ncn(int i, int j, int d) { ncn_updated[get_id(i, j)] += d; }
  void update_ncr(int i, int j, int d) { ncr_updated[get_id(i, j)] += d; }
  inline std::pair<int, int> decode_id(int key) const { return {key / n_, key % n_}; }

 private:
  int n_;
  inline int get_id(int i, int j) const { return std::min(i, j) * n_ + std::max(i, j); }
};
}  // namespace graph
}  // namespace ds
