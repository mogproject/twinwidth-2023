#pragma once

#include <vector>

#include "ds/set/FastSet.hpp"

namespace ds {
namespace graph {
class ContractionInfo {
 public:
  int last_merge;
  int last_merged;
  std::vector<int> common_neighbors;
  std::vector<std::pair<int, int>> updated_pairs;

 private:
  int n_;
  ds::FastSet fs_;

 public:
  ContractionInfo(int n) : n_(n), fs_(n * n) {}

  void clear() {
    last_merge = -1;
    last_merged = -1;
    updated_pairs.clear();
    common_neighbors.clear();
    fs_.clear();
  }

  void add_pair(int i, int j) {
    int id = get_id(i, j);
    if (!fs_.get(id)) {
      fs_.set(id);
      updated_pairs.push_back({std::min(i, j), std::max(i, j)});
    }
  }

 private:
  int get_id(int i, int j) { return std::min(i, j) * n_ + std::max(i, j); }
};
}  // namespace graph
}  // namespace ds
