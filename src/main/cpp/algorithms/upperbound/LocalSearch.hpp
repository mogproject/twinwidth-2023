#pragma once

#include "algorithms/base/SolverInfo.hpp"
#include "ds/graph/TimelineEncoding.hpp"
#include "util/Random.hpp"
#include "util/logger.hpp"

namespace algorithms {
namespace upperbound {

class LocalSearch {
 private:
  ds::graph::TimelineEncoding timeline_;
  base::SolverInfo &result_;
  bool timed_out_;

 public:
  LocalSearch(ds::graph::Graph const &graph, base::SolverInfo &result, std::vector<std::pair<int, int>> const &contractions = {})
      : timeline_(graph, contractions.empty() ? result.contraction_sequence() : contractions), result_(result), timed_out_(false) {
    assert(result.has_upper_bound());
  }

  int upper_bound() const { return result_.upper_bound(); }

  std::vector<std::pair<int, int>> contraction_sequence() const { return result_.contraction_sequence(); }

  void run(util::Random &rand, int time_limit_sec = 0);

  bool search_parent();

  bool search_permutation(int v);

  bool search_permutation_full();

  // bool search_triangle_permutation_full();

  bool search_red_shift(util::Random &rand);

 private:
  bool search_red_shift_vertex_iteration(util::Random &rand);
  bool search_red_shift_iteration(util::Random &rand);
};
}  // namespace upperbound
}  // namespace algorithms
