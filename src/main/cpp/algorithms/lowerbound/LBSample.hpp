#pragma once

#include "algorithms/base/SolverInfo.hpp"
#include "algorithms/exact/DirectSolver.hpp"
#include "ds/graph/Graph.hpp"
#include "ds/set/FastSet.hpp"
#include "util/Random.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

namespace algorithms {
namespace lowerbound {

namespace sample {
enum SampleStrategy { SAMPLE_UNIFORM, SAMPLE_DEGREE, SAMPLE_PEEL };

struct SampleOptions {
  int strategy = SAMPLE_DEGREE;
  int max_sample_size = 50;
  int num_iterations = 10;           // number of iterations
  int time_limit_per_iteration = 0;  // time limit in seconds for each iteration
  int lb_required = 100;
};

}  // namespace sample

class LBSample {
  ds::graph::Graph const& original_graph_;
  base::SolverInfo& result_;
  std::vector<ds::graph::Graph> processed_graphs_;

  ds::FastSet visited_;

 public:
  LBSample(ds::graph::Graph const& graph, base::SolverInfo& result)
      : original_graph_(graph), result_(result), processed_graphs_({graph}), visited_(graph.number_of_nodes()) {}

  void preprocess(bool remove_leaves = true, bool smooth = true, int separator_size = 3);

  int run(util::Random& rand, std::vector<sample::SampleOptions> const& options, int time_limit_sec = 0);

  std::vector<int> sample_naive(ds::graph::Graph const& graph, util::Random& rand, int sample_size);
  std::vector<int> sample_degree_weighted(ds::graph::Graph const& graph, util::Random& rand, int sample_size);
  std::vector<int> sample_peel(ds::graph::Graph const& graph, util::Random& rand, int sample_size, int lb);
};
}  // namespace lowerbound
}  // namespace algorithms
