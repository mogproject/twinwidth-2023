#pragma once

#include <cassert>

#include "algorithms/base/SolverInfo.hpp"
#include "algorithms/base/TimeManager.hpp"
#include "algorithms/exact/BranchSolver.hpp"
#include "algorithms/exact/DirectSolver.hpp"
#include "algorithms/exact/PrimeTreeSolver.hpp"
#include "algorithms/lowerbound/LBGreedy.hpp"
#include "algorithms/lowerbound/LBManager.hpp"
#include "algorithms/lowerbound/LBSample.hpp"
#include "algorithms/lowerbound/LBSeparate.hpp"
#include "algorithms/lowerbound/LBCore.hpp"
#include "algorithms/upperbound/GreedySolver.hpp"
#include "algorithms/upperbound/LocalSearch.hpp"
#include "algorithms/upperbound/UBSeparate.hpp"
#include "ds/graph/Graph.hpp"
#include "ds/graph/TriGraph.hpp"
#include "sat/SATSolver.hpp"
#include "util/Random.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

namespace algorithms {
class PrimeSolver {
 private:
  ds::graph::Graph graph_;
  base::SolverInfo& result_;
  bool lb_mode_;
  int lb_greedy_num_iterations_;
  base::TimeManager time_manager_;

 public:
  PrimeSolver(ds::graph::Graph const& graph, base::SolverInfo& solver_info, bool lb_mode, int lb_greedy_num_iterations,
              base::TimeManager time_manager = base::TimeManager())
      : graph_(graph),
        result_(solver_info),
        lb_mode_(lb_mode),
        lb_greedy_num_iterations_(lb_greedy_num_iterations),
        time_manager_(time_manager) {
    assert(graph.number_of_nodes() >= 4);
  }

  void run(util::Random& rand) {
    //
    log_info("%s PrimeSolver started: n=%d, m=%d", result_.to_string().c_str(), graph_.number_of_nodes(), graph_.number_of_edges());

    //==========================================================================
    // (0) Exact: Trees (solvable in linear time)
    //==========================================================================

    if (graph_.number_of_edges() == graph_.number_of_nodes() - 1) {
      // prime graph => non-empty and connected
      // empty or (connected and n-1 edges) => tree
      auto tree_solver = exact::PrimeTreeSolver(graph_);
      tree_solver.run();

      result_.update_exact(tree_solver.twin_width(), tree_solver.contraction_sequence());
      return;
    }

    // util::print(graph_.edges());

    //==========================================================================
    // (1) Lowerbound: LBGreedy
    //==========================================================================

    graph_.compute_all_pairs_symmetric_differences();
    auto lb_greedy = lowerbound::LBGreedy(graph_, result_);
    lb_greedy.run(rand, lb_greedy_num_iterations_);

    // start heuristics
    RUN_WITH_TRIGRAPH(graph_, run_main, rand);
  }

  template <typename T>
  void run_main(T trigraph, util::Random& rand) {
    if (time_manager_.is_time_over()) return;

    //==========================================================================
    // (2) Quick Branch
    //==========================================================================

    int branch_time_limit = time_manager_.adjust_time_limit(5);
    long long branch_counter = 100000L;
    exact::BranchSolver<T> lb_branch(trigraph, result_);
    lb_branch.run(branch_time_limit, branch_counter);
    if (result_.resolved()) return;
    if (time_manager_.is_time_over()) return;

    //==========================================================================
    // (3) Quick Greedy
    //==========================================================================

    auto greedy_ub = (result_.has_upper_bound() ? result_.upper_bound() : trigraph.number_of_nodes()) - 1;
    trigraph.compute_greedy_criteria(greedy_ub);
    int greedy_time_limit = time_manager_.adjust_time_limit(2);
    int greedy_num_iterations = 1000;
    int volatility_rate = 500;
    int greedy_permissiveness = 0;

    upperbound::GreedySolver<T> gsolver(trigraph, result_);
    gsolver.run(rand, greedy_num_iterations, greedy_time_limit, volatility_rate, greedy_permissiveness);
    if (result_.resolved()) return;
    if (time_manager_.is_time_over()) return;

    //==========================================================================
    // (4) Quick Sample
    //==========================================================================

    if (!lb_mode_) {
      int sample_time_limit_light = 20;
      std::vector<lowerbound::sample::SampleOptions> options_light = {
          lowerbound::sample::SampleOptions({lowerbound::sample::SAMPLE_DEGREE, 16, 20, 10, 6}),
          lowerbound::sample::SampleOptions({lowerbound::sample::SAMPLE_DEGREE, 20, 20, 10, 7}),
      };

      lowerbound::LBSample lb_sample_light(graph_, result_);
      lb_sample_light.run(rand, options_light, sample_time_limit_light);
      if (result_.resolved()) return;
    }

    //==========================================================================
    // (5) Quick LBSeparate + UBSeparate
    //==========================================================================

    lowerbound::LBManager<T> lb_manager(trigraph, result_);
    preprocess::VertexSeparator sep;
    auto aps = sep.find_articulation_points(graph_);

    upperbound::UBSeparate<T> ub_separate(trigraph, result_, aps);
    ub_separate.run(10, rand, 0);
    if (result_.resolved()) return;

    lowerbound::LBSeparate<T> lb_separate(lb_manager, aps);
    lb_separate.run(20, 0, rand);  // level 0
    if (result_.resolved()) return;

    //==========================================================================
    // (6) Quick Greedy
    //==========================================================================

    greedy_time_limit = time_manager_.adjust_time_limit(3);
    greedy_num_iterations = 100000;
    volatility_rate = 10000;
    greedy_permissiveness = 0;

    gsolver.run(rand, greedy_num_iterations, greedy_time_limit, volatility_rate, greedy_permissiveness);
    if (result_.resolved()) return;
    if (time_manager_.is_time_over()) return;

    //==========================================================================
    // (7) LBSeparate Level 1
    //==========================================================================

    lb_separate.run(20, 1, rand);

    //==========================================================================
    // (8) UBSeparate + Greedy
    //==========================================================================

    ub_separate.run(20, rand, 0);
    if (result_.resolved()) return;
    gsolver.run(rand, greedy_num_iterations, 30, volatility_rate, 0);
    if (result_.resolved()) return;
    gsolver.run(rand, greedy_num_iterations, 10, volatility_rate, 1);
    if (result_.resolved()) return;

    //==========================================================================
    // (9) LBSeparate Level 2 + UBSeparate
    //==========================================================================

    while (!result_.resolved()) {
      lb_separate.run(50, 2, rand);
      if (result_.resolved()) break;
      ub_separate.run(50, rand, 0);
      if (result_.resolved()) break;

      volatility_rate = std::max(100, volatility_rate / 2);
      gsolver.run(rand, greedy_num_iterations, 30, volatility_rate, 0);
    }
  }
};
}  // namespace algorithms
