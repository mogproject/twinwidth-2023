#pragma once

#include <algorithm>
#include <unordered_map>
#include <vector>

#include "algorithms/PrimeSolver.hpp"
#include "algorithms/base/TimeManager.hpp"
#include "ds/graph/Graph.hpp"
#include "modular/MDTree.hpp"
#include "util/Random.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

namespace algorithms {
class ExactSolver {
 private:
  ds::graph::Graph const& graph_;
  int lb_greedy_num_iterations_;
  int tww_;
  std::vector<std::pair<int, int>> contractions_;

 public:
  ExactSolver(ds::graph::Graph const& graph, int lb_greedy_num_iterations = 20)
      : graph_(graph), lb_greedy_num_iterations_(lb_greedy_num_iterations), tww_(0) {}

  int twin_width() const { return tww_; }

  std::vector<std::pair<int, int>> contraction_sequence() const { return contractions_; }

  void run(util::Random& rand, int lower_bound = 0, int upper_bound = -1, bool lb_mode = false, int time_limit_sec = 0) {
    log_info("ExactSolver started: n=%d, m=%d, lb=%d, ub=%d, time_limit=%ds", graph_.number_of_nodes(),
             graph_.number_of_edges(), lower_bound, upper_bound, time_limit_sec);
    base::TimeManager time_manager(time_limit_sec);

    //-----------------------------------------------------------------------
    //    I. Modular decomposition
    //-----------------------------------------------------------------------
    modular::MDTree mdtree(graph_, false);
    if (mdtree.empty()) return;

    //-----------------------------------------------------------------------
    //    II. Collect information of internal nodes
    //-----------------------------------------------------------------------
    auto const& t = mdtree.get_tree();
    std::vector<int> nodes;

    for (auto i : t.dfs_reverse_preorder_nodes(mdtree.get_root())) {
      if (!t[i].is_leaf()) nodes.push_back(i);
    }

    // Traverse from the bottom of the modular decomposition tree.
    std::reverse(nodes.begin(), nodes.end());

    std::unordered_map<int, std::vector<int>> leaders;
    std::unordered_map<int, std::vector<std::pair<int, int>>> partial_results;
    std::vector<std::pair<int, int>> prime_stats;

    for (auto node : nodes) {
      // Given a module, find the leader for each child module.
      // Assuming that vertex v gets merged into vertex u only if u's label is larger than v's,
      // every leader must have the maximum label among its module.
      std::vector<int> vs;
      for (auto child : t.get_children(node)) {
        int leader = 0;
        for (int j = t[child].data.vertices_begin; j < t[child].data.vertices_end; ++j) {
          leader = std::max(leader, mdtree.get_vertex(j));
        }
        vs.push_back(leader);
      }
      leaders[node] = vs;

      if (t[node].data.is_prime_node()) {
        // prime -> store information and solve later
        prime_stats.push_back({vs.size(), node});
      } else {
        // non-prime -> immediately create results
        if (!lb_mode) {
          for (int i = 1; i < static_cast<int>(vs.size()); ++i) {
            // any contraction is safe
            partial_results[node].push_back({0, i});
          }
        }
      }
    }

    //-----------------------------------------------------------------------
    //    III. Sort prime graphs
    //-----------------------------------------------------------------------

    // Sort prime graphs by size (largest to smallest)
    std::sort(prime_stats.rbegin(), prime_stats.rend());
    int num_prime_graphs = prime_stats.size();

#if LOGGING_ON
    for (int i = 0; i < num_prime_graphs; ++i) {
      log_info("ExactSolver found prime [%d]: n=%d", i, prime_stats[i].first);
    }
#endif

    //-----------------------------------------------------------------------
    //    IV. Process prime graphs
    //-----------------------------------------------------------------------

    // Now, process each prime graph
    for (int graph_id = 0; graph_id < num_prime_graphs; ++graph_id) {
      int node_id = prime_stats[graph_id].second;
      auto const& vs = leaders[node_id];

      lower_bound = std::max(lower_bound, 1);  // prime => tww >= 1
      auto H = graph_.induce_and_relabel(vs);
      assert(H.number_of_nodes() == prime_stats[graph_id].first);

      // if density > 0.5, take the complement
      if (H.number_of_edges() * 4 > H.number_of_nodes() * (H.number_of_nodes() - 1)) {
        H = H.complement();
        log_debug("working on the complement: n=%d, m=%d", H.number_of_nodes(), H.number_of_edges());
      }

      // compute an optimal contraction sequence
      base::SolverInfo result(graph_id, lower_bound, upper_bound);
      PrimeSolver solver(H, result, lb_mode, lb_greedy_num_iterations_, time_manager);
      solver.run(rand);  // run with the lower-bound so far

      // store the result
      partial_results[node_id] = result.contraction_sequence();

      // update the lower-bound
      lower_bound = std::max(lower_bound, result.lower_bound());
    }
    tww_ = lower_bound;

    //-----------------------------------------------------------------------
    //    V. Construct contraction sequence
    //-----------------------------------------------------------------------

    // Lastly, compose the contraction sequence from the bottom
    if (lb_mode) return;

    contractions_.clear();
    for (auto node : nodes) {
      assert(partial_results[node].size() == leaders[node].size() - 1);

      // convert vertex labels
      std::vector<std::pair<int, int>> seq;
      for (auto& p : partial_results[node]) seq.push_back({leaders[node][p.first], leaders[node][p.second]});

      // normalize the contraction sequence so that the maximum label always remains
      for (auto& p : ds::graph::normalize_contraction_sequence(seq)) contractions_.push_back(p);
    }
  }
};
}  // namespace algorithms
