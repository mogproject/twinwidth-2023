#pragma once
#include <chrono>

#include "algorithms/base/SolverInfo.hpp"
#include "ds/graph/Graph.hpp"
#include "ds/queue/BucketQueue.hpp"
#include "ds/set/SortedVectorSet.hpp"
#include "util/Random.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

namespace algorithms {
namespace lowerbound {
/**
 * @brief Greedy algorithm for finding lower bounds of the twin-width.
 */
class LBGreedy {
 private:
  ds::graph::Graph const& graph_;
  base::SolverInfo& result_;
  int lb_;

 public:
  LBGreedy(ds::graph::Graph const& graph, base::SolverInfo& result) : graph_(graph), result_(result), lb_(0) {}

  /**
   * @brief
   *
   * Precondition: `graph.apsd_` (all-pairs symmetric differences) must be properly set
   *
   * @param rand
   * @param num_iterations
   * @return int
   */
  int run(util::Random& rand, int num_iterations = 20) {
    log_info("%s LBGreedy started: num_iterations=%d", result_.to_string().c_str(), num_iterations);
    util::timer_start();

    for (int t = 0; t < num_iterations; ++t) {
      run_iteration(rand, t);
      // auto lb = run_iteration(rand, t);
      // log_trace("LBGreedy iteration %3d: lb=%d, best=%d", t, lb, lb_);
    }

    log_info("%s LBGreedy finished: runtime=%.2fs", result_.to_string().c_str(), util::timer_stop());
    return lb_;
  }

 private:
  int run_iteration(util::Random& rand, int iteration_id) {
    int n = graph_.number_of_nodes();
    ds::FastSet fs(n);
    ds::SortedVectorSet vertices;
    std::vector<int> removed_vertices;
    int num_vertices = n;

    // initialize the set of vertices
    for (int i = 0; i < n; ++i) vertices.set(i);

    // initialize queue
    std::vector<int> initial_delta(n * n, n);  // initialized to n
    int delta_min = n;
    for (int i = 0; i < n - 1; ++i) {
      for (int j = i + 1; j < n; ++j) {
        int delta = graph_.get_symmetric_difference_size(i, j);
        initial_delta[i * n + j] = delta;
        delta_min = std::min(delta_min, delta);
      }
    }

    bool updated = false;
    // int core_size = result_.core_size();
    ds::queue::BucketQueue q(initial_delta);

    while (lb_ + 2 <= num_vertices) {
      int delta = q.min_value();

      // update global status
      if (delta > lb_) {
        // core_size = num_vertices;
        updated = true;
      }
      lb_ = std::max(lb_, delta);

      // randomly pick a vertex in the front bucket
      int x = q.get_random_key_with_min_value(rand);
      int v = rand.random() < 0.5 ? x / n : x % n;

      // remove vertex from the graph
      --num_vertices;
      vertices.reset(v);
      removed_vertices.push_back(v);

      // remove entries including v (v <-> V \ v)
      for (auto u : vertices.to_vector()) q.remove(std::min(u, v) * n + std::max(u, v));

      // decrease delta for N(v) <-> V \ N(v): vertex v was their unshared neighbor
      fs.clear();
      for (auto u : graph_.neighbors(v)) fs.set(u);
      for (auto u : graph_.neighbors(v)) {
        if (!vertices.get(u)) continue;
        for (auto w : vertices.to_vector()) {
          if (!fs.get(w)) q.update(std::min(u, w) * n + std::max(u, w), -1);
        }
      }
    }

    // recover core
    if (updated) {
      // std::vector<int> core = vertices.to_vector();
      // while (static_cast<int>(core.size()) < core_size) {
      //   core.push_back(removed_vertices.back());
      //   removed_vertices.pop_back();
      // }
      // if (result_.update_lower_bound(lb_, core)) {
      if (result_.update_lower_bound(lb_)) {
        // log_warning("%s LBGreedy found new LB: lb=%d, core_size=%d", result_.to_string().c_str(), lb_, core_size);
        log_warning("%s LBGreedy found new LB: lb=%d", result_.to_string().c_str(), lb_);
      }
    }

    return lb_;
  }
};
}  // namespace lowerbound
}  // namespace algorithms
