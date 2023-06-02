#pragma once

#include "algorithms/exact/BranchSolver.hpp"
#include "algorithms/exact/DirectSolver.hpp"
#include "algorithms/lowerbound/LBCore.hpp"
#include "algorithms/lowerbound/LBGreedy.hpp"
#include "algorithms/upperbound/GreedySolver.hpp"
#include "util/hash_table.hpp"
#include "ds/cache/SimpleCache.hpp"

namespace algorithms {
namespace lowerbound {
namespace manage {
struct CacheEntry {
  int level = -1;
  int upper_bound = -1;
};
}  // namespace manage

template <typename T>
class LBManager {
 private:
  T const& trigraph_;
  base::SolverInfo& result_;
  std::unordered_map<uint64_t, manage::CacheEntry> cache_table_;
  int prev_log_level_;
  uint64_t hash_all_;
  int lb_core_counter_ = 0;

 public:
  LBManager(T const& trigraph, base::SolverInfo& result) : trigraph_(trigraph), result_(result) {  //
    prev_log_level_ = util::logging::log_level;
    hash_all_ = get_hash(trigraph.nodes().to_vector());
  }

  T const& get_trigraph() const { return trigraph_; }

  ds::graph::Graph const& get_graph() const { return trigraph_.original_graph(); }

  base::SolverInfo const& result() const { return result_; }

  bool resolved() const { return result_.resolved(); }

  int get_sample_size(int level) {
    switch (level) {
      case 0: return 23;
      case 1: return 30;
      case 2: return 60;
      default: return 23;
    }
  }

  /**
   * @brief
   *
   * @param vertices
   * @param level
   * @param rand
   * @return true if the lower-bound is best possible
   */
  bool compute_lower_bound(std::vector<int> const& vertices, int level, util::Random& rand) {
    // printf("s compute_lower_bound: level=%d\n", level);
    // trivial cases
    int n = vertices.size();
    if (result_.lower_bound() >= 4 && n <= 10) return true;
    if (result_.lower_bound() >= 3 && n <= 8) return true;
    if (result_.lower_bound() >= 2 && n <= 7) return true;

    // fetch cache entry
    uint64_t hash = get_hash(vertices);
    auto cached_entry = cache_table_[hash];

    // resolved
    if (cached_entry.upper_bound >= 0 && result_.lower_bound() >= cached_entry.upper_bound) {
      // printf("f1compute_lower_bound: hash=%0llx\n", hash);
      return true;
    }

    // level already computed
    if (cached_entry.level >= level) {
      // printf("f2compute_lower_bound: hash=%0llx\n", hash);
      return false;
    }

    // compute induced subgraph with all-pairs symmetric differences
    // util::print(vertices);
    auto g = trigraph_.original_graph().induce_and_relabel(vertices, nullptr, true);
    T t(g);  // trigraph

    int ub = cached_entry.upper_bound >= 0 ? cached_entry.upper_bound : result_.upper_bound();
    base::SolverInfo info(-1, result_.lower_bound(), ub);
    t.compute_greedy_criteria(info.upper_bound());

    for (int lv = cached_entry.level + 1; lv <= level; ++lv) {
      if (lv == 0) {
        run_ub_greedy(t, info, rand, 10, 10, 6);
        if (info.resolved()) break;

        run_lb_greedy(g, info, rand, 10);
        if (info.resolved()) break;

        run_branch(t, info, 27, 1, 10000000LL);  // 10^7
        if (info.resolved()) break;
      } else if (lv == 1) {
        run_core(g, info, 40, hash == hash_all_);
        if (info.resolved()) break;

        run_branch(t, info, 36, 1, 10000000LL);  // 10^7
        if (info.resolved()) break;

        run_direct(g, info, 23, 120);
        if (info.resolved()) break;
      } else if (lv == 2) {
        run_branch(t, info, 123, 120, 1000000000LL);  // 10^9
        if (info.resolved()) break;

        run_direct(g, info, 50, 500);
        if (info.resolved()) break;
      }
    }

    cache_table_[hash] = manage::CacheEntry({level, info.upper_bound()});
    // printf("f0compute_lower_bound: hash=%0llx, level=%d, ub=%d\n", hash, level, info.upper_bound());
    return info.resolved();
  }

 private:
  void disable_logging() { util::set_log_level(util::logging::CRITICAL); }

  void enable_logging() { util::set_log_level(prev_log_level_); }

  //==========================================================================
  // (1) UBGreedy for early exit
  //==========================================================================
  void run_ub_greedy(T const& g, base::SolverInfo& info, util::Random& rand, int num_iterations, int time_limit_sec, int volatility_rate) {
    upperbound::GreedySolver<T> gsolver(g, info, {}, true);

    disable_logging();
    gsolver.run(rand, num_iterations, time_limit_sec, volatility_rate);
    enable_logging();
  }

  //==========================================================================
  // (2) LBGreedy
  //==========================================================================
  void run_lb_greedy(ds::graph::Graph const& g, base::SolverInfo& info, util::Random& rand, int num_iterations) {
    lowerbound::LBGreedy lb_greedy(g, info);

    disable_logging();
    lb_greedy.run(rand, num_iterations);
    enable_logging();

    if (result_.update_lower_bound(info.lower_bound())) {
      log_warning("%s LBM (LBGreedy) found new LB: lb=%d, n=%d", result_.to_string().c_str(), info.lower_bound(),
                  g.number_of_nodes());
    }
  }

  //==========================================================================
  // (3) BranchSolver
  //==========================================================================
  void run_branch(T const& g, base::SolverInfo& info, int max_n, int time_limit_sec, long long counter_limit) {
    int n = g.number_of_nodes();
    if (n <= result_.lower_bound() * 2) return;
    if (n > max_n) return;

    exact::BranchSolver<T> lb_branch(g, info);

    disable_logging();
    lb_branch.run(time_limit_sec, counter_limit);
    enable_logging();

    // update upper bound if solver finds an exact solution
    info.update_upper_bound(info.upper_bound(), {});

    if (result_.update_lower_bound(info.lower_bound())) {
      log_warning("%s LBM (Branch) found new LB: lb=%d, n=%d", result_.to_string().c_str(), info.lower_bound(), n);
    }
    if (lb_branch.timed_out()) {
      log_trace("%s LBM (Branch) timed out: n=%d, time_limit=%ds", result_.to_string().c_str(), n, time_limit_sec);
    }
  }

  //==========================================================================
  // (4) DirectSolver
  //==========================================================================
  void run_direct(ds::graph::Graph const& g, base::SolverInfo& info, int max_n, int time_limit_sec) {
    int n = g.number_of_nodes();
    if (n <= result_.lower_bound() * 2) return;
    if (n > max_n) return;

    exact::DirectSolver d_solver(g);

    disable_logging();
    auto d_solver_ret = d_solver.run(result_.lower_bound(), info.upper_bound(), time_limit_sec);
    enable_logging();

    // update upper bound if solver finds an exact solution
    info.update_upper_bound(d_solver.upper_bound(), {});
    info.update_lower_bound(d_solver.lower_bound());

    if (!d_solver_ret) {
      log_trace("%s LBM (Direct) timed out: n=%d, time_limit=%ds", result_.to_string().c_str(), n, time_limit_sec);
    }
    if (result_.update_lower_bound(info.lower_bound())) {
      log_warning("%s LBM (Direct) found new LB: lb=%d, n=%d", result_.to_string().c_str(), d_solver.lower_bound(), n);
    }
  }

  //==========================================================================
  // (5) LBCore
  //==========================================================================
  void run_core(ds::graph::Graph const& g, base::SolverInfo& info, int time_limit_sec, bool is_entire) {
    if (g.number_of_nodes() > 123 && lb_core_counter_ >= 2) return;
    ++lb_core_counter_;

    lowerbound::LBCore lb_core(g, info);
    disable_logging();
    if (lb_core.run(time_limit_sec, false)) {
      if (is_entire) { lb_core_counter_ = 10; }
    }
    enable_logging();
    result_.update_lower_bound(info.lower_bound());
  }

  uint64_t get_hash(std::vector<int> const& vertices) {
    uint64_t ret = 0;
    for (auto v : vertices) ret ^= util::get_hash(v);
    return ret;
  }
};
}  // namespace lowerbound
}  // namespace algorithms
