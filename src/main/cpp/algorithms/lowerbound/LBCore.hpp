#pragma once
#include <chrono>

#include "algorithms/base/SolverInfo.hpp"
#include "ds/graph/Graph.hpp"
#include "sat/SATSolver.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

namespace algorithms {
namespace lowerbound {

class LBCore {
 private:
  ds::graph::Graph const& graph_;
  base::SolverInfo& result_;

 public:
  LBCore(ds::graph::Graph const& graph, base::SolverInfo& result) : graph_(graph), result_(result) {}

  // true: found the tight bound
  bool run(int time_limit_sec = 0, bool find_smallest_core = false) {
    log_info("%s LBCore started: n=%d, time_limit=%ds, small_core=%s", result_.to_string().c_str(), graph_.number_of_nodes(), time_limit_sec,
             find_smallest_core ? "True" : "False");
    util::timer_start();
    bool timeout = false;

    // (1) Find lower-bound (ascending order)
    while (!result_.resolved()) {
      int d = result_.lower_bound() + 1;
      log_trace("%s LBCore checking: d=%d", result_.to_string().c_str(), d);

      auto result = find_core_lb(d, time_limit_sec);
      if (result == sat::status::SATISFIABLE) {
        // continue
      } else if (result == sat::status::INCONSISTENT || result == sat::status::INCONSISTENT_AND_CORE_COMPUTED) {
        break;
      } else {
        timeout = true;
        break;
      }
    }

    // (2) (Optional) Find smallest core (binary search)
    // if (!timeout && find_smallest_core) {
    //   int sz_lo = 4;  // largest infeasible core size (core must have at least 4 vertices if the graph is prime)
    //   int sz_hi = result_.has_upper_bound() ? result_.core_size() : -1;
    //   int sz = sz_hi < 0 ? sz_lo * 2 : (sz_lo + sz_hi) / 2;

    //   while (sz_hi < 0 || sz_lo + 1 < sz_hi) {
    //     log_trace("LBCore checking: d=%d, sz=%d", result_.lower_bound(), sz);

    //     auto result = find_core_lb(result_.lower_bound(), time_limit_sec, sz);
    //     if (result == sat::status::SATISFIABLE) {
    //       // found LB
    //       sz_hi = sz;
    //     } else if (result == sat::status::INCONSISTENT || result == sat::status::INCONSISTENT_AND_CORE_COMPUTED) {
    //       // infeasible
    //       sz_lo = sz;
    //     } else {
    //       timeout = true;
    //       break;
    //     }
    //     sz = sz_hi < 0 ? sz * 2 : (sz_lo + sz_hi) / 2;
    //   }
    // }

    if (timeout) {
      log_error("%s LBCore timed out: runtime=%.2fs", result_.to_string().c_str(), util::timer_stop());
    } else {
      log_info("%s LBCore finished: lb=%d, runtime=%.2fs", result_.to_string().c_str(), result_.lower_bound(), util::timer_stop());
    }
    return !timeout;
  }

 private:
  inline int x(int i) const { return i + 1; }

  int find_core_lb(int d, int time_limit_sec, int size_ubound = -1) {
    int n = graph_.number_of_nodes();
    std::vector<int> xs;
    for (int i = 0; i < n; ++i) xs.push_back(x(i));

    sat::SATSolver solver;

    // construct clauses
    solver.add_atleast(xs, 4, n);  // requires 4 vertices
    if (size_ubound >= 0) solver.add_atmost(xs, size_ubound, n);

    for (int i = 0; i < n - 1; ++i) {
      for (int j = i + 1; j < n; ++j) {
        if (graph_.get_symmetric_difference_size(i, j) < d) {
          solver.add_clause({-x(i), -x(j)});
        } else {
          std::vector<int> ks;
          for (auto k : graph_.get_symmetric_difference(i, j)) ks.push_back(x(k));
          solver.add_atleast(ks, d, n, {-x(i), -x(j)});
        }
      }
    }

    // solve
    int ret = solver.solve(time_limit_sec);
    if (ret == sat::status::SATISFIABLE) {
      // decode core information
      std::vector<int> core;
      for (int i = 1; i <= n; ++i) {
        if (solver.get_witness(i)) core.push_back(i - 1);
      }

      // int prev_core_size = result_.core_size();
      // int core_size = static_cast<int>(core.size());
      // if (result_.update_lower_bound(d, core)) {
      if (result_.update_lower_bound(d)) {
        // log_warning("%s LBCore found new LB: lb=%d, core_size=%d", result_.to_string().c_str(), d, core_size);
        log_warning("%s LBCore found new LB: lb=%d", result_.to_string().c_str(), d);
      }
      //  else if (d == result_.lower_bound() && core_size < prev_core_size) {
      //   log_trace("%s LBCore found smaller core: lb=%d, core_size=%d", result_.to_string().c_str(), d, core_size);
      // }
    }
    return ret;
  }
};

}  // namespace lowerbound
}  // namespace algorithms
