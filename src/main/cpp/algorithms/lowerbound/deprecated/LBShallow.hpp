#pragma once

#include "ds/graph/Graph.hpp"
#include "sat/SATSolver.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

namespace algorithms {
namespace lowerbound {

class LBShallow {
 private:
  ds::graph::Graph const& graph_;
  int n_;
  sat::SATSolver solver_;
  std::vector<std::pair<int, int>> partial_contraction_;

 public:
  LBShallow(ds::graph::Graph const& graph) : graph_(graph), n_(graph.number_of_nodes()) {}

  std::vector<std::pair<int, int>> const& get_partial_contraction() const { return partial_contraction_; }

  int run(int min_depth = 5, int max_depth = -1, int lower_bound = 0, int upper_bound = -1, int time_limit_sec = 0) {
    log_info("LBShallow started: n=%d, m=%d, min_depth=%d, max_depth=%d, lb=%d, ub=%d, time_limit=%ds",
             graph_.number_of_nodes(), graph_.number_of_edges(), min_depth, max_depth, lower_bound, upper_bound, time_limit_sec);
    util::timer_start();

    bool timed_out = false;
    int d = lower_bound;
    for (int depth = std::min(min_depth, n_ - 1);
         !timed_out && (upper_bound < 0 || d < upper_bound) && depth <= n_ - 1 && (max_depth < 0 || depth <= max_depth); ++depth) {
      while (upper_bound < 0 || d < upper_bound) {
        log_trace("LBShallow checking d=%d, depth=%d", d, depth);

        auto result = find_shallow_lb(depth, d, time_limit_sec);
        if (result == sat::status::SATISFIABLE) {
          log_trace("LBShallow feasible: saved partial contraction d=%d, len=%d", d,
                    static_cast<int>(partial_contraction_.size()));
          break;
        } else if (result == sat::status::INCONSISTENT || result == sat::status::INCONSISTENT_AND_CORE_COMPUTED) {
          ++d;
          log_warning("LBShallow New LB: lb=%d", d);
        } else {
          log_error("LBShallow timed out: d=%d, depth=%d", d, depth);
          timed_out = true;
          break;
        }
      }
    }

    log_info("LBShallow %slb=%d, runtime=%.2fs", d == lower_bound ? "(no update) " : "", d, util::timer_stop());
    return d;
  }

 private:
  // SAT variables
  int v(int i, int k) { return solver_.id(1000000000L + i * n_ + k); }
  int u(int i, int k) { return solver_.id(2000000000L + i * n_ + k); }
  int p(int i, int k) { return solver_.id(3000000000L + i * n_ + k); }
  int rho(int i, int t) { return solver_.id(4000000000L + i * n_ + t); }
  int r(int i, int j, int k) { return solver_.id(5000000000L + (std::min(i, j) * n_ + std::max(i, j)) * n_ + k); }

  void encode_v(int max_depth) {
    for (int i = 0; i < n_; ++i) {
      // -u(i,0) => v(i,0)
      solver_.add_clause({u(i, 0), v(i, 0)});

      for (int t = 0; t < max_depth; ++t) {
        // v(i,t) => -u(i,t)
        solver_.add_clause({-v(i, t), -u(i, t)});
        // p(i,t) => v(i,t)
        solver_.add_clause({-p(i, t), v(i, t)});

        if (t > 0) {
          // u(i,t) => v(i,t-1)
          solver_.add_clause({v(i, t - 1), -u(i, t)});
          // v(i,t-1) <= v(i,t)
          solver_.add_clause({v(i, t - 1), -v(i, t)});
          // v(i,t-1) and -u(i,t) => v(i,t)
          solver_.add_clause({-v(i, t - 1), u(i, t), v(i, t)});
        }
      }
    }
  }

  void encode_u(int max_depth) {
    for (int k = 0; k < max_depth; ++k) {
      std::vector<int> us;
      for (int i = 0; i < n_; ++i) us.push_back(u(i, k));
      solver_.add_equals_one(us);
    }
  }

  void encode_p(int max_depth, int d) {
    // one true for each k
    for (int k = 0; k < max_depth; ++k) {
      std::vector<int> ps;
      for (int i = 0; i < n_; ++i) ps.push_back(p(i, k));
      solver_.add_equals_one(ps);
    }

    // contraction hinting
    for (int i = 0; i < n_ - 1; ++i) {
      for (int j = i + 1; j < n_; ++j) {
        int diff = std::min(max_depth, graph_.get_symmetric_difference_size(i, j) - d);
        for (int t = 0; t < diff; ++t) {
          // i, j cannot be a contraction pair at time t
          solver_.add_clause({-u(i, t), -p(j, t)});
          solver_.add_clause({-u(j, t), -p(i, t)});
        }
      }
    }
  }

  void encode_rho(int max_depth) {
    for (int k = 0; k < max_depth; ++k) {
      for (int i = 0; i < n_ - 1; ++i) {
        for (int j = i + 1; j < n_; ++j) {
          solver_.add_clause({-u(j, k), -r(i, j, k - 1), rho(i, k)});
          solver_.add_clause({-u(i, k), -r(i, j, k - 1), rho(j, k)});
        }
      }
    }
  }

  void encode_r(int max_depth, int d, bool use_rho = true) {
    if (use_rho) encode_rho(max_depth);

    for (int i = 0; i < n_ - 1; ++i) {
      for (int j = i + 1; j < n_; ++j) {
        // Introduced red edges
        if (graph_.get_symmetric_difference_size(i, j) <= max_depth + d) {
          for (auto w : graph_.get_symmetric_difference(i, j)) {
            for (int k = 0; k < max_depth; ++k) {
              // u(i,k) and p(j,k) and v(w,k) => r(j,w,k); i and j symmetric
              solver_.add_clause({-u(i, k), -p(j, k), -v(w, k), r(j, w, k)});
              solver_.add_clause({-u(j, k), -p(i, k), -v(w, k), r(i, w, k)});
            }
          }
        }

        for (int k = 1; k < max_depth; ++k) {
          // Continuous red edges.
          // r(i,j,k-1) and -u(i,k) and -u(j,k) => r(i,j,k)
          solver_.add_clause({-r(i, j, k - 1), u(i, k), u(j, k), r(i, j, k)});

          // Transferred red edges
          if (use_rho) {
            // rho(k,i) and p(j,k) => r(i,j,k); i and j symmetric
            solver_.add_clause({-rho(i, k), -p(j, k), r(i, j, k)});
            solver_.add_clause({-rho(j, k), -p(i, k), r(i, j, k)});
          } else {
            for (int w = 0; w < n_; ++w) {
              if (i == w || j == w) continue;

              // r(i,w,k-1) and u(i,k) and p(j,k) => r(j,w,k); i and j symmetric
              solver_.add_clause({-r(i, w, k - 1), -u(i, k), -p(j, k), r(j, w, k)});
              solver_.add_clause({-r(j, w, k - 1), -u(j, k), -p(i, k), r(i, w, k)});
            }
          }
        }
      }
    }
  }

  void encode_counter(int max_depth, int d) {
    // cardinality constraints
    for (int k = 0; k < max_depth; ++k) {
      for (int i = 0; i < n_; ++i) {
        // red degree unti time K
        std::vector<int> lits;
        for (int w = 0; w < n_; ++w) {
          if (i != w) lits.push_back(r(i, w, k));
        }
        solver_.add_atmost(lits, d);
      }
    }
  }

 public:
  int find_shallow_lb(int max_depth, int d, int time_limit_sec = 0) {
    assert(0 < max_depth && max_depth <= n_ - 1);

    // trivial case
    if (d >= n_ - 2) return sat::status::SATISFIABLE;

    // construct clauses
    solver_.restart();
    encode_v(max_depth);
    encode_u(max_depth);
    encode_p(max_depth, d);
    encode_r(max_depth, d, true);
    encode_counter(max_depth, d);

    // solve
    int ret = solver_.solve(time_limit_sec);
    if (ret == sat::status::SATISFIABLE) {
      // decode partial contraction
      partial_contraction_.clear();
      for (int t = 0; t < max_depth; ++t) {
        int u_val = -1, p_val = -1;
        for (int i = 0; i < n_ && (u_val < 0 || p_val < 0); ++i) {
          if (solver_.get_witness(u(i, t))) u_val = i;
          if (solver_.get_witness(p(i, t))) p_val = i;
        }
        assert(u_val >= 0 && p_val >= 0);
        partial_contraction_.push_back({p_val, u_val});
      }
    }
    return ret;
  }
};
}  // namespace lowerbound
}  // namespace algorithms
