#pragma once
#include <cassert>

#include "ds/graph/Graph.hpp"
#include "ds/set/FastSet.hpp"
#include "sat/SATSolver.hpp"
#include "util/Random.hpp"
#include "util/logger.hpp"

namespace algorithms {
namespace exact {

class DirectSolver {
 private:
  ds::graph::Graph const& graph_;
  int n_;
  sat::SATSolver solver_;
  int lower_bound_;
  int upper_bound_;
  std::vector<std::pair<int, int>> contraction_;

 public:
  DirectSolver(ds::graph::Graph const& graph)
      : graph_(graph), n_(graph.number_of_nodes()), lower_bound_(0), upper_bound_(-1) {}

  int lower_bound() const { return lower_bound_; }

  int upper_bound() const { return upper_bound_; }

  int twin_width() const {
    assert(lower_bound_ == upper_bound_);
    return upper_bound_;
  }

  std::vector<std::pair<int, int>> const& contraction_sequence() { return contraction_; }

  bool run(int lower_bound = 0, int upper_bound = -1, int time_limit_sec = 0) {
    log_info("DirectSolver started: n=%d, m=%d, lb=%d, ub=%d, time_limit=%ds", graph_.number_of_nodes(),
             graph_.number_of_edges(), lower_bound, upper_bound, time_limit_sec);
    util::timer_start();
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;

    bool timed_out = false;
    for (int d = lower_bound_; !timed_out && (upper_bound_ < 0 || d < upper_bound_); ++d) {
      log_trace("DirectSolver checking d=%d", d);

      auto result = solve(d, time_limit_sec);
      if (result == sat::status::SATISFIABLE) {
        break;
      } else if (result == sat::status::INCONSISTENT || result == sat::status::INCONSISTENT_AND_CORE_COMPUTED) {
        lower_bound_ = d + 1;
        log_trace("DirectSolver found new LB: lb=%d", lower_bound_);
      } else {
        timed_out = true;
        break;
      }
    }

    if (timed_out) {
      log_error("DirectSolver timed out: lb=%d, ub=%d, runtime=%.2fs", lower_bound_, upper_bound_, util::timer_stop());
    } else {
      log_info("DirectSolver found exact solution: d=%d, runtime=%.2fs", upper_bound_, util::timer_stop());
    }
    return !timed_out;
  }

 private:
  int solve(int d, int time_limit_sec = 0) {
    // construct clauses
    solver_.restart();
    encode_o();
    encode_p();
    encode_a();
    encode_r(d);
    encode_counter(d);

    // solve
    int ret = solver_.solve(time_limit_sec);
    if (ret == sat::status::SATISFIABLE) {
      upper_bound_ = d;

      // decode solution
      auto xs = decode_o();

      contraction_.clear();
      for (int i = 0; i < n_ - 1; ++i) {
        int x = xs[i];
        for (int j = x + 1; j < n_; ++j) {
          if (solver_.get_witness(p(x, j))) {
            contraction_.push_back({j, x});  // `x` gets merged into `j`
            break;
          }
        }
      }
    }
    return ret;
  }

  // SAT variables
  int o(int i, int j) {
    assert(i != j);
    return solver_.id(1000000000L + std::min(i, j) * n_ + std::max(i, j)) * (i < j ? 1 : -1);
  }
  int p(int i, int j) {
    assert(i < j);
    return solver_.id(2000000000L + i * n_ + j);
  }
  int r(int k, int i, int j) {
    assert(k != i && k != j && i != j);
    return solver_.id(3000000000L + (k * n_ + std::min(i, j)) * n_ + std::max(i, j));
  }
  int a(int i, int j) {
    assert(i != j);
    return solver_.id(4000000000L + std::min(i, j) * n_ + std::max(i, j));
  }

  // encoders
  void encode_o() {
    // (1) Transitivity: o(i,j) & o(j,k) => o(i,k)
    for (int i = 0; i < n_; ++i) {
      for (int j = 0; j < n_; ++j) {
        if (i == j) continue;
        for (int k = 0; k < n_; ++k) {
          if (i == k || j == k) continue;
          solver_.add_clause({-o(i, j), -o(j, k), o(i, k)});
        }
      }
    }
  }

  void encode_p() {
    // (2a + 2b) at least and at most one parent except the root
    for (int i = 0; i < n_ - 1; ++i) {
      std::vector<int> lits;
      for (int j = i + 1; j < n_; ++j) lits.push_back(p(i, j));
      solver_.add_equals_one(lits);
    }

    // (2c) ordering condition: p(i,j) => o(i,j)
    for (int i = 0; i < n_ - 1; ++i) {
      for (int j = i + 1; j < n_; ++j) solver_.add_clause({-p(i, j), o(i, j)});
    }
  }

  void encode_a() {
    // (3a) semantics of a
    for (int i = 0; i < n_ - 1; ++i) {
      for (int j = i + 1; j < n_; ++j) {
        for (int k = 0; k < n_; ++k) {
          if (i == k || j == k) continue;
          solver_.add_clause({-o(k, i), -o(k, j), -r(k, i, j), a(i, j)});
        }
      }
    }
  }

  void encode_r(int d) {
    int const max_diff = 3;

    // (3b) semantics of red edges
    for (int i = 0; i < n_ - 1; ++i) {
      for (int j = i + 1; j < n_; ++j) {
        for (auto k : graph_.get_symmetric_difference(i, j)) solver_.add_clause({-p(i, j), -o(i, k), r(i, j, k)});

        // extra hints
        for (int diff = 1; diff <= max_diff; ++diff) {
          if (graph_.get_symmetric_difference_size(i, j) >= d + diff) {
            // (j,i) cannot be int the first `diff` contraction pair
            std::vector<int> lits;
            for (int k = 0; k < n_; ++k) {
              if (i != k && j != k) lits.push_back(o(k, i));
            }
            solver_.add_atleast(lits, diff, -1, {-p(i, j)});
          }
        }
      }
    }

    // (3c) transfer red edges
    for (int i = 0; i < n_ - 1; ++i) {
      for (int j = i + 1; j < n_; ++j) {
        for (int k = 0; k < n_; ++k) {
          if (i == k || j == k) continue;
          solver_.add_clause({-p(i, j), -o(i, k), -a(i, k), r(i, j, k)});
        }
      }
    }

    // (3d) maintain red edges
    for (int i = 0; i < n_; ++i) {
      for (int j = 0; j < n_; ++j) {
        if (i == j) continue;
        for (int k = 0; k < n_ - 1; ++k) {
          if (i == k || j == k) continue;
          for (int m = k + 1; m < n_; ++m) {
            if (i == m || j == m) continue;
            solver_.add_clause({-o(i, j), -o(j, k), -o(j, m), -r(i, k, m), r(j, k, m)});
          }
        }
      }
    }
  }

  void encode_counter(int ubound) {
    for (int i = 0; i < n_ - 1; ++i) {
      for (int x = 0; x < n_; ++x) {
        if (i == x) continue;
        std::vector<int> lits;
        for (int y = 0; y < n_; ++y) {
          if (x != y && i != y) lits.push_back(r(i, x, y));
        }
        solver_.add_atmost(lits, ubound);
      }
    }
  }

  // decoder
  std::vector<int> decode_o() {
    std::vector<int> xs;
    for (int i = 0; i < n_; ++i) xs.push_back(i);
    auto compare = [&](int i, int j) { return solver_.get_witness(o(i, j)); };
    std::sort(xs.begin(), xs.end(), compare);
    return xs;
  }
};
}  // namespace exact
}  // namespace algorithms
