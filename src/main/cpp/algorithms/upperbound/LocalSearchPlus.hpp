#pragma once

#include "algorithms/base/SolverInfo.hpp"
#include "ds/graph/TimelineEncodingPlus.hpp"
#include "util/Random.hpp"
#include "util/logger.hpp"

namespace algorithms {
namespace upperbound {

//==============================================================================
//  Signal Handling
//==============================================================================
namespace localsearchplus {

constexpr int const ALGORITHM_ID = 41;
extern bool volatile solver_ignore_alarm;
extern bool volatile solver_terminate_flag;

void solver_alarm_handler(int sig);
void set_timeout(int time_limit_sec);
void reset_timeout();
}  // namespace localsearchplus

//==============================================================================
//  Main Class
//==============================================================================

template <typename T>
class LocalSearchPlus {
 private:
  ds::graph::TimelineEncodingPlus<T> timeline_;
  base::SolverInfo &result_;
  bool timed_out_;

 public:
  LocalSearchPlus(T const &trigraph, base::SolverInfo &result, std::vector<int> frozen_vertices,
                  std::vector<std::pair<int, int>> const &contractions = {})
      : timeline_(trigraph, frozen_vertices, contractions.empty() ? result.contraction_sequence() : contractions),
        result_(result),
        timed_out_(false) {
    assert(result.has_upper_bound());
  }

  int upper_bound() const { return result_.upper_bound(); }

  std::vector<std::pair<int, int>> contraction_sequence() const { return result_.contraction_sequence(); }

  void run(util::Random &rand, int time_limit_sec = 0) {
    util::timer_start(localsearchplus::ALGORITHM_ID);
    log_info("%s LocalSearchPlus started: time_limit=%ds", result_.to_string().c_str(), time_limit_sec);
    timed_out_ = false;

    if (time_limit_sec > 0) localsearchplus::set_timeout(time_limit_sec);

    search_parent();

    if (time_limit_sec > 0) localsearchplus::reset_timeout();
    if (timed_out_) {
      log_error("%s LocalSearchPlus timed out: runtime=%.2fs", result_.to_string().c_str(),
                util::timer_stop(localsearchplus::ALGORITHM_ID));
    } else {
      log_info("%s LocalSearchPlus finished: runtime=%.2fs", result_.to_string().c_str(), util::timer_stop(localsearchplus::ALGORITHM_ID));
    }
  }

  bool search_parent() {
    // log_info("LocalSearch: search_parent() started: ub=%d", upper_bound_);
    int na = timeline_.number_of_active_nodes();
    int nu = timeline_.number_of_unfrozen_nodes();

    bool updated = true, ret = false;
    while (!timed_out_ && updated) {
      if (localsearchplus::solver_terminate_flag) {
        timed_out_ = true;
        break;
      }

      updated = false;
      for (int i = 0; i < nu - 1; ++i) {
        if (na - i - 1 <= result_.upper_bound()) break;  // too few vertices

        // get the current parent-child pair
        int p = timeline_.get_parent(i);

        // unlink the current parent
        timeline_.unlink_parent_(i, p);
        if (timeline_.twin_width() >= result_.upper_bound()) {
          timeline_.link_parent_(i, p);
          continue;  // we can't improve
        }

        // try new parent
        for (int j = i + 1; j < nu; ++j) {
          if (j == p) continue;

          timeline_.link_parent_(i, j);
          int w = timeline_.twin_width();

          if (w < result_.upper_bound()) {
            ret = updated = true;
            result_.update_upper_bound(w, timeline_.contraction_sequence());
            log_warning("%s LocalSearchPlus [parent] found new UB: ub=%d", result_.to_string().c_str(), w);
            break;
          }
          timeline_.unlink_parent_(i, j);
        }

        if (updated) break;  // move on to the next iteratione

        // get back the original parent
        timeline_.link_parent_(i, p);
      }
    }
    return ret;
  }

  // bool search_permutation(int v);

  // bool search_permutation_full();

  // bool search_red_shift(util::Random &rand);

 private:
  // bool search_red_shift_vertex_iteration(util::Random &rand);
  // bool search_red_shift_iteration(util::Random &rand);
};
}  // namespace upperbound
}  // namespace algorithms
