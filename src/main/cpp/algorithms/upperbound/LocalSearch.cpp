#include <signal.h>
#include <unistd.h>

#include "LocalSearch.hpp"

namespace algorithms {
namespace upperbound {

//==============================================================================
//  Signal Handling
//==============================================================================
namespace localsearch {

constexpr int const ALGORITHM_ID = 40;
static bool volatile solver_ignore_alarm = false;
static bool volatile solver_terminate_flag = false;

static void solver_alarm_handler(int sig) {
  switch (sig) {
    case SIGALRM: {
      if (!solver_ignore_alarm) {
        solver_terminate_flag = true;
        solver_ignore_alarm = true;
      }
      break;
    }
    default: {
      // do nothing
    }
  }
}

static void set_timeout(int time_limit_sec) {
  if (time_limit_sec > 0) {
    solver_terminate_flag = false;
    solver_ignore_alarm = false;
    signal(SIGALRM, solver_alarm_handler);
    alarm(time_limit_sec);
  }
}

static void reset_timeout() {
  solver_ignore_alarm = true;
  alarm(0);  // cancel previous alarm
  signal(SIGALRM, nullptr);
}
}  // namespace localsearch

//==============================================================================
// Main Logic
//==============================================================================

void LocalSearch::run(util::Random &rand, int time_limit_sec) {
  util::timer_start(localsearch::ALGORITHM_ID);
  // log_info("%s LocalSearch started: time_limit=%ds", result_.to_string().c_str(), time_limit_sec);
  timed_out_ = false;

  if (time_limit_sec > 0) localsearch::set_timeout(time_limit_sec);

  search_red_shift(rand);
  // search_permutation_full();

  if (time_limit_sec > 0) localsearch::reset_timeout();
  if (timed_out_) {
    log_error("%s LocalSearch timed out: runtime=%.2fs", result_.to_string().c_str(), util::timer_stop(localsearch::ALGORITHM_ID));
  } else {
    // log_info("%s LocalSearch finished: runtime=%.2fs", result_.to_string().c_str(), util::timer_stop(localsearch::ALGORITHM_ID));
  }
}

bool LocalSearch::search_parent() {
  // log_info("LocalSearch: search_parent() started: ub=%d", upper_bound_);
  int n = timeline_.number_of_nodes();

  bool updated = true, ret = false;
  while (!timed_out_ && updated) {
    if (localsearch::solver_terminate_flag) {
      timed_out_ = true;
      break;
    }

    updated = false;
    for (int i = 0; i < n - 1; ++i) {
      if (n - i - 1 <= result_.upper_bound()) break;  // too few vertices

      // get the current parent-child pair
      int p = timeline_.get_parent(i);

      // unlink the current parent
      timeline_.unlink_parent_(i, p);
      if (timeline_.twin_width() >= result_.upper_bound()) {
        timeline_.link_parent_(i, p);
        continue;  // we can't improve
      }

      // try new parent
      for (int j = i + 1; j < n; ++j) {
        if (j == p) continue;

        timeline_.link_parent_(i, j);
        int w = timeline_.twin_width();

        if (w < result_.upper_bound()) {
          ret = updated = true;
          result_.update_upper_bound(w, timeline_.contraction_sequence());
          log_warning("%s LocalSearch [parent] found new UB: ub=%d", result_.to_string().c_str(), w);
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

bool LocalSearch::search_permutation(int v) {
  // log_info("LocalSearch: search_permutation() started: ub=%d", upper_bound_);
  int n = timeline_.number_of_nodes();

  bool updated = true, ret = false;
  while (!timed_out_ && updated) {
    updated = false;

    for (int u = 0; u < n && !timed_out_; ++u) {
      if (u == v) continue;

      if (localsearch::solver_terminate_flag) {
        timed_out_ = true;
        break;
      }

      bool perm_updated = false;

      // try new permutation
      timeline_.update_permutation(u, v);
      int w = timeline_.twin_width();

      if (w < result_.upper_bound()) {
        perm_updated = true;
        result_.update_upper_bound(w, timeline_.contraction_sequence());
        log_warning("%s LocalSearch [perm] found new UB: ub=%d", result_.to_string().c_str(), w);
      }

      // further, find the best parent
      perm_updated |= search_parent();
      if (perm_updated) {
        ret = updated = true;
        break;
      }

      // get back the original permutation
      timeline_.update_permutation(u, v);
    }
  }
  return ret;
}

bool LocalSearch::search_permutation_full() {
  int n = timeline_.number_of_nodes();

  bool updated = true, ret = false;

  while (!timed_out_ && updated) {
    updated = false;

    for (int i = 0; i < n && !timed_out_; ++i) {
      if (search_permutation(i)) {
        ret = updated = true;
        break;
      }
    }
  }
  return ret;
}

// bool LocalSearch::search_triangle_permutation_full() {
//   int n = timeline_.number_of_nodes();
//   int orig_w = timeline_.twin_width();
//   bool updated = false;

//   for (int i = 0; i < n - 2 && !updated; ++i) {
//     for (int j = i + 1; j < n && !updated; ++j) {
//       for (int k = i + 1; k < n && !updated; ++k) {
//         if (j == k) continue;

//         // (i,j,k)
//         timeline_.update_permutation(i, j);
//         timeline_.update_permutation(j, k);

//         int w = timeline_.twin_width();

//         if (w < orig_w) {
//           updated = true;
//           result_.update_upper_bound(w, timeline_.contraction_sequence());
//           log_warning("%s LocalSearch found new UB: ub=%d", result_.to_string().c_str(), w);
//         }
//         updated |= search_parent();
//         if (updated) break;

//         // get back
//         timeline_.update_permutation(j, k);
//         timeline_.update_permutation(i, j);
//       }
//     }
//   }
//   return updated;
// }

bool LocalSearch::search_red_shift_vertex_iteration(util::Random &rand) {
  int n = timeline_.number_of_nodes();
  int orig_w = timeline_.twin_width();
  int target = orig_w - 1;
  bool updated = false;

  std::vector<int> vertices;
  for (int i = 0; i < n; ++i) vertices.push_back(i);

  for (int it = 0; it < 1000; ++it) {
    // 1. Create a random permutation of all vertices
    rand.shuffle(vertices);

    // 2. Find a vertex such that red degree equals max red degree
    int i = 0;
    for (; i < n; ++i) {
      if (timeline_.get_max_red_degree_by_vertex(vertices[i]) > target) break;
    }
    int v = vertices[i];

    // 3. Pick another vertex to change permutation
    for (int j = 0; j < n; ++j) {
      if (i == j) continue;

      // 4. Try permutation
      int u = vertices[j];
      timeline_.update_permutation(v, u);

      if (timeline_.get_max_red_degree_by_vertex(v) <= target) {
        // ok, resolved this vertex
        break;
      } else {
        timeline_.update_permutation(v, u);  // revert previous permutation
      }
    }

    int w = timeline_.twin_width();
    if (w <= target) {
      updated = true;
      result_.update_upper_bound(w, timeline_.contraction_sequence());
      log_warning("%s LocalSearch [redshift1] found new UB: ub=%d", result_.to_string().c_str(), w);
      search_parent();
      target = result_.upper_bound() - 1;
      if (result_.resolved()) break;
    }
  }
  return updated;
}

bool LocalSearch::search_red_shift_iteration(util::Random &rand) {
  int n = timeline_.number_of_nodes();
  int orig_w = timeline_.twin_width();
  int target = orig_w - 1;

  for (int t = 0; t < n - 1 && !timed_out_; ++t) {
    int max_red_degree = timeline_.get_max_red_degree_at_time(t);
    if (max_red_degree > target) {
      int x = timeline_.phiinv(t);

      // try changing permutation
      std::vector<int> vs;
      for (int s = t + 1; s < n; ++s) vs.push_back(s);

      rand.shuffle(vs);

      bool ok = false;

      for (auto v : vs) {
        if (localsearch::solver_terminate_flag) {
          timed_out_ = true;
          break;
        }

        int y = timeline_.phiinv(v);
        timeline_.update_permutation(x, y);

        if (timeline_.get_max_red_degree_at_time(t) <= target) {
          ok = true;  // condition satisfied
          break;
        }
        timeline_.update_permutation(x, y);  // revert permutation
      }
      if (!ok) return false;
    }
  }

  // find better parents
  search_parent();

  if (result_.update_upper_bound(timeline_.twin_width(), timeline_.contraction_sequence())) {
    log_warning("%s LocalSearch [redshift] found new UB: ub=%d", result_.to_string().c_str(), timeline_.twin_width());
    return true;
  } 
  return false;
}

bool LocalSearch::search_red_shift(util::Random &rand) {
  bool ret = false;
  bool updated = true;
  while (updated) {
    updated = search_red_shift_iteration(rand);
    // updated = search_red_shift_vertex_iteration(rand);

    updated |= search_parent();
    ret |= updated;
  }

  return ret;
}

}  // namespace upperbound
}  // namespace algorithms
