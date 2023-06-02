#pragma once
#include <unistd.h>

#include <chrono>

#include "algorithms/upperbound/LocalSearch.hpp"
#include "algorithms/upperbound/LocalSearchPlus.hpp"
#include "algorithms/upperbound/WeakRedPotential.hpp"
#include "ds/graph/Graph.hpp"
#include "ds/graph/TriGraph.hpp"
#include "util/Random.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

namespace algorithms {
namespace upperbound {

//==============================================================================
//  Signal Handling
//==============================================================================
namespace greedy {

constexpr int const ALGORITHM_ID = 20;
extern bool volatile solver_ignore_alarm;
extern bool volatile solver_terminate_flag;

void solver_alarm_handler(int sig);
void set_timeout(int time_limit_sec);
void reset_timeout();
}  // namespace greedy

//==============================================================================
//  Main Class
//==============================================================================
template <typename T>
class GreedySolver {
 private:
  T trigraph_;
  base::SolverInfo& result_;
  std::vector<int> frozen_vertices_;
  bool local_search_enabled_;

 public:
  GreedySolver(T const& trigraph, base::SolverInfo& result, std::vector<int> const& frozen_vertices = {},
               bool local_search_enabled = true)
      : trigraph_(trigraph), result_(result), frozen_vertices_(frozen_vertices), local_search_enabled_(local_search_enabled) {
    assert(trigraph.is_compute_greedy_criteria_done());  // make sure TriGraph::compute_greedy_criteria() has been called
    // printf("----\n");
    // printf("nodes:");
    // util::print(trigraph.nodes().to_vector());
    // printf("edges:");
    // util::print(trigraph.edges());
    // printf("red edges:");
    // util::print(trigraph.red_edges());
    // printf("frozen:");
    // util::print(frozen_vertices);
    // printf("red-deg-cap: %d\n", trigraph.get_red_degree_cap());
    // printf("red degree: ");
    // for (auto i: trigraph.nodes().to_vector()) printf("%d->%d, ", i, trigraph.red_degree(i));
    // printf("\n");
    // printf("----\n");
    // trigraph.check_consistency();
  }

  void run(util::Random& rand, int num_iterations, int time_limit_sec = 0, int volatility_rate = 5000, int permissiveness = 0) {
    log_info("%s GreedySolver started: n=%d, num_iterations=%d, time_limit=%ds, volatility_rate=%d, permissiveness=%d",
             result_.to_string().c_str(), trigraph_.number_of_nodes(), num_iterations, time_limit_sec, volatility_rate,
             permissiveness);
    util::timer_start(greedy::ALGORITHM_ID);

    // set alarm
    if (time_limit_sec > 0) greedy::set_timeout(time_limit_sec);

    bool timed_out = false;
    // trigraph_.recompute_greedy_criteria(result_.upper_bound() - 1 + permissiveness);

    // main loop
    int t = 0;
    for (; t < num_iterations && !result_.resolved(); ++t) {
      if (greedy::solver_terminate_flag) {
        timed_out = true;
        break;
      }

      double volatility = 1.0 + (t / volatility_rate);
      run_iteration(rand, volatility, permissiveness, t);
    }

    // cancel alarm
    if (time_limit_sec > 0) greedy::reset_timeout();
    if (timed_out) {
      log_error("%s GreedySolver time out: t=%d, runtime=%.2fs", result_.to_string().c_str(), t,
                util::timer_stop(greedy::ALGORITHM_ID));
    } else {
      log_info("%s GreedySolver finished: t=%d, runtime=%.2fs", result_.to_string().c_str(), t,
               util::timer_stop(greedy::ALGORITHM_ID));
    }
  }

  void run_iteration(util::Random& rand, double volatility = 1.0, int permissiveness = 0, int t = 0) {
    // if (trigraph_.number_of_nodes() == 18) {
    //   printf("----\n");
    //   printf("nodes:");
    //   util::print(trigraph_.nodes().to_vector());
    //   printf("edges:");
    //   util::print(trigraph_.edges());
    //   printf("red edges:");
    //   util::print(trigraph_.red_edges());
    //   printf("frozen:");
    //   util::print(frozen_vertices_);
    //   printf("red-deg-cap: %d\n", trigraph_.get_red_degree_cap());
    //   printf("red degree: ");
    //   for (auto i : trigraph_.nodes().to_vector()) printf("%d->%d, ", i, trigraph_.red_degree(i));
    //   printf("\n");
    //   printf("----\n");
    //   trigraph_.check_consistency();
    // }

    // log_info("GreedySolver started: volatility=%.2f, upper_bound=%d", volatility, upper_bound);
    // auto timer_start = std::chrono::system_clock::now();

    // create a GreedyCriteria instance
    int wrp_ub = result_.upper_bound() + (result_.has_upper_bound() ? permissiveness : 0);
    auto criteria = WeakRedPotential<T>(trigraph_, rand, volatility, wrp_ub, frozen_vertices_);

    // main loop
    int num_frozen = static_cast<int>(frozen_vertices_.size());
    bool early_exit = false;
    int max_red_deg = 0;
    std::vector<std::pair<int, int>> seq;

    while (criteria.number_of_nodes() > max_red_deg + 1 && criteria.number_of_nodes() > num_frozen + 1) {
      // choose the best vertex pair
      auto p = criteria.dequeue();
      if (p.first < 0) {
        early_exit = true;
        break;  // could not improve the current solution
      }

      // make change
      // printf("contract (%d, %d)\n", p.second, p.first);
      auto d = criteria.contract(p.second, p.first);
      if (result_.has_upper_bound() && d >= result_.upper_bound() + permissiveness) {
        log_critical("%s GreedySolver unexpected width (%d), permissiveness=%d", result_.to_string().c_str(), d, permissiveness);
      }
      assert(!result_.has_upper_bound() || d < result_.upper_bound() + permissiveness);

      max_red_deg = std::max(max_red_deg, d);
      seq.push_back({p.second, p.first});

      // enqueue updated information (only improved pairs)
      criteria.enqueue();
    }

    if (!early_exit) {
      // rest of the contraction sequence;
      if (criteria.number_of_nodes() > 1 + num_frozen) {
        std::vector<int> vs = criteria.unfrozen_nodes();
        for (std::size_t i = 0; i < vs.size() - 1; ++i) seq.push_back({vs[vs.size() - 1], vs[i]});
      }

      assert(static_cast<int>(seq.size()) == trigraph_.number_of_nodes() - 1 - num_frozen);

      // store results
      if (result_.update_upper_bound(max_red_deg, seq)) {
        log_warning("%s GreedySolver found new UB: ub=%d, t=%d", result_.to_string().c_str(), max_red_deg, t);
      }
      if (result_.resolved()) return;

      if (local_search_enabled_) {
        // local search
        if (trigraph_.number_of_nodes() == trigraph_.original_graph().number_of_nodes() && frozen_vertices_.empty()) {
          // log_trace("%s GreedySolver local search: width=%d, t=%d", result_.to_string().c_str(), verify_contraction_sequence(trigraph_.original_graph(), seq), t);
          LocalSearch ls(trigraph_.original_graph(), result_, seq);
          ls.run(rand, 5);
        } else {
          LocalSearchPlus<T> lsp(trigraph_, result_, frozen_vertices_, seq);
          lsp.run(rand, 5);
        }
      }
      // log_trace("%s GreedySolver after LS: ub=%d, t=%d", result_.to_string().c_str(), result_.upper_bound(), t);
    }
  }
};
}  // namespace upperbound
}  // namespace algorithms
