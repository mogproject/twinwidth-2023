#include "LBSample.hpp"

#include "algorithms/ExactSolver.hpp"
#include "algorithms/preprocess/VertexSeparator.hpp"

namespace algorithms {
namespace lowerbound {

void LBSample::preprocess(bool remove_leaves, bool smooth, int separator_size) {
  if (remove_leaves) processed_graphs_.back() = processed_graphs_.back().remove_leaves();
  if (smooth) processed_graphs_.back() = processed_graphs_.back().smooth();

  if (separator_size > 0) {
    preprocess::VertexSeparator sep;
    auto ret = sep.decompose_into_covers(processed_graphs_.back(), separator_size);
    processed_graphs_.clear();
    for (auto& g : ret) {
      if (g.number_of_nodes() <= result_.lower_bound() + 1) continue;
      processed_graphs_.push_back(g);
    }
  }

#if LOGGING_ON
  for (auto& g : processed_graphs_) {
    log_trace("%s LBSample preprocess: new_n=%d, new_m=%d", result_.to_string().c_str(), g.number_of_nodes(), g.number_of_edges());
  }
#endif
  // TODO: modular decomposition
}

int LBSample::run(util::Random& rand, std::vector<sample::SampleOptions> const& options, int time_limit_sec) {
  log_info("%s LBSample started: time_limit=%ds", result_.to_string().c_str(), time_limit_sec);
  util::timer_start(10);
  base::TimeManager tm(time_limit_sec);

  bool timeout = false;
  int prev_lb = result_.lower_bound();
  int prev_log_level = util::logging::log_level;
  util::set_log_level(util::logging::NONE);

  for (auto& opt : options) {
    if (result_.lower_bound() >= opt.lb_required) continue;  // LB too large

    for (auto& g : processed_graphs_) {
      if (g.number_of_nodes() <= result_.lower_bound() + 1) continue;  // graph too small
      // if (g.number_of_nodes() <= opt.max_sample_size) continue;        // never recurse infinitely

      int num_iterations = opt.num_iterations;
      for (int t = 0; t < num_iterations; ++t) {
        std::vector<int> vs;
        switch (opt.strategy) {
          case sample::SAMPLE_UNIFORM: {
            vs = sample_naive(g, rand, opt.max_sample_size);
            break;
          }
          case sample::SAMPLE_DEGREE: {
            vs = sample_degree_weighted(g, rand, opt.max_sample_size);
            break;
          }
          case sample::SAMPLE_PEEL: {
            g.compute_all_pairs_symmetric_differences();
            vs = sample_peel(g, rand, opt.max_sample_size, result_.lower_bound());
            break;
          }
          default: {
          }
        }

        auto h = g.induce_and_relabel(vs);

        ExactSolver solver(h);
        int time_limit = tm.adjust_time_limit(opt.time_limit_per_iteration);
        solver.run(rand, result_.lower_bound(), result_.upper_bound(), true, time_limit);  // run with LB mode
        result_.update_lower_bound(solver.twin_width());

        // vvv temporary implementation
        // h.compute_all_pairs_symmetric_differences();
        // exact::DirectSolver solver(h);
        // solver.run(result_.lower_bound(), -1, opt.time_limit_per_iteration);
        // result_.update_lower_bound(solver.lower_bound());
        // ^^^ ------------------------
        if (tm.is_time_over()) timeout = true;
        if (timeout || result_.resolved()) break;
      }
      if (timeout || result_.resolved()) break;
    }
    if (timeout || result_.resolved()) break;
  }

  util::set_log_level(prev_log_level);
  if (prev_lb < result_.lower_bound()) {
    log_warning("%s LBSample found new LB: lb=%d, runtime=%.2fs", result_.to_string().c_str(), result_.lower_bound(),
                util::timer_stop(10));
  } else {
    log_info("%s LBSample finished: lb=%d (no change), runtime=%.2fs", result_.to_string().c_str(),
             result_.lower_bound(), util::timer_stop(10));
  }
  return result_.lower_bound();
}

std::vector<int> LBSample::sample_naive(ds::graph::Graph const& graph, util::Random& rand, int sample_size) {
  std::vector<int> vs, q;
  int n = graph.number_of_nodes();
  visited_.clear();

  // randomly pick a starting vertex
  int start_v = rand.randint(0, n - 1);
  vs.push_back(start_v);
  q.push_back(start_v);
  visited_.set(start_v);

  for (int cursor = 0; cursor < sample_size && cursor < static_cast<int>(vs.size()); ++cursor) {
    // pick next destination
    int i = rand.randint(cursor, static_cast<int>(vs.size()) - 1);
    std::swap(vs[cursor], vs[i]);
    int u = vs[cursor];

    // enqueue u's unseen neighbors
    for (auto v : graph.neighbors(u)) {
      if (!visited_.get(v)) {
        visited_.set(v);
        vs.push_back(v);
      }
    }
  }

  // trim vertices
  if (sample_size < static_cast<int>(vs.size())) vs.resize(sample_size);
  return vs;
}

std::vector<int> LBSample::sample_degree_weighted(ds::graph::Graph const& graph, util::Random& rand, int sample_size) {
  std::vector<int> vs, degs, q;
  auto all_degs = graph.degrees();
  visited_.clear();

  // pick the first vertex
  int start_v = rand.weighted_choice(all_degs);
  vs.push_back(start_v);
  degs.push_back(all_degs[start_v]);
  q.push_back(start_v);
  visited_.set(start_v);

  for (int cursor = 0; cursor < sample_size && cursor < static_cast<int>(vs.size()); ++cursor) {
    // pick next destination
    int i = rand.weighted_choice(degs.begin() + cursor, degs.end()) + cursor;
    std::swap(vs[cursor], vs[i]);
    std::swap(degs[cursor], degs[i]);
    int u = vs[cursor];

    // enqueue u's unseen neighbors
    for (auto v : graph.neighbors(u)) {
      if (!visited_.get(v)) {
        visited_.set(v);
        vs.push_back(v);
        degs.push_back(all_degs[v]);
      }
    }
  }

  // trim vertices
  if (sample_size < static_cast<int>(vs.size())) vs.resize(sample_size);
  return vs;
}

std::vector<int> LBSample::sample_peel(ds::graph::Graph const& graph, util::Random& rand, int sample_size, int lb) {
  int n = graph.number_of_nodes();
  double sum = 0.0;
  std::vector<int> count(n);
  for (int i = 0; i < n - 1; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (graph.get_symmetric_difference_size(i, j) <= lb) {
        ++count[i];
        ++count[j];
        sum += 2.0;
      }
    }
  }
  assert(sum != 0.0);

  int num_to_remove = n - sample_size;
  int num_removed = 0;
  for (int i = 0; i < n && num_removed < num_to_remove; ++i) {
    if (n - i == num_to_remove - num_removed || rand.random() < count[i] / sum) {
      count[i] = -1;  // chosen
      ++num_removed;
    }
  }
  assert(num_removed == num_to_remove);

  std::vector<int> ret;
  for (int i = 0; i < n; ++i) {
    if (count[i] >= 0) ret.push_back(i);
  }
  return ret;
}

}  // namespace lowerbound
}  // namespace algorithms
