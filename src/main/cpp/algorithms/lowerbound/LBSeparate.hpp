#pragma once

#include "algorithms/lowerbound/LBManager.hpp"
#include "algorithms/preprocess/VertexSeparator.hpp"

namespace algorithms {
namespace lowerbound {

template <typename T>
class LBSeparate {
 private:
  // base::SolverInfo& lb_manager_.result();
  LBManager<T>& lb_manager_;
  std::vector<int> articulation_points_;
  preprocess::VertexSeparator sep_;

  ds::FastSet fs_;
  ds::FastSet fs2_;
  ds::FastSet visited_;  // used for sample_degree_weighted

 public:
  LBSeparate(LBManager<T>& lb_manager, std::vector<int> const& articulation_points)
      : lb_manager_(lb_manager),
        articulation_points_(articulation_points),
        fs_(lb_manager_.get_graph().number_of_nodes()),
        fs2_(lb_manager_.get_graph().number_of_nodes()),
        visited_(lb_manager_.get_graph().number_of_nodes()) {}

  void run(int num_iterations, int alg_level, util::Random& rand) {
    log_info("%s LBSeparate start: n=%d, num_iters=%d, alg_level=%d", lb_manager_.result().to_string().c_str(),
             lb_manager_.get_graph().number_of_nodes(), num_iterations, alg_level);
    util::timer_start(9);

    for (int t = 0; t < num_iterations && !lb_manager_.resolved(); ++t) {
      rand.shuffle(articulation_points_);
      run_iteration(rand, alg_level, t);
    }

    log_info("%s LBSeparate finish: runtime=%.2fs", lb_manager_.result().to_string().c_str(), util::timer_stop(9));
  }

  void run_iteration(util::Random& rand, int alg_level, int iteration_id = 0) {
    // log_trace("%s s run_iteration: alg_level=%d, it=%d\n", lb_manager_.result().to_string().c_str(), alg_level, iteration_id);

    std::vector<int> vertices;  // initially all vertices
    for (int i = 0; i < lb_manager_.get_graph().number_of_nodes(); ++i) vertices.push_back(i);
    std::vector<int> frozen_vertices;
    bool last_loop = false;

    while (true) {
      // log_trace("%s iteration %d: compute LB n=%d", lb_manager_.result().to_string().c_str(), iteration_id, (int)vertices.size());
      // util::print(vertices);
      // printf("frozen: ");
      // util::print(frozen_vertices);

      // int n = vertices.size();
      if (lb_manager_.compute_lower_bound(vertices, alg_level, rand)) {
        // cannot obtain better lower bounds
        // printf("f1run_iteration\n");
        break;
      }
      if (last_loop) break;

      // induce
      std::unordered_map<int, int> label_map;
      auto g = lb_manager_.get_graph().induce_and_relabel(vertices, &label_map, false);
      // printf("induce: ");
      // util::print(label_map);

      // separate
      // log_trace("%s iteration %d: separate", lb_manager_.result().to_string().c_str(), iteration_id);
      if (separate(rand, g, label_map, vertices, frozen_vertices)) {
        // found a separator

        // printf("===frozen: ");
        // util::print(frozen_vertices);

        // fs_.clear();
        // for (auto x: vertices) fs_.set(x);
        // std::vector<int> removed;
        // for (int i = 0; i < lb_manager_.get_graph().number_of_nodes(); ++i) {
        //   if (!fs_.get(i)) removed.push_back(i);
        // }
        // printf("===removed: ");
        // util::print(removed);
      } else {
        // separator not found; sample vertices
        int num_samples = std::max(lb_manager_.result().lower_bound() * 2 + 4, lb_manager_.get_sample_size(alg_level));
        // log_trace("%s LBSeparate separator not found; sampling: n=%d, nsamples=%d, t=%d",
        //           lb_manager_.result().to_string().c_str(), lb_manager_.get_graph().number_of_nodes(), num_samples, iteration_id);
        vertices = convert_vertex_labels(sample_degree_weighted(g, num_samples, rand), vertices);
        last_loop = true;
      }
    }
  }

 private:
  // [0, n) -> [0, orig_n)
  inline std::vector<int> convert_vertex_labels(std::vector<int> const& vs, std::vector<int> const& mapping) const {
    std::vector<int> ret;
    for (auto x : vs) ret.push_back(mapping[x]);
    return ret;
  }

  // [0, orig_n) -> [0, n)
  inline std::vector<int> convert_vertex_labels(std::vector<int> const& vs, std::unordered_map<int, int> const& mapping) const {
    std::vector<int> ret;
    for (auto x : vs) ret.push_back(mapping.at(x));
    return ret;
  }

  bool separate(                                      //
      util::Random& rand,                             //
      ds::graph::Graph const& g,                      //
      std::unordered_map<int, int> const& label_map,  //
      std::vector<int>& vertices,                     // may be updated
      std::vector<int>& frozen_vertices               // may be updated
  ) {
    // printf("s separate\n");
    std::vector<int> a, b, s;
    int n = vertices.size();

    assert(g.is_connected());

    // find a separator
    for (                                                                                      //
        int sep_size = 1, not_found_count = 0;                                                 //
        s.empty() && sep_size < n - 1 && not_found_count < 5;                                  //
        sep_size = std::min(sep_size * 2, sep_size + std::max(1, n / 100)), ++not_found_count  //
    ) {
      s = find_separator(rand, g, label_map, vertices, frozen_vertices, sep_size, a, b);
    }

    if (s.empty()) {
      // printf("f0separate\n");
      return false;  // separator not found
    }

    // printf("vertices: ");
    // util::print(vertices);
    // printf("s: ");
    // util::print(s);
    // printf("a: ");
    // util::print(a);
    // printf("b: ");
    // util::print(b);

    // choose the larger part
    vertices = a.size() >= b.size() ? a : b;
    fs_.clear();
    for (auto x : vertices) fs_.set(x);  // set of active vertices

    // pick one vertex (spike) adjacent to the separator from outside if one exists
    int spike = -1;
    for (auto u : s) {
      for (auto v : lb_manager_.get_graph().neighbors(u)) {
        if (!fs_.get(v)) {
          spike = v;
          break;
        }
      }
      if (spike >= 0) break;
    }
    assert(spike >= 0);

    vertices.push_back(spike);
    fs_.set(spike);

    // pick the largest component of G[vertices]
    std::vector<int> largest_component;

    fs2_.clear();  // visited
    for (auto v0 : vertices) {
      if (fs2_.get(v0)) continue;
      std::vector<int> component = {v0};
      std::stack<int> st;
      st.push(v0);
      fs2_.set(v0);

      // find a component including v0
      while (!st.empty()) {
        auto x = st.top();
        st.pop();
        for (auto y : lb_manager_.get_graph().neighbors(x)) {
          if (!fs_.get(y)) continue;
          if (!fs2_.get(y)) {
            component.push_back(y);
            st.push(y);
            fs2_.set(y);
          }
        }
      }

      if (largest_component.size() < component.size()) largest_component = component;
    }

    // take care of removed frozen vertices
    vertices = largest_component;
    fs_.clear();
    for (auto x : vertices) fs_.set(x);

    std::vector<int> next_frozen;
    for (auto x : frozen_vertices) {
      if (fs_.get(x)) next_frozen.push_back(x);
    }

    // add spike and separator to the frozen vertices
    if (fs_.get(spike)) next_frozen.push_back(spike);

    for (auto x : s) {
      if (fs_.get(x)) next_frozen.push_back(x);
    }

    frozen_vertices = next_frozen;

    // printf("f1separate\n");
    return true;
  }

  std::vector<int> find_separator(                    //
      util::Random& rand,                             //
      ds::graph::Graph const& g,                      //
      std::unordered_map<int, int> const& label_map,  //
      std::vector<int> const& vertices,               //
      std::vector<int> const& frozen_vertices,        //
      int separator_size,                             //
      std::vector<int>& a,                            //
      std::vector<int>& b                             //
  ) {
    // printf("s find_separator: sep_size=%d\n", separator_size);
    std::vector<int> result_a, result_b, s;

    // printf("======================\n");
    // printf("frozen: ");
    // util::print(frozen_vertices);

    // printf("label_map: ");
    // util::print(label_map);
    // printf("======================\n");

    auto frozen_relabeled = convert_vertex_labels(frozen_vertices, label_map);

    if (separator_size == 1) {
      s = find_cut_vertex(rand, g, label_map, frozen_relabeled, result_a, result_b);
    } else {
      s = sep_.find_vertex_separator(g, separator_size, &result_a, &result_b, {}, {}, &rand, false, false, frozen_relabeled);
    }
    if (s.empty()) {
      // printf("f1find_separator\n");
      return s;  // early return
    }

    a = convert_vertex_labels(result_a, vertices);
    b = convert_vertex_labels(result_b, vertices);
    // printf("f0find_separator\n");
    return convert_vertex_labels(s, vertices);
  }

  std::vector<int> find_cut_vertex(                   //
      util::Random& rand,                             //
      ds::graph::Graph const& g,                      //
      std::unordered_map<int, int> const& label_map,  //
      std::vector<int> const& frozen_vertices,        //
      std::vector<int>& a,                            //
      std::vector<int>& b                             //
  ) {
    // assert(false);
    // traverse from a vertex
    int n = g.number_of_nodes();

    fs_.clear();  // frozen vertices
    for (auto x : frozen_vertices) fs_.set(x);

    for (auto ap_orig : articulation_points_) {
      // check if this vertex is a cut vertex of the current graph
      if (!util::contains(label_map, ap_orig)) continue;  // vertex does not exist

      auto ap = label_map.at(ap_orig);
      if (fs_.get(ap)) continue;        // frozen vertex cannot be a separator
      if (g.degree(ap) <= 1) continue;  // cut vertex must have at least 2 neighbors

      // obtain the set of vertices reachable from one of ap's neighbors avoiding ap
      auto nbrs = g.neighbors(ap);
      int start_v = nbrs[rand.randint(0, static_cast<int>(nbrs.size()) - 1)];
      std::stack<int> st;
      st.push(start_v);
      fs2_.clear();  // visited vertices
      int visited_cnt = 2;
      fs2_.set(ap);
      fs2_.set(start_v);

      while (!st.empty()) {
        auto u = st.top();
        st.pop();

        for (auto v : g.neighbors(u)) {
          if (!fs2_.get(v)) {
            ++visited_cnt;
            fs2_.set(v);
            st.push(v);
          }
        }
      }

      if (visited_cnt < n) {  // reject if all vertices were visited
        // pick ap as a separator
        a.clear();
        b.clear();
        for (int i = 0; i < n; ++i) {
          if (i == ap) {
            a.push_back(i);
            b.push_back(i);
          } else if (fs2_.get(i)) {
            a.push_back(i);
          } else {
            b.push_back(i);
          }
        }

        return {ap};
      }
    }
    return {};
  }

  std::vector<int> sample_degree_weighted(ds::graph::Graph const& graph, int sample_size, util::Random& rand) {
    // printf("s sample_degree_weighted\n");
    std::vector<int> vs, degs, q;
    int n = graph.number_of_nodes();

    if (n <= sample_size) {
      for (int i = 0; i < n; ++i) vs.push_back(i);
      return vs;
    }

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

    // printf("f sample_degree_weighted\n");
    return vs;
  }
};
}  // namespace lowerbound
}  // namespace algorithms
