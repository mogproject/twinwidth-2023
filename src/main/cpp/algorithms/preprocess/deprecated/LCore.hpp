#pragma once

#include "algorithms/preprocess/VertexSeparator.hpp"
#include "ds/graph/Graph.hpp"
#include "modular/MDTree.hpp"

namespace algorithms {
namespace preprocess {

/**
 * @brief Cores for computing lower bounds.
 */
class LCore {
 private:
  constexpr static int const NUM_CANDIDATES = 3;
  ds::graph::Graph const& graph_;
  std::vector<std::vector<ds::graph::Graph>> lcores_;

 public:
  LCore(ds::graph::Graph const& graph) : graph_(graph) {}

  std::vector<ds::graph::Graph> const& get_cores(int separator_size) const { return lcores_[separator_size - 1]; }

  void compute(int max_separator_size) {
    VertexSeparator sep;
    lcores_.clear();
    lcores_.resize(max_separator_size);

    for (int sep_size = 1; sep_size <= max_separator_size; ++sep_size) {
      std::queue<ds::graph::Graph> q;  // queue of the candidates
      q.push(graph_);

      while (!q.empty()) {
        auto g = q.front();
        q.pop();
        // printf("popped: ");
        // std::cout << g << std::endl;

        if (g.number_of_nodes() <= 3) continue;  // graph too small

        // take complement if dense
        if (is_dense(g)) g = g.complement();
        // printf("compl: ");
        // std::cout << g << std::endl;

        // @note Smoothing is not safe.

        // (1) Separate
        std::vector<ds::graph::Graph> hs, gs;
        bool updated = false;
        std::vector<int> a, b;
        auto s = sep.find_vertex_separator(g, sep_size, &a, &b);
        if (!s.empty()) {
          // separator found
          updated = true;
          hs.push_back(g.induce_and_relabel(a));
          hs.push_back(g.induce_and_relabel(b));
        } else {
          hs.push_back(g);
        }
        // printf("separate: ");
        // util::print(hs);

        // (2) Modular decomposition
        for (auto& h : hs) {
          auto ps = find_prime_graphs(h);
          if (ps.empty()) continue;
          updated |= ps.size() > 1;
          updated |= ps[0].number_of_nodes() < g.number_of_nodes();
          for (auto& p : ps) gs.push_back(p);
        }
        if (gs.empty()) continue;

        // printf("modular: ");
        // util::print(gs);

        // (3) Keep only large graphs
        sort_graphs_and_trim(gs);

        // printf("trim: ");
        // util::print(gs);

        // printf("updated: %s\n", updated ? "True" : "False");

        if (updated) {
          for (auto& gg : gs) q.push(gg);
        } else {
          assert(gs.size() == 1);
          lcores_[sep_size - 1].push_back(gs[0]);  // cannot decompose further
        }
      }

      // keep only a few graphs
      sort_graphs_and_trim(lcores_[sep_size - 1]);
    }
  }

 private:
  std::vector<int> find_leader_vertices(modular::MDTree const& mdtree, int node_id) const {
    auto const& t = mdtree.get_tree();
    std::vector<int> ret;
    for (auto child : t.get_children(node_id)) {
      int leader = 0;
      for (int j = t[child].data.vertices_begin; j < t[child].data.vertices_end; ++j) {
        leader = std::max(leader, mdtree.get_vertex(j));
      }
      ret.push_back(leader);
    }
    return ret;
  }

  std::vector<ds::graph::Graph> find_prime_graphs(ds::graph::Graph const& graph) const {
    std::vector<ds::graph::Graph> ret;
    modular::MDTree mdtree(graph, false);
    auto const& t = mdtree.get_tree();

    for (auto node : t.dfs_reverse_preorder_nodes(mdtree.get_root())) {
      if (t[node].is_leaf()) continue;
      if (!t[node].data.is_prime_node()) continue;

      auto h = graph.induce_and_relabel(find_leader_vertices(mdtree, node));
      if (h.number_of_nodes() > h.number_of_edges()) continue;  // trees will be eventually decomposed into K_2
      ret.push_back(h);
    }
    return ret;
  }

  void sort_graphs_by_size(std::vector<ds::graph::Graph>& graphs) const {
    std::sort(graphs.begin(), graphs.end(), [&](ds::graph::Graph const& a, ds::graph::Graph const& b) {
      return a.number_of_nodes() > b.number_of_nodes();
    });
  }

  void sort_graphs_and_trim(std::vector<ds::graph::Graph>& graphs) const {
    sort_graphs_by_size(graphs);
    if (graphs.size() > NUM_CANDIDATES) graphs.resize(NUM_CANDIDATES);
  }

  /**
   * @brief Returns true if density > 0.5.
   *
   * @param g graph
   * @return true if density > 0.5.
   */
  bool is_dense(ds::graph::Graph const& g) const {
    return (g.number_of_edges() * 4 > g.number_of_nodes() * (g.number_of_nodes() - 1));
  }
};
}  // namespace preprocess
}  // namespace algorithms
