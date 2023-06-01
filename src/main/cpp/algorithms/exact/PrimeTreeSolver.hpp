#pragma once
#include <cassert>

#include "ds/graph/Graph.hpp"
#include "ds/set/FastSet.hpp"
#include "util/Random.hpp"
#include "util/logger.hpp"

namespace algorithms {
namespace exact {

class PrimeTreeSolver {
 private:
  ds::graph::Graph const &graph_;
  int twin_width_;
  std::vector<std::pair<int, int>> contraction_;

 public:
  /**
   * @brief Construct a new PrimeTreeSolver object
   *
   * @param graph prime tree (connected acyclic graph with no twins)
   */
  PrimeTreeSolver(ds::graph::Graph const &graph) : graph_(graph), twin_width_(0) {
    assert(graph.number_of_edges() == std::max(0, graph.number_of_nodes() - 1));
  }

  int twin_width() const { return twin_width_; }

  std::vector<std::pair<int, int>> contraction_sequence() const { return contraction_; }

  void run() {
    if (is_caterpillar()) {
      log_trace("Found a caterpillar: n=%d, m=%d", graph_.number_of_nodes(), graph_.number_of_edges());
      solve_caterpillar();
    } else {
      log_trace("Found a non-caterpillar tree: n=%d, m=%d", graph_.number_of_nodes(), graph_.number_of_edges());
      solve_noncaterpillar();
    }
  }

  /**
   * @brief Check if a tree is a caterpillar by checking if each vertex has
   * at most 2 neighbors with degree >= 2.
   *
   * @return true if the tree is a caterpillar
   */
  bool is_caterpillar() {
    for (int i = 0; i < graph_.number_of_nodes(); ++i) {
      if (graph_.degree(i) <= 2) continue;
      int count = 0;
      for (auto j : graph_.neighbors(i)) {
        if (graph_.degree(j) >= 2) {
          if (++count > 2) return false;
        }
      }
    }
    return true;
  }

 private:
  /**
   * @brief Solves a prime caterpillar in a twin-width 1 contraction sequence.
   *
   * If the graph is not a prime caterpillar, behavior is undefined.
   */
  void solve_caterpillar() {
    int n = graph_.number_of_nodes();
    ds::FastSet removed(n);  // manage removed vertices

    // find an end of the spine
    int u = 0, v = 0;
    for (u = 0; u < n; ++u) {
      if (graph_.degree(u) == 1) {
        v = graph_.neighbors(u)[0];        // u's only neighbor
        if (graph_.degree(v) == 2) break;  // found one
      }
    }

    while (v >= 0) {
      // remove vertex u
      removed.set(u);

      // check if v is adjacent to a leaf other than u
      int nextu = v, nextv = -1;

      for (auto w : graph_.neighbors(v)) {
        if (removed.get(w)) continue;  // already removed
        if (graph_.degree(w) == 1) {
          nextu = w;
          nextv = v;
          break;
        } else {
          nextv = w;
        }
      }

      // add contraction
      contraction_.push_back({nextu, u});
      u = nextu;
      v = nextv;
    }
    twin_width_ = 1;
  }

  void solve_noncaterpillar() {
    // construct a rooted tree
    int n = graph_.number_of_nodes();
    std::vector<int> last_child(n, -1), left_sibling(n, -1);
    auto xs = reverse_dfs(0, last_child, left_sibling);

    // contract vertices from the bottom
    for (auto x : xs) {
      // merge last child
      if (last_child[x] >= 0) contraction_.push_back({x, last_child[x]});
      // merge left sibling
      if (left_sibling[x] >= 0) contraction_.push_back({x, left_sibling[x]});
    }

    twin_width_ = 2;
  }

  // reverse dfs nodes
  std::vector<int> reverse_dfs(int root, std::vector<int> &last_child, std::vector<int> &left_sibling) {
    std::vector<int> ret;
    std::vector<std::pair<int, int>> stack;
    stack.push_back({-1, root});

    while (!stack.empty()) {
      auto p = stack.back();
      auto parent = p.first;
      auto x = p.second;
      stack.pop_back();
      ret.push_back(x);

      int prev = -1;
      for (int y : graph_.neighbors(x)) {
        if (y == parent) continue;

        if (prev >= 0) left_sibling[y] = prev;
        prev = y;
        last_child[x] = y;
        stack.push_back({x, y});
      }
    }
    std::reverse(ret.begin(), ret.end());
    return ret;
  }
};
}  // namespace exact
}  // namespace algorithms
