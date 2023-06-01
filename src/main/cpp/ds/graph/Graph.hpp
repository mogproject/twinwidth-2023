#pragma once

#include "ds/set/ArrayBitset.hpp"
#include "ds/set/FastSet.hpp"
#include "ds/set/SortedVectorSet.hpp"
#include "util/logger.hpp"

namespace ds {
namespace graph {
/**
 * @brief Graph with integer labels.
 *
 * This class does not support any modifications except `add_edge()`.
 */
class Graph {
 private:
  /** Number of vertices. */
  int n_;

  /** Number of edges; must be less than 2^31 (2147483648). */
  int m_;

  /** Adjacency lists. */
  std::vector<std::vector<int>> adj_;

  /** All-pairs symmetric differences. */
  std::vector<std::vector<std::vector<int>>> apsd_;

 public:
  Graph(std::size_t n = 0, std::vector<std::pair<int, int>> const& edges = {}) : n_(n), m_(0), adj_(n) {
    for (auto& p : edges) add_edge(p.first, p.second);
  }

  /**
   * equality
   */
  friend bool operator==(Graph const& lhs, Graph const& rhs) {
    if (lhs.n_ != rhs.n_) return false;
    if (lhs.m_ != rhs.m_) return false;
    if (lhs.adj_ != rhs.adj_) return false;    // TODO: better to disregard ordering
    if (lhs.apsd_ != rhs.apsd_) return false;  // TODO: better to disregard ordering

    return true;
  }

  friend bool operator!=(Graph const& lhs, Graph const& rhs) { return !(lhs == rhs); }

  int number_of_nodes() const { return n_; }
  int number_of_edges() const { return m_; }

  std::vector<int> const& neighbors(int v) const { return adj_[v]; }

  std::vector<std::pair<int, int>> edges() const {
    std::vector<std::pair<int, int>> ret;
    for (int i = 0; i < n_ - 1; ++i) {
      for (auto j : adj_[i]) {
        if (i < j) ret.push_back({i, j});
      }
    }
    return ret;
  }

  int degree(int v) const { return adj_[v].size(); }

  std::vector<int> degrees() const {
    std::vector<int> ret;
    for (int i = 0; i < n_; ++i) ret.push_back(degree(i));
    return ret;
  }

  bool has_vertex(int i) const { return 0 <= i && i < n_; }

  bool has_edge(int u, int v) const {
    if (!has_vertex(u) || !has_vertex(v) || u == v) return false;
    if (degree(u) > degree(v)) std::swap(u, v);
    return std::find(adj_[u].begin(), adj_[u].end(), v) != adj_[u].end();
  }

  /**
   * @brief Adds a new edge to the graph.
   *
   * Precondition: edge uv does not exist; use caution
   *
   * @param u endpoint 1
   * @param v endpoint 2
   */
  void add_edge(int u, int v) {
    assert(has_vertex(u) && has_vertex(v) && u != v);
    adj_[u].push_back(v);
    adj_[v].push_back(u);
    ++m_;
  }

  std::vector<int> const& get_symmetric_difference(int i, int j) const {
    assert(static_cast<int>(apsd_.size()) == n_ - 1);  // apsd must have been computed
    return apsd_[std::min(i, j)][std::max(i, j) - std::min(i, j) - 1];
  }

  int get_symmetric_difference_size(int i, int j) const {
    assert(static_cast<int>(apsd_.size()) == n_ - 1);  // apsd must have been computed
    return apsd_[std::min(i, j)][std::max(i, j) - std::min(i, j) - 1].size();
  }

  void compute_all_pairs_symmetric_differences() {
    if (n_ <= 1) return;
    if (static_cast<int>(apsd_.size()) == n_ - 1) return;  // assume already computed

    // resize data storage
    apsd_.clear();
    apsd_.resize(n_ - 1);
    for (int i = 0; i < n_ - 1; ++i) apsd_[i].resize(n_ - 1 - i);

    // sort vertices in increasing degree
    std::vector<std::pair<int, int>> degrees;
    for (int i = 0; i < n_; ++i) degrees.push_back({degree(i), i});
    std::sort(degrees.begin(), degrees.end());

    // compute symmetric differences for every vertex pair
    ds::FastSet fs(n_);
    for (int i = 0; i < n_ - 1; ++i) {
      int v = degrees[i].second;
      fs.clear();

      for (auto w : adj_[v]) fs.set(w);

      for (int j = i + 1; j < n_; ++j) {
        std::vector<int> diff;
        int u = degrees[j].second;
        for (auto w : adj_[u]) {
          if (w == v) continue;
          if (fs.get(w)) {
            fs.reset(w);
          } else {
            diff.push_back(w);
          }
        }

        for (auto w : adj_[v]) {
          if (fs.get(w)) {
            if (w != u) diff.push_back(w);
          } else {
            fs.set(w);
          }
        }

        apsd_[std::min(u, v)][std::max(u, v) - std::min(u, v) - 1] = diff;
      }
    }

    // log_trace("Computed all-pairs symmetric differences: n=%d", n_);
    // util::print(apsd_);
  }

  // O(n+m)
  bool is_connected() const {
    if (n_ <= 1) return true;
    if (m_ < n_ - 1) return false;

    int cnt = 1;
    ds::FastSet visited(n_);
    std::stack<int> st;
    st.push(0);
    visited.set(0);
    while (!st.empty()) {
      auto x = st.top();
      st.pop();
      for (auto y : neighbors(x)) {
        if (!visited.get(y)) {
          ++cnt;
          visited.set(y);
          st.push(y);
        }
      }
    }
    return cnt == n_;
  }

  //==================================================================================================
  // Graph conversions
  //==================================================================================================

  /**
   * @brief Creates the graph complement.
   *
   * @return Graph new Graph instance
   */
  Graph complement() const;

  /**
   * @brief Creates an induced graph with new labels.
   *
   * Label vertices[i] maps to i in the resulting graph.
   *
   * @param vertices list of vertices to induce on
   * @return Graph new Graph instance
   */
  Graph induce_and_relabel(std::vector<int> const& vertices, std::unordered_map<int, int>* label_map = nullptr,
                           bool update_apsd = true) const;

  /**
   * @brief Iteratively removes leaves (degree-1 vertices) and returns the result.
   *
   * @return Graph
   */
  Graph remove_leaves() const;

  /**
   * @brief Iteratively smoothes subdivided vertices and returns the result.
   *
   * @return Graph
   */
  Graph smooth() const;
};

//==================================================================================================
// I/O
//==================================================================================================

std::ostream& operator<<(std::ostream& os, Graph const& graph);

}  // namespace graph
}  // namespace ds
