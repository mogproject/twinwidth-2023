#include "Graph.hpp"
#include "ds/queue/BucketQueue.hpp"

namespace ds {
namespace graph {
/**
 * @brief Creates the graph complement.
 *
 * @return Graph new Graph instance
 */
Graph Graph::complement() const {
  if (n_ <= 1) return *this;

  std::vector<std::vector<uint64_t>> a(n_, std::vector<uint64_t>((n_ + 63) / 64));
  for (int i = 0; i < n_; ++i) {
    for (auto j : neighbors(i)) {
      if (i < j) a[i][j / 64] |= 1ULL << j % 64;
    }
  }

  Graph ret(n_);
  for (int i = 0; i < n_ - 1; ++i) {
    for (int j = i + 1; j < n_; ++j) {
      if (((a[i][j / 64] >> (j % 64)) & 1ULL) == 0) ret.add_edge(i, j);
    }
  }
  return ret;
}

/**
 * @brief Creates an induced graph with new labels.
 *
 * Label vertices[i] maps to i in the resulting graph.
 *
 * @param vertices list of vertices to induce on
 * @return Graph new Graph instance
 */
Graph Graph::induce_and_relabel(std::vector<int> const& vertices, std::unordered_map<int, int>* label_map, bool update_apsd) const {
  FastSet fs(n_);
  std::unordered_map<int, int> label_mapping;  // current label to new label
  label_map = label_map ? label_map : &label_mapping;

  int nn = vertices.size();
  for (int i = 0; i < nn; ++i) {
    assert(0 <= vertices[i] && vertices[i] < n_);
    fs.set(vertices[i]);
    label_map->insert({vertices[i], i});
  }

  Graph ret(nn);

  for (int i = 0; i < n_; ++i) {
    if (!fs.get(i)) continue;

    for (int j : neighbors(i)) {
      if (j <= i || !fs.get(j)) continue;
      ret.add_edge(label_map->at(i), label_map->at(j));
    }
  }

  // update all-pairs symmetric differences
  if (update_apsd && static_cast<int>(apsd_.size()) == n_ - 1 && nn > 1) {
    fs.clear();
    for (auto x: vertices) fs.set(x);

    // resize data storage
    ret.apsd_.clear();
    ret.apsd_.resize(nn - 1);
    for (int i = 0; i < nn - 1; ++i) ret.apsd_[i].resize(nn - 1 - i);

    for (int i = 0; i < nn - 1; ++i) {
      for (int j = i + 1; j < nn; ++j) {
        std::vector<int> diff;
        for (auto x: get_symmetric_difference(vertices[i], vertices[j])) {
          if (fs.get(x)) diff.push_back(label_map->at(x));
        }
        ret.apsd_[i][j - i - 1] = diff;
      }
    }
  }
  return ret;
}

/**
 * @brief Iteratively removes leaves (degree-1 vertices) and returns the result.
 *
 * @return Graph
 */
Graph Graph::remove_leaves() const {
  ds::FastSet removed(n_);
  std::vector<int> degs;
  int min_deg = n_;
  for (int i = 0; i < n_; ++i) {
    int d = degree(i);
    min_deg = std::min(min_deg, d);
    degs.push_back(d);
  }
  if (min_deg > 1) return *this;  // no leaves

  ds::queue::BucketQueue q(degs);
  while (!q.empty() && q.min_value() == 1) {
    int x = q.remove_min();
    removed.set(x);
    for (auto y : neighbors(x)) {
      if (!removed.get(y)) q.update(y, -1);  // decrement the degree of x's neighbors
    }
  }

  // create an induced subgraph on the remaining vertices
  std::vector<int> vs;
  for (int i = 0; i < n_; ++i) {
    if (!removed.get(i)) vs.push_back(i);
  }
  return induce_and_relabel(vs);
}

/**
 * @brief Iteratively smoothes subdivided vertices and returns the result.
 *
 * @return Graph
 */
Graph Graph::smooth() const {
  std::vector<int> target;
  for (int i = 0; i < n_; ++i) {
    if (degree(i) == 2) target.push_back(i);
  }
  if (target.empty()) return *this;  // no vertices with degree 2

  std::vector<std::pair<int, int>> new_edges;
  ds::FastSet visited(n_), removed(n_);

  auto f = [&](int a, int b) {
    return has_edge(a, b) ||
           std::find(new_edges.begin(), new_edges.end(), std::make_pair(std::min(a, b), std::max(a, b))) != new_edges.end();
  };
  auto h = [&](int a, int b) { new_edges.push_back({std::min(a, b), std::max(a, b)}); };

  for (auto x : target) {
    if (visited.get(x)) continue;

    visited.set(x);

    // traverse to the left
    std::vector<int> left_vs = {x};
    int y = adj_[x][0];
    int y_prev = x;
    while (degree(y) == 2 && y != x) {
      left_vs.push_back(y);
      visited.set(y);

      int y_next = adj_[y][0] + adj_[y][1] - y_prev;
      y_prev = y;
      y = y_next;
    }

    int num_left = left_vs.size();

    if (left_vs.back() == adj_[x][1]) {
      // (Case 1) found a disjoint cycle: keep 3 vertices
      assert(num_left >= 3);
      for (int i = 2; i < num_left - 1; ++i) removed.set(left_vs[i]);
      if (num_left > 3) h(left_vs[1], left_vs.back());
      continue;
    }

    // traverse to the right
    std::vector<int> right_vs = {x};
    int z = adj_[x][1];
    int z_prev = x;
    while (degree(z) == 2) {
      right_vs.push_back(z);
      visited.set(z);

      int z_next = adj_[z][0] + adj_[z][1] - z_prev;
      z_prev = z;
      z = z_next;
    }

    int num_right = right_vs.size();

    // find left and right ends
    int left_end = num_left == 1 ? adj_[x][0]
                                 : adj_[left_vs[num_left - 1]][0] + adj_[left_vs[num_left - 1]][1] - left_vs[num_left - 2];
    int right_end = num_right == 1
                        ? adj_[x][1]
                        : adj_[right_vs[num_right - 1]][0] + adj_[right_vs[num_right - 1]][1] - right_vs[num_right - 2];

    if (left_end == right_end) {
      // (Case 2) cycle with one cut vertex
      assert(num_left + num_right >= 3);
      if (num_right == 1) {
        for (int i = 1; i < num_left - 1; ++i) removed.set(left_vs[i]);
      } else {
        for (int i = 0; i < num_left - 1; ++i) removed.set(left_vs[i]);
        for (int i = 1; i < num_right - 1; ++i) removed.set(right_vs[i]);
      }
      if (num_left + num_right > 3) h(left_vs[num_left - 1], right_vs[num_right - 1]);
      continue;
    }

    if (f(left_end, right_end)) {
      // (Case 3) keep a triangle
      for (int i = 0; i < num_left - 1; ++i) removed.set(left_vs[i]);
      for (int i = 1; i < num_right; ++i) removed.set(right_vs[i]);
      if (num_left + num_right > 2) h(left_vs[num_left - 1], right_end);
    } else {
      // (Case 4) make edge
      for (int i = 0; i < num_left; ++i) removed.set(left_vs[i]);
      for (int i = 1; i < num_right; ++i) removed.set(right_vs[i]);
      h(left_end, right_end);
    }
  }

  // create an induced subgraph on the remaining vertices
  std::vector<int> vs;
  for (int i = 0; i < n_; ++i) {
    if (!removed.get(i)) vs.push_back(i);
  }

  std::unordered_map<int, int> label_map;
  auto ret = induce_and_relabel(vs, &label_map);
  for (auto& p : new_edges) ret.add_edge(label_map[p.first], label_map[p.second]);

  return ret;
}

std::ostream& operator<<(std::ostream& os, Graph const& graph) {
  return os << "Graph(n=" << graph.number_of_nodes() << ",m=" << graph.number_of_edges() << ")";
}

}  // namespace graph
}  // namespace ds
