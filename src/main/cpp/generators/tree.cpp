#include "generators/tree.hpp"

namespace generators {
/**
 * @brief Generates a path graph of `n` vertices.
 *
 * @param n number of vertices
 * @return Graph instance
 */
ds::graph::Graph path_graph(int n) { return full_rary_tree(1, n); }

/**
 * @brief Generates a full `r`-ary tree of `n` vertices.
 *
 * @param r branching factor
 * @param n number of vertices
 * @return Graph instance
 */
ds::graph::Graph full_rary_tree(int r, int n) {
  if (r <= 0) throw std::invalid_argument("r must be positive");
  std::vector<std::pair<int, int>> edges;
  for (int i = 1; i < n; ++i) edges.push_back({(i - 1) / r, i});
  return ds::graph::Graph(n, edges);
}

/**
 * @brief Generates a random tree of `n` vertices.
 *
 * @param rand util::Random instance
 * @param n number of vertices
 * @return Graph instance
 */
ds::graph::Graph random_tree(util::Random& rand, int n) {
  std::vector<int> labels;

  // randomize vertices
  for (int i = 0; i < n; ++i) labels.push_back(i);
  rand.shuffle(labels);

  // add random edges
  std::vector<std::pair<int, int>> edges;
  for (int i = 1; i < n; ++i) {
    int parent = rand.randint(0, i - 1);
    edges.push_back({labels[parent], labels[i]});
  }

  return ds::graph::Graph(n, edges);
}

}  // namespace generators
