#pragma once

#include "ds/graph/Graph.hpp"
#include "util/Random.hpp"

namespace generators {
/**
 * @brief Generates a path graph of `n` vertices.
 *
 * @param n number of vertices
 * @return Graph instance
 */
ds::graph::Graph path_graph(int n);

/**
 * @brief Generates a full `r`-ary tree of `n` vertices.
 *
 * @param r branching factor
 * @param n number of vertices
 * @return Graph instance
 */
ds::graph::Graph full_rary_tree(int r, int n);

/**
 * @brief Generates a random tree of `n` vertices.
 *
 * @param rand util::Random instance
 * @param n number of vertices
 * @return Graph instance
 */
ds::graph::Graph random_tree(util::Random& rand, int n);
}  // namespace generators
