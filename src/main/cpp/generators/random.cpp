#include "random.hpp"

namespace generators {
ds::graph::Graph erdos_renyi_graph(int n, double p, util::Random &rand) {
  ds::graph::Graph g(n);
  for (int i = 0; i < n - 1; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (rand.random() < p) g.add_edge(i, j);
    }
  }
  return g;
}
}  // namespace generators
