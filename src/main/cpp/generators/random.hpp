#pragma once

#include "ds/graph/Graph.hpp"
#include "util/Random.hpp"

namespace generators {
ds::graph::Graph erdos_renyi_graph(int n, double p, util::Random &rand);
}
