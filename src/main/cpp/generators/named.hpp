#pragma once

#include "ds/graph/Graph.hpp"

namespace generators {

ds::graph::Graph paley_graph_9();
ds::graph::Graph hoffman_graph();
ds::graph::Graph chvatal_graph();
ds::graph::Graph durer_graph();
ds::graph::Graph errera_graph();
ds::graph::Graph folkman_graph();
ds::graph::Graph frucht_graph();
ds::graph::Graph grotzsch_graph();

ds::graph::Graph complete_graph(int n);

}  // namespace generators
