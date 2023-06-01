#pragma once
#include <fstream>
#include <sstream>

#include "ds/graph/Graph.hpp"

namespace readwrite {
ds::graph::Graph read_pace_2023(std::istream &is);

ds::graph::Graph load_pace_2023(char const *path);
}  // namespace readwrite
