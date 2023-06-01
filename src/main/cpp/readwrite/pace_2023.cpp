#include <map>

#include "ds/graph/Graph.hpp"
#include "pace_2023.hpp"

namespace readwrite {
ds::graph::Graph read_pace_2023(std::istream &is) {
  std::vector<std::pair<int, int>> edges;

  int i = 0;
  int n;
  long long m;
  int u, v;
  std::string s;

  for (std::string line; std::getline(is, line);) {
    if (line.empty()) continue;
    while (i < static_cast<int>(line.size()) && !std::isprint(line[i])) ++i;
    if (line[i] == 'c') continue;  // ignore comments

    auto ss = std::stringstream(line);
    if (line[i] == 'p') {
      ss >> s >> s >> n >> m;
    } else {
      ss >> u >> v;
      edges.push_back({u - 1, v - 1});
    }
  }
  return ds::graph::Graph(n, edges);
}

ds::graph::Graph load_pace_2023(char const *path) {
  std::ifstream f(path);
  if (f.fail()) throw std::invalid_argument(util::format("Failed to open file: %s", path));
  return read_pace_2023(f);
}
}  // namespace readwrite
