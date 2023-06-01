#include "named.hpp"

namespace generators {

ds::graph::Graph paley_graph_9() {
  return ds::graph::Graph(
      9,
      {{0, 1}, {0, 2}, {1, 2}, {3, 4}, {3, 5}, {4, 5}, {6, 7}, {6, 8}, {7, 8}, {0, 3}, {0, 6}, {3, 6}, {1, 4}, {1, 7}, {4, 7}, {2, 5}, {2, 8}, {5, 8}});
}

ds::graph::Graph hoffman_graph() {
  ds::graph::Graph g(12);
  for (auto j : std::vector<int>({8, 9, 10, 11})) g.add_edge(0, j);
  for (auto j : std::vector<int>({8, 9, 10, 12})) g.add_edge(1, j);
  for (auto j : std::vector<int>({8, 11, 13, 14})) g.add_edge(2, j);
  for (auto j : std::vector<int>({9, 11, 13, 15})) g.add_edge(3, j);
  for (auto j : std::vector<int>({10, 11, 14, 15})) g.add_edge(4, j);
  for (auto j : std::vector<int>({8, 12, 13, 14})) g.add_edge(5, j);
  for (auto j : std::vector<int>({9, 12, 13, 15})) g.add_edge(6, j);
  for (auto j : std::vector<int>({10, 12, 14, 15})) g.add_edge(7, j);
  return g;
}

ds::graph::Graph chvatal_graph() {
  return ds::graph::Graph(
      12, {{0, 1}, {1, 2}, {2, 3},  {3, 0},  {4, 5}, {6, 7}, {8, 9},  {10, 11}, {0, 4}, {1, 5},  {1, 6}, {2, 7},
           {2, 8}, {3, 9}, {3, 10}, {0, 11}, {4, 8}, {5, 9}, {6, 10}, {7, 11},  {4, 7}, {5, 10}, {6, 9}, {8, 11}});
}

ds::graph::Graph durer_graph() {
  return ds::graph::Graph(
      12,
      {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 0}, {6, 8}, {7, 9}, {8, 10}, {9, 11}, {10, 6}, {11, 7}, {0, 6}, {1, 7}, {2, 8}, {3, 9}, {4, 10}, {5, 11}});
}

ds::graph::Graph errera_graph() {
  return ds::graph::Graph(17,
                          {{0, 1},   {1, 2},   {2, 0},   {0, 4},   {1, 4},   {1, 6},   {2, 6},   {2, 8},   {0, 8},
                           {0, 3},   {1, 5},   {2, 7},   {3, 4},   {4, 5},   {5, 6},   {6, 7},   {7, 8},   {8, 3},
                           {3, 10},  {4, 10},  {5, 10},  {5, 13},  {5, 16},  {3, 9},   {8, 9},   {7, 9},   {7, 12},
                           {7, 15},  {9, 10},  {10, 13}, {13, 16}, {16, 6},  {6, 15},  {15, 12}, {12, 9},  {10, 11},
                           {11, 13}, {13, 14}, {14, 16}, {9, 11},  {11, 12}, {12, 14}, {14, 15}, {11, 14}, {15, 16}});
}

ds::graph::Graph folkman_graph() {
  return ds::graph::Graph(
      20, {{0, 1},   {1, 2},  {2, 3},   {3, 0},   {4, 5},   {5, 6},   {6, 7},   {7, 4},   {8, 9},   {9, 10},
           {10, 11}, {11, 8}, {12, 13}, {13, 14}, {14, 15}, {15, 12}, {16, 17}, {17, 18}, {18, 19}, {19, 16},
           {0, 13},  {0, 15}, {1, 8},   {1, 6},   {2, 17},  {2, 19},  {3, 8},   {3, 6},   {4, 17},  {4, 19},
           {5, 12},  {5, 10}, {7, 12},  {7, 10},  {9, 16},  {9, 14},  {11, 16}, {11, 14}, {13, 18}, {15, 18}});
}

ds::graph::Graph frucht_graph() {
  ds::graph::Graph g(12, {{0, 2}, {1, 9}, {3, 5}, {4, 11}, {6, 8}, {7, 10}});
  for (int i = 0; i < 12; ++i) g.add_edge(i, (i + 1) % 12);
  return g;
}

ds::graph::Graph grotzsch_graph() {
  return ds::graph::Graph(11, {{0, 5},  {1, 5},  {1, 6},  {2, 6},  {2, 7},  {3, 7}, {3, 8}, {4, 8}, {4, 9}, {0, 9},
                               {10, 5}, {10, 6}, {10, 7}, {10, 8}, {10, 9}, {0, 2}, {1, 3}, {2, 4}, {3, 0}, {4, 1}});
}

ds::graph::Graph complete_graph(int n) {
  ds::graph::Graph g(n);
  for (int i = 0; i < n - 1; ++i) {
    for (int j = i + 1; j < n; ++j) g.add_edge(i, j);
  }
  return g;
}
}  // namespace generators
