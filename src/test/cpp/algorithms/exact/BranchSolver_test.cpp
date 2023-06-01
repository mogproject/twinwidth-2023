#include <gtest/gtest.h>

#include "algorithms/exact/BranchSolver.hpp"
#include "generators/named.hpp"
#include "generators/tree.hpp"

using namespace algorithms::exact;
using namespace algorithms::base;
using namespace generators;
using namespace ds::graph;

typedef std::vector<int> VI;
typedef std::vector<std::pair<int, int>> VII;

template <typename T>
void test_run(T G, int lb, int expect) {
  SolverInfo result;
  BranchSolver<T> solver(G, result);
  solver.run(0);
  EXPECT_EQ(result.lower_bound(), expect);
  EXPECT_EQ(result.upper_bound(), expect);
}

//
// BranchSolverTest
//
TEST(BranchSolverTest, RunWithNamedGraphs) {
  // util::set_log_level(util::logging::NONE);

  Graph g;
  g = path_graph(5);
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, test_run, 0, 1);

  g = path_graph(8);
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, test_run, 0, 1);

  g = complete_graph(8);
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, test_run, 0, 0);

  g = chvatal_graph();
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, test_run, 0, 3);

  g = durer_graph();
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, test_run, 2, 3);
}

template <typename T>
void test_branch_frozen_vertices_1(T G) {
  util::Random rand(12345);
  SolverInfo info;
  G.compute_greedy_criteria();
  for (auto v: VI({5, 6, 7, 8, 9, 10, 11, 12, 14, 15})) G.remove_vertex(v);
  G.make_edge_red(13, 4);
  G.recompute_greedy_criteria();
  G.check_consistency();

  BranchSolver<T> solver(G, info, {0, 1, 2, 3});
  solver.run();
  EXPECT_EQ(info.lower_bound(), 3);
  EXPECT_EQ(info.upper_bound(), 3);
  EXPECT_EQ(info.contraction_sequence(), VII({{13, 4}}));
}

template <typename T>
void test_branch_frozen_vertices_2(T G) {
  util::Random rand(12345);
  SolverInfo info;
  G.compute_greedy_criteria();
  for (auto v: VI({5, 6, 7, 8, 9, 10, 11, 12, 14, 15})) G.remove_vertex(v);
  G.make_edge_red(0, 4);
  G.make_edge_red(1, 4);
  G.make_edge_red(2, 4);
  G.add_edge(13, 2, true);
  G.recompute_greedy_criteria();
  G.check_consistency();

  BranchSolver<T> solver(G, info, {0, 1, 2, 3});
  solver.run();
  EXPECT_EQ(info.lower_bound(), 3);
  EXPECT_EQ(info.upper_bound(), 3);
  EXPECT_EQ(info.contraction_sequence(), VII({{13, 4}}));
}

TEST(BranchSolverTest, RunWithFrozenVertices) {
  auto g = Graph(16, {{13, 4}, {4, 0}, {4, 1}, {4, 2}, {0, 1}, {0, 2}, {1, 2}, {0, 3}});
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, test_branch_frozen_vertices_1);
  RUN_WITH_TRIGRAPH(g, test_branch_frozen_vertices_2);
}
