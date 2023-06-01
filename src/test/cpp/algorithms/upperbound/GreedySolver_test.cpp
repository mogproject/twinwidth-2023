#include <gtest/gtest.h>

#include "algorithms/upperbound/GreedySolver.hpp"
#include "ds/graph/TriGraph.hpp"
#include "readwrite/pace_2023.hpp"
#include "util/Random.hpp"

using namespace std;
using namespace algorithms::upperbound;
using namespace ds::graph;
using namespace algorithms::base;

typedef std::vector<int> VI;
typedef std::vector<std::pair<int, int>> VII;

template <typename T>
void verify_instance(T G, int tww, int max_error) {
  util::Random rand(12345);
  SolverInfo info;
  G.compute_greedy_criteria();
  GreedySolver<T> solver(G, info);

  // first run
  solver.run_iteration(rand, 1.0, -1);
  int prev = info.upper_bound();
  EXPECT_GE(prev, tww);
  EXPECT_LE(prev, tww + max_error);

  // second run
  solver.run_iteration(rand, 1.0, prev);
  int result = info.upper_bound();
  EXPECT_GE(result, tww);
  EXPECT_LE(result, prev);
}

void verify_provided_instance(char const* path, int tww, int max_error) {
  auto g = readwrite::load_pace_2023(path);
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, verify_instance, tww, max_error);
}

TEST(GreedySolverTest, RunWithProvidedTinyInstances) {
  util::set_log_level(util::logging::NONE);

  verify_provided_instance("data/tiny-set/tiny001.gr", 1, 1);  // P_10
  verify_provided_instance("data/tiny-set/tiny002.gr", 2, 0);  // C_10
  verify_provided_instance("data/tiny-set/tiny003.gr", 0, 0);  // K_10
  verify_provided_instance("data/tiny-set/tiny004.gr", 0, 0);  // K_1,9
  verify_provided_instance("data/tiny-set/tiny005.gr", 3, 1);  // Grid_5,5
  verify_provided_instance("data/tiny-set/tiny006.gr", 0, 0);  // 5 * K_2
  verify_provided_instance("data/tiny-set/tiny007.gr", 2, 1);  // tree
  verify_provided_instance("data/tiny-set/tiny008.gr", 4, 1);
  verify_provided_instance("data/tiny-set/tiny009.gr", 1, 1);
  verify_provided_instance("data/tiny-set/tiny010.gr", 2, 1);
}

template <typename T>
void test_run_with_frozen_vertices(T G) {
  util::Random rand(12345);
  SolverInfo info;
  G.compute_greedy_criteria();
  G.remove_vertex(5);
  G.remove_vertex(6);
  G.remove_vertex(7);
  G.remove_vertex(8);
  G.remove_vertex(9);
  G.remove_vertex(10);
  G.remove_vertex(11);
  G.remove_vertex(12);
  G.remove_vertex(14);
  G.remove_vertex(15);
  G.make_edge_red(13, 4);
  G.add_ncr(0, 13, 1);
  G.add_ncr(1, 13, 1);
  G.add_ncr(2, 13, 1);
  G.check_consistency();

  GreedySolver<T> solver(G, info, {0, 1, 2, 3}, false);
  solver.run(rand, 3, 5);
  EXPECT_EQ(info.upper_bound(), 3);
  EXPECT_EQ(info.contraction_sequence(), VII({{13, 4}}));
}

TEST(GreedySolverTest, RunWithFrozenVertices) {
  auto g = Graph(16, {{13, 4}, {4, 0}, {4, 1}, {4, 2}, {0, 1}, {0, 2}, {1, 2}, {0, 3}});
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, test_run_with_frozen_vertices);
}

template <typename T>
void test_run_with_frozen_vertices_2(T G, util::Random& rand) {
  SolverInfo info(-1, 3, 4);
  G.compute_greedy_criteria(3);
  for (auto x : VI({0, 4, 5, 29, 40, 41})) G.remove_vertex(x);
  G.make_edge_red(1, 6);
  G.make_edge_red(3, 6);
  G.make_edge_red(6, 10);
  G.recompute_greedy_criteria(3);

  GreedySolver<T> solver(G, info, {30}, false);
  solver.run(rand, 100, 0, 30, 0);
  EXPECT_EQ(info.upper_bound(), 4);
}

TEST(GreedySolverTest, RunWithFrozenVertices2) {
  VII edges = {{1, 2},   {1, 3},   {1, 6},   {1, 11},  {2, 11},  {2, 12},  {2, 13},  {2, 15},  {3, 6},   {3, 10},
               {3, 11},  {3, 13},  {6, 10},  {7, 9},   {7, 10},  {7, 11},  {7, 12},  {8, 11},  {8, 21},  {8, 22},
               {8, 25},  {9, 10},  {9, 11},  {9, 21},  {10, 11}, {10, 20}, {11, 13}, {11, 20}, {12, 13}, {12, 14},
               {12, 20}, {12, 21}, {13, 20}, {13, 24}, {14, 18}, {14, 24}, {15, 16}, {15, 17}, {15, 18}, {15, 23},
               {16, 17}, {16, 18}, {16, 19}, {17, 18}, {17, 23}, {18, 30}, {19, 30}, {20, 33}, {21, 35}, {22, 24},
               {22, 25}, {22, 31}, {22, 32}, {22, 33}, {23, 24}, {23, 25}, {23, 31}, {24, 34}, {25, 31}, {26, 27},
               {26, 28}, {26, 31}, {26, 32}, {27, 31}, {27, 32}, {28, 30}, {30, 36}, {31, 32}, {31, 33}, {31, 37},
               {32, 38}, {32, 39}, {33, 35}, {33, 37}, {33, 38}, {34, 35}, {34, 38}, {35, 38}, {36, 42}, {37, 42},
               {38, 42}};  //, {39, 42}};
  auto g = Graph(43, edges);
  util::Random rand(12345);
  g.compute_all_pairs_symmetric_differences();
  for (int t = 0; t < 10; ++t) { RUN_WITH_TRIGRAPH(g, test_run_with_frozen_vertices_2, rand); }
}
