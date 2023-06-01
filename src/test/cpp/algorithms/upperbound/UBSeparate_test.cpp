#include <gtest/gtest.h>

#include "algorithms/upperbound/UBSeparate.hpp"
#include "ds/graph/TriGraph.hpp"
#include "readwrite/pace_2023.hpp"

using namespace algorithms::upperbound;
using namespace algorithms::base;
using namespace ds::graph;
typedef std::vector<int> VI;

namespace algorithms {
namespace upperbound {
namespace separate {

template <typename T>
void verify_instance(T G, int tww) {
  util::Random rand(12345);
  SolverInfo info;
  algorithms::preprocess::VertexSeparator sep;
  auto aps = sep.find_articulation_points(G.original_graph());
  G.compute_greedy_criteria(G.number_of_nodes() - 1);
  UBSeparate<T> solver(G, info, aps);
  solver.run(5, rand);

  EXPECT_EQ(info.upper_bound(), tww);  // expect to find an exact solution
}

void verify_provided_instance(char const* path, int tww) {
  auto g = readwrite::load_pace_2023(path);
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, verify_instance, tww);
}
}  // namespace separate
}  // namespace upperbound
}  // namespace algorithms

TEST(UBSeparateTest, Run) {
  // two Paley-9's with a cut vertex
  Graph g1(20, {{0, 1},   {0, 2},   {1, 2},   {3, 4},   {3, 5},   {4, 5},   {6, 7},   {6, 8},   {7, 8},   {0, 3},
                {0, 6},   {3, 6},   {1, 4},   {1, 7},   {4, 7},   {2, 5},   {2, 8},   {5, 8},   {10, 11}, {10, 12},
                {11, 12}, {13, 14}, {13, 15}, {14, 15}, {16, 17}, {16, 18}, {17, 18}, {10, 13}, {10, 16}, {13, 16},
                {11, 14}, {11, 17}, {14, 17}, {12, 15}, {12, 18}, {15, 18}, {9, 0},   {9, 1},   {9, 2},   {9, 10},
                {9, 11},  {9, 12},  {19, 0},  {19, 1},  {19, 2},  {19, 10}, {19, 11}, {19, 12}});
  g1.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g1, separate::verify_instance, 4);
}

TEST(UBSeparateTest, RunWithProvidedTinyInstances) {
  separate::verify_provided_instance("data/tiny-set/tiny001.gr", 1);  // P_10
  separate::verify_provided_instance("data/tiny-set/tiny002.gr", 2);  // C_10
  separate::verify_provided_instance("data/tiny-set/tiny005.gr", 3);  // Grid_5,5
  separate::verify_provided_instance("data/tiny-set/tiny007.gr", 2);  // tree
  separate::verify_provided_instance("data/tiny-set/tiny008.gr", 4);
  separate::verify_provided_instance("data/tiny-set/tiny009.gr", 1);
  separate::verify_provided_instance("data/tiny-set/tiny010.gr", 2);
}

template <typename T>
void test_find_separators_1(T g) {
  util::Random rand(12345);
  SolverInfo info;
  algorithms::preprocess::VertexSeparator sep;
  auto aps = sep.find_articulation_points(g.original_graph());
  g.compute_greedy_criteria(5);
  UBSeparate<T> ub_separate(g, info, aps);

  std::vector<int> a, b, s;
  s = ub_separate.find_separators(g, rand, 1, {6}, {}, &a, &b);
  EXPECT_EQ(s, VI({2}));

  s = ub_separate.find_separators(g, rand, 1, {2, 6}, {}, &a, &b);
  EXPECT_EQ(s, VI({}));

  s = ub_separate.find_separators(g, rand, 1, {2}, {3}, &a, &b);
  EXPECT_EQ(s, VI({6}));

  g.remove_vertex(3);
  g.recompute_greedy_criteria(5);

  s = ub_separate.find_separators(g, rand, 1, {}, {}, &a, &b);
  EXPECT_EQ(s, VI({6}));

  s = ub_separate.find_separators(g, rand, 1, {}, {0}, &a, &b);
  EXPECT_EQ(s, VI({6}));
  EXPECT_EQ(util::sorted(a), VI({6, 7}));
  EXPECT_EQ(util::sorted(b), VI({0, 1, 2, 4, 5, 6}));

  s = ub_separate.find_separators(g, rand, 1, {}, {7}, &a, &b);
  EXPECT_EQ(s, VI({6}));
  EXPECT_EQ(util::sorted(a), VI({0, 1, 2, 4, 5, 6}));
  EXPECT_EQ(util::sorted(b), VI({6, 7}));
}

template <typename T>
void test_find_separators_2(T g) {
  util::Random rand(12345);
  SolverInfo info;
  algorithms::preprocess::VertexSeparator sep;
  auto aps = sep.find_articulation_points(g.original_graph());
  g.compute_greedy_criteria(5);
  UBSeparate<T> ub_separate(g, info, aps);

  std::vector<int> a, b, s;

  for (int t = 0; t < 10; ++t) {
    s = ub_separate.find_separators(g, rand, 1, {}, {0}, &a, &b);
    EXPECT_EQ(s, VI({1}));
    EXPECT_EQ(util::sorted(a), VI({1, 3}));
    EXPECT_EQ(util::sorted(b), VI({0, 1, 2}));
  }
}

TEST(UBSeparateTest, FindSeparators) {
  Graph g1(8, {{0, 1}, {0, 2}, {2, 3}, {2, 4}, {4, 6}, {1, 5}, {1, 6}, {5, 6}, {6, 7}});  // ap: 2 and 6
  g1.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g1, test_find_separators_1);

  Graph g2(4, {{2,0},{0,1},{1,3}});
  g2.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g2, test_find_separators_2);
}
