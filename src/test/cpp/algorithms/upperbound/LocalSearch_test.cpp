#include <gtest/gtest.h>

#include "algorithms/upperbound/LocalSearch.hpp"
#include "ds/graph/TriGraph.hpp"
#include "generators/tree.hpp"
#include "readwrite/pace_2023.hpp"
#include "util/Random.hpp"

using namespace std;
using namespace algorithms::upperbound;
using namespace algorithms::base;
using namespace ds::graph;

typedef std::vector<std::pair<int, int>> VII;

TEST(LocalSearchTest, SearchParent) {
  util::set_log_level(util::logging::NONE);

  auto g = generators::path_graph(5);
  g.compute_all_pairs_symmetric_differences();
  VII seq1 = {{0, 4}, {2, 1}, {2, 0}, {2, 3}};
  EXPECT_EQ(verify_contraction_sequence(g, seq1), 2);

  SolverInfo info1;
  info1.update_upper_bound(2, seq1);
  auto ls1 = LocalSearch(g, info1);

  EXPECT_FALSE(ls1.search_parent());

  VII seq2 = {{3, 0}, {2, 1}, {3, 2}, {4, 3}};
  EXPECT_EQ(verify_contraction_sequence(g, seq2), 3);
  SolverInfo info2;
  info2.update_upper_bound(3, seq2);
  auto ls2 = LocalSearch(g, info2);

  EXPECT_TRUE(ls2.search_parent());
  EXPECT_EQ(info2.upper_bound(), 1);
  EXPECT_EQ(info2.contraction_sequence(), VII({{1, 0}, {2, 1}, {3, 2}, {4, 3}}));

  VII seq3 = {{0, 4}, {2, 3}, {1, 2}, {0, 1}};
  EXPECT_EQ(verify_contraction_sequence(g, seq3), 2);
  SolverInfo info3;
  info3.update_upper_bound(3, seq3);
  auto ls3 = LocalSearch(g, info3);

  EXPECT_TRUE(ls3.search_parent());
  EXPECT_EQ(info3.upper_bound(), 1);
  EXPECT_EQ(info3.contraction_sequence(), VII({{3, 4}, {2, 3}, {1, 2}, {0, 1}}));

  auto p10 = generators::path_graph(10);
  p10.compute_all_pairs_symmetric_differences();
  VII seq4 = {{1, 0}, {2, 1}, {3, 2}, {6, 3}, {6, 4}, {6, 5}, {7, 6}, {8, 7}, {9, 8}};
  EXPECT_EQ(verify_contraction_sequence(p10, seq4), 3);
  SolverInfo info4;
  info4.update_upper_bound(3, seq4);
  auto ls4 = LocalSearch(p10, info4);

  EXPECT_TRUE(ls4.search_parent());
  EXPECT_EQ(info4.upper_bound(), 1);

  VII seq5 = {{1, 0}, {2, 1}, {3, 2}, {6, 3}, {5, 4}, {8, 5}, {7, 6}, {8, 7}, {9, 8}};
  EXPECT_EQ(verify_contraction_sequence(p10, seq5), 3);
  SolverInfo info5;
  info5.update_upper_bound(3, seq5);
  auto ls5 = LocalSearch(p10, info5);

  EXPECT_FALSE(ls5.search_parent());
}

TEST(LocalSearchTest, SearchPermutation) {
  util::set_log_level(util::logging::NONE);

  auto p10 = generators::path_graph(10);
  p10.compute_all_pairs_symmetric_differences();
  VII seq5 = {{1, 0}, {2, 1}, {3, 2}, {6, 3}, {5, 4}, {8, 5}, {7, 6}, {8, 7}, {9, 8}};
  EXPECT_EQ(verify_contraction_sequence(p10, seq5), 3);
  SolverInfo info;
  info.update_upper_bound(3, seq5);
  auto ls5 = LocalSearch(p10, info);

  EXPECT_TRUE(ls5.search_permutation(3));
  EXPECT_LE(info.upper_bound(), 2);
}
