#include <gtest/gtest.h>

#include "algorithms/exact/Reducer.hpp"

using namespace ds::graph;
using namespace algorithms::exact;

typedef std::vector<std::pair<int, int>> VII;

template <typename T>
void test_reduce_free_contractions(T G) {
  Reducer red;
  Graph g1(7, {{0, 2}, {1, 2}, {1, 3}, {2, 6}, {3, 6}, {4, 6}, {5, 6}});
  g1.compute_all_pairs_symmetric_differences();
  T t1(g1);
  t1.compute_greedy_criteria(3);

  t1.make_edge_red(3, 6);
  t1.make_edge_red(4, 6);
  t1.make_edge_red(5, 6);
  auto ret1 = red.reduce_free_contractions(t1);
  EXPECT_EQ(ret1.size(), 6);
  EXPECT_EQ(t1.number_of_nodes(), 1);

  Graph g2(
      10,
      {{0, 2}, {1, 2}, {1, 3}, {2, 6}, {3, 6}, {4, 6}, {5, 6}, {3, 7}, {5, 7}, {5, 8}, {4, 8}, {3, 9}, {4, 9}, {0, 7}, {0, 8}, {0, 9}});
  g2.compute_all_pairs_symmetric_differences();
  T t2(g2);
  t2.compute_greedy_criteria(3);
  t2.make_edge_red(3, 6);
  t2.make_edge_red(4, 6);
  t2.make_edge_red(5, 6);

  auto ret2 = red.reduce_free_contractions(t2);
  EXPECT_EQ(ret2, VII({{6, 1}}));
  EXPECT_EQ(t2.number_of_nodes(), 9);

  Graph g3(5, {{0, 4}, {2, 4}, {2, 3}, {4, 3}, {4, 1}, {1, 3}});
  g3.compute_all_pairs_symmetric_differences();
  T t3(g3);
  t3.compute_greedy_criteria(3);
  t3.make_edge_red(4, 0);

  auto ret3 = red.reduce_free_contractions(t3);
  EXPECT_EQ(ret3, VII({{2, 1}, {4, 2}, {4, 3}, {4, 0}}));

  Graph g4(6, {{0, 4}, {2, 4}, {2, 3}, {4, 3}, {4, 1}, {1, 3}, {1, 5}});
  g4.compute_all_pairs_symmetric_differences();
  T t4(g4);
  t4.compute_greedy_criteria(3);
  EXPECT_TRUE(red.reduce_free_contractions(t4).empty());  // all black & no twins -> no effect

  t4.make_edge_red(4, 0);
  t4.make_edge_red(5, 1);

  auto ret4 = red.reduce_free_contractions(t4);
  EXPECT_EQ(ret4, VII({{2, 1}, {3, 2}}));
}

//
// ReducerTest
//
TEST(ReducerTest, ReduceFreeContractions) {
  auto g = Graph(10);
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, test_reduce_free_contractions);
}
