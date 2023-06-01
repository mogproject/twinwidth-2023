#include <gtest/gtest.h>

#include "algorithms/lowerbound/LBSeparate.hpp"
#include "algorithms/preprocess/VertexSeparator.hpp"

using namespace algorithms::base;
using namespace algorithms::lowerbound;
using namespace ds::graph;

typedef std::vector<int> VI;

template <typename T>
void test_run(T g, int expect_lb, int expect_ub) {
  util::Random rand(12345);
  algorithms::preprocess::VertexSeparator sep;
  auto aps = sep.find_articulation_points(g.original_graph());

  SolverInfo info(-1, 0, -1);
  LBManager<T> lb_man(g, info);
  LBSeparate<T> lb_sep(lb_man, aps);
  lb_sep.run(10, 0, rand);

  EXPECT_GE(info.lower_bound(), expect_lb);
  EXPECT_LE(info.lower_bound(), expect_ub);
}

TEST(LBSeparateTest, Run) {
  Graph g1(14, {{0, 9},
                {0, 10},
                {0, 13},
                {10, 13},  //
                {9, 8},
                {8, 1},
                {8, 11},
                {8, 12},
                {1, 11},
                {1, 12},
                {11, 12},
                {8, 2},
                {8, 7},
                {2, 3},
                {3, 7},
                {3, 4},
                {3, 6},
                {6, 5}});
  g1.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g1, test_run, 1, 1);

  Graph g2(8, {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {1, 2}, {1, 5}, {2, 4}, {2, 6}, {3, 4}, {3, 7}});
  g2.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g2, test_run, 2, 2);

  Graph g3(30, {{0, 1}, {1, 2}, {2, 0}, {2, 3}, {3, 4}, {4, 2}, {29, 28}, {28, 27}, {27, 26}});
  g3.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g3, test_run, 1, 1);
}
