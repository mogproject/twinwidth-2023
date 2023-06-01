#include <gtest/gtest.h>

#include "algorithms/lowerbound/deprecated/LBShallow.hpp"
#include "ds/graph/TriGraph.hpp"
#include "generators/named.hpp"
#include "generators/tree.hpp"
#include "util/Random.hpp"
#include "util/util.hpp"

using namespace std;
using namespace ds::graph;
using namespace algorithms::lowerbound;
using namespace sat::status;

//
// LBShallowTest
//
TEST(LBShallowTest, RunWithNamedInstances) {
  util::set_log_level(util::logging::NONE);

  // paths
  auto p4 = generators::path_graph(4);
  p4.compute_all_pairs_symmetric_differences();
  auto lbs = LBShallow(p4);

  EXPECT_EQ(lbs.find_shallow_lb(1, 0), INCONSISTENT);
  EXPECT_EQ(lbs.find_shallow_lb(1, 1), SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(1, 2), SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(1, 3), SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(1, 4), SATISFIABLE);

  EXPECT_EQ(lbs.find_shallow_lb(2, 0), INCONSISTENT);
  EXPECT_EQ(lbs.find_shallow_lb(2, 1), SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(2, 2), SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(2, 3), SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(2, 4), SATISFIABLE);

  EXPECT_EQ(lbs.find_shallow_lb(3, 0), INCONSISTENT);
  EXPECT_EQ(lbs.find_shallow_lb(3, 1), SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(3, 2), SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(3, 3), SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(3, 4), SATISFIABLE);

  auto p16 = generators::path_graph(16);
  auto gc = generators::chvatal_graph();
  auto gd = generators::durer_graph();
  auto gf = generators::frucht_graph();
  auto gg = generators::grotzsch_graph();
  auto gp = generators::paley_graph_9();

  p16.compute_all_pairs_symmetric_differences();
  gc.compute_all_pairs_symmetric_differences();
  gd.compute_all_pairs_symmetric_differences();
  gf.compute_all_pairs_symmetric_differences();
  gg.compute_all_pairs_symmetric_differences();
  gp.compute_all_pairs_symmetric_differences();

  for (auto depth : std::vector<int>({1, 2})) {
    EXPECT_EQ(LBShallow(p16).run(depth, depth), 1);
    EXPECT_EQ(LBShallow(gc).run(depth, depth), 2);
    EXPECT_EQ(LBShallow(gd).run(depth, depth), 2);
    EXPECT_EQ(LBShallow(gf).run(depth, depth), 2);
    EXPECT_EQ(LBShallow(gg).run(depth, depth), 2);
    EXPECT_EQ(LBShallow(gp).run(depth, depth), 4);
  }

  for (auto depth : std::vector<int>({5, 8})) {
    EXPECT_EQ(LBShallow(p16).run(depth, depth), 1);
    EXPECT_EQ(LBShallow(gc).run(depth, depth), 3);
    EXPECT_EQ(LBShallow(gd).run(depth, depth), 3);
    EXPECT_EQ(LBShallow(gf).run(depth, depth), 3);
    EXPECT_EQ(LBShallow(gg).run(depth, depth), 3);
  }
}

int do_run(int n, std::vector<std::pair<int, int>> const& edges, int depth) {
  auto g = ds::graph::Graph(n, edges);
  g.compute_all_pairs_symmetric_differences();
  auto lbs = LBShallow(g);
  return lbs.run(depth, depth);
}

TEST(LBShallowTest, RunWithSmallInstances) {
  util::set_log_level(util::logging::NONE);
  EXPECT_EQ(do_run(10, {{0, 3}, {0, 5}, {0, 7}, {0, 8}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7},
                        {1, 8}, {2, 3}, {2, 4}, {2, 5}, {2, 7}, {2, 9}, {3, 6}, {3, 9}, {4, 8}, {4, 9},
                        {5, 6}, {5, 7}, {5, 8}, {5, 9}, {6, 7}, {6, 8}, {6, 9}, {7, 9}},
                   3),
            2);
  EXPECT_EQ(do_run(5, {{0, 2}, {0, 3}, {1, 2}, {1, 3}, {1, 4}, {2, 3}}, 2), 1);
  EXPECT_EQ(do_run(5, {{0, 2}, {0, 3}, {2, 3}, {2, 4}, {3, 4}}, 2), 0);
  EXPECT_EQ(do_run(5, {{0, 4}, {2, 4}}, 2), 0);
  EXPECT_EQ(do_run(15, {{0, 1},  {0, 6},  {0, 9},  {0, 10}, {0, 12}, {1, 2},   {1, 3},   {1, 5},   {1, 6},   {1, 8},
                        {1, 10}, {1, 11}, {1, 12}, {1, 14}, {2, 4},  {2, 5},   {2, 6},   {2, 7},   {2, 8},   {2, 10},
                        {2, 12}, {2, 14}, {3, 5},  {3, 6},  {3, 7},  {3, 8},   {3, 9},   {3, 10},  {3, 11},  {3, 13},
                        {4, 9},  {5, 6},  {5, 7},  {5, 8},  {5, 10}, {5, 11},  {5, 13},  {6, 8},   {6, 10},  {6, 11},
                        {6, 12}, {6, 13}, {6, 14}, {7, 8},  {7, 10}, {7, 11},  {7, 12},  {7, 13},  {8, 9},   {8, 11},
                        {8, 13}, {8, 14}, {9, 11}, {9, 13}, {9, 14}, {10, 11}, {10, 13}, {11, 12}, {12, 14}, {13, 14}},
                   4),
            3);
  EXPECT_EQ(do_run(10, {{0, 2}, {0, 4}, {0, 6}, {1, 2}, {1, 4}, {1, 6}, {2, 3}, {2, 4}, {2, 6},
                        {2, 7}, {2, 9}, {3, 4}, {3, 5}, {3, 6}, {3, 7}, {3, 8}, {3, 9}, {4, 5},
                        {4, 7}, {4, 8}, {4, 9}, {5, 6}, {5, 9}, {6, 7}, {6, 8}, {6, 9}, {7, 8}},
                   5),
            2);
  EXPECT_EQ(do_run(10, {{0, 2}, {0, 4}, {0, 5}, {0, 6}, {0, 7}, {1, 3}, {1, 4}, {1, 8}, {2, 4}, {2, 6}, {2, 7}, {3, 6},
                        {3, 9}, {4, 6}, {4, 7}, {4, 8}, {5, 7}, {5, 8}, {6, 7}, {6, 9}, {7, 8}, {7, 9}, {8, 9}},
                   3),
            3);
  EXPECT_EQ(do_run(5, {{0, 1}, {0, 3}, {0, 4}, {2, 3}, {2, 4}}, 3), 1);
}

TEST(LBShallowTest, RunWithProvidedInstances) {
  util::set_log_level(util::logging::NONE);

  // exact_002.gr
  ds::graph::Graph g(
      20, {{0, 1},   {0, 2},   {0, 3},   {0, 4},   {0, 5},   {1, 2},   {1, 6},   {1, 7},   {1, 8},   {1, 9},
           {2, 6},   {2, 3},   {2, 10},  {2, 11},  {2, 12},  {3, 6},   {3, 7},   {3, 10},  {3, 4},   {3, 16},
           {4, 10},  {4, 13},  {4, 8},   {4, 11},  {4, 16},  {4, 5},   {5, 16},  {5, 18},  {5, 19},  {5, 9},
           {5, 12},  {6, 7},   {6, 13},  {6, 14},  {6, 15},  {7, 10},  {7, 13},  {7, 8},   {7, 17},  {8, 13},
           {8, 11},  {8, 14},  {8, 17},  {8, 9},   {9, 17},  {9, 19},  {9, 12},  {9, 15},  {10, 13}, {10, 11},
           {10, 18}, {11, 14}, {11, 16}, {11, 18}, {11, 12}, {12, 18}, {12, 15}, {13, 14}, {13, 19}, {14, 16},
           {14, 17}, {14, 19}, {14, 15}, {15, 19}, {16, 17}, {16, 18}, {17, 18}, {17, 19}, {18, 19}});
  g.compute_all_pairs_symmetric_differences();

  auto lbs = LBShallow(g);
  EXPECT_EQ(lbs.find_shallow_lb(2, 5), sat::status::SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(2, 6), sat::status::SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(2, 7), sat::status::SATISFIABLE);

  EXPECT_EQ(lbs.find_shallow_lb(3, 5), sat::status::INCONSISTENT);
  EXPECT_EQ(lbs.find_shallow_lb(3, 6), sat::status::SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(3, 7), sat::status::SATISFIABLE);

  EXPECT_EQ(lbs.find_shallow_lb(4, 5), sat::status::INCONSISTENT);
  EXPECT_EQ(lbs.find_shallow_lb(4, 6), sat::status::SATISFIABLE);
  EXPECT_EQ(lbs.find_shallow_lb(4, 7), sat::status::SATISFIABLE);
}

template <typename T>
void test_partial_contraction(T G, std::vector<std::pair<int, int>> contraction) {
  for (auto& p : contraction) { EXPECT_LE(G.contract(p.first, p.second), 1); }
}

TEST(LBShallowTest, GetPartialContraction) {
  util::set_log_level(util::logging::NONE);

  auto g = generators::path_graph(16);
  g.compute_all_pairs_symmetric_differences();
  auto lbs = LBShallow(g);
  EXPECT_EQ(lbs.find_shallow_lb(5, 1), sat::status::SATISFIABLE);

  auto contraction = lbs.get_partial_contraction();
  EXPECT_EQ(contraction.size(), 5);
  RUN_WITH_TRIGRAPH(g, test_partial_contraction, contraction);
}
