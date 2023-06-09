#include <gtest/gtest.h>

#include "algorithms/preprocess/deprecated/LCore.hpp"

using namespace ds::graph;
using namespace algorithms::preprocess;

TEST(LCoreTest, Compute) {
  //----------------------------------------------------------------------------
  auto g1 = Graph(7, {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}});  // K_1,6
  LCore lcore1(g1);
  lcore1.compute(3);
  EXPECT_TRUE(lcore1.get_cores(1).empty());

  //----------------------------------------------------------------------------
  auto g5 = Graph(
      8, {{1, 2}, {1, 4}, {5, 2}, {5, 4}, {7, 2}, {7, 4}, {6, 2}, {6, 4}, {0, 2}, {0, 4}, {3, 2}, {3, 4}, {1, 5}, {5, 7}, {6, 0}, {0, 3}});
  LCore lcore5(g5);
  lcore5.compute(3);
  EXPECT_TRUE(lcore5.get_cores(1).empty());

  //----------------------------------------------------------------------------
  auto g6 = Graph(12, {
                          {3, 4},  {4, 8},  {8, 3},  {0, 5},  {5, 6}, {6, 0},  {1, 2},  {2, 10}, {10, 1},  {7, 9},
                          {9, 11}, {11, 7}, {3, 0},  {3, 5},  {3, 6}, {4, 0},  {4, 5},  {4, 6},  {8, 0},   {8, 5},
                          {8, 6},  {0, 1},  {0, 2},  {0, 10}, {5, 1}, {5, 2},  {5, 10}, {6, 1},  {6, 2},   {6, 10},
                          {1, 7},  {1, 9},  {1, 11}, {2, 7},  {2, 9}, {2, 11}, {10, 7}, {10, 9}, {10, 11},
                      });
  LCore lcore6(g6);
  lcore6.compute(3);
  EXPECT_TRUE(lcore6.get_cores(1).empty());

  auto g7 = Graph(19, {
                          {0, 1},   {0, 2},   {1, 2},   {3, 4},   {3, 5},   {4, 5},   {6, 7},   {6, 8},   {7, 8},
                          {0, 3},   {0, 6},   {3, 6},   {1, 4},   {1, 7},   {4, 7},   {2, 5},   {2, 8},   {5, 8},
                          {10, 11}, {10, 12}, {11, 12}, {13, 14}, {13, 15}, {14, 15}, {16, 17}, {16, 18}, {17, 18},
                          {10, 13}, {10, 16}, {13, 16}, {11, 14}, {11, 17}, {14, 17}, {12, 15}, {12, 18}, {15, 18},
                          {0, 9},   {1, 9},   {2, 9},   {10, 9},  {11, 9},  {12, 9},
                      });

  //----------------------------------------------------------------------------
  LCore lcore7(g7);
  lcore7.compute(3);
  EXPECT_EQ(lcore7.get_cores(1).size(), 2);
  EXPECT_EQ(lcore7.get_cores(1)[0].number_of_nodes(), 10);
  EXPECT_EQ(lcore7.get_cores(1)[1].number_of_nodes(), 10);
}
