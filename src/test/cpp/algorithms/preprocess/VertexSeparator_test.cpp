#include <gtest/gtest.h>

#include "algorithms/preprocess/VertexSeparator.hpp"
#include "generators/tree.hpp"

using namespace algorithms::preprocess;
using namespace ds::graph;

typedef std::vector<int> VI;

//
// VertexSeparatorTest
//
TEST(VertexSeparatorTest, FindVertexSeparator) {
  VertexSeparator sep;
  auto g1 = Graph(7, {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}});  // K_1,6
  EXPECT_EQ(sep.find_vertex_separator(g1, 1), VI({0}));

  auto g2 = Graph(4, {{0, 1}, {0, 2}, {1, 2}, {1, 3}, {2, 3}});
  EXPECT_EQ(sep.find_vertex_separator(g2, 1), VI({}));
  EXPECT_EQ(sep.find_vertex_separator(g2, 2), VI({1, 2}));
  EXPECT_EQ(sep.find_vertex_separator(g2, 3), VI({1, 2}));  // this is the only option

  auto g3 = Graph(4, {{0, 1}, {0, 2}, {1, 2}, {1, 3}, {2, 3}, {0, 3}});  // K_4
  EXPECT_EQ(sep.find_vertex_separator(g3, 1), VI({}));
  EXPECT_EQ(sep.find_vertex_separator(g3, 2), VI({}));
  EXPECT_EQ(sep.find_vertex_separator(g3, 3), VI({}));

  auto g4 = Graph(8, {{1, 2}, {1, 4}, {5, 2}, {5, 4}, {7, 2}, {7, 4}, {6, 2}, {6, 4}, {0, 2}, {0, 4}, {3, 2}, {3, 4}});
  EXPECT_EQ(sep.find_vertex_separator(g4, 1), VI({}));
  EXPECT_EQ(sep.find_vertex_separator(g4, 2), VI({2, 4}));

  auto g5 = Graph(
      8, {{1, 2}, {1, 4}, {5, 2}, {5, 4}, {7, 2}, {7, 4}, {6, 2}, {6, 4}, {0, 2}, {0, 4}, {3, 2}, {3, 4}, {1, 5}, {5, 7}, {6, 0}, {0, 3}});
  EXPECT_EQ(sep.find_vertex_separator(g5, 1), VI({}));
  EXPECT_EQ(sep.find_vertex_separator(g5, 2), VI({2, 4}));
}

TEST(VertexSeparatorTest, DecomposeIntoCovers) {
  VertexSeparator sep;

  //----------------------------------------------------------------------------
  auto g1 = Graph(7, {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}});  // K_1,6
  auto ret1 = sep.decompose_into_covers(g1, 1);
  EXPECT_EQ(ret1.size(), 6);
  for (int i = 0; i < 6; ++i) EXPECT_EQ(ret1[i].number_of_nodes(), 2);

  //----------------------------------------------------------------------------
  auto g2 = Graph(4, {{0, 1}, {0, 2}, {1, 2}, {1, 3}, {2, 3}});
  auto ret2_1 = sep.decompose_into_covers(g2, 1);
  EXPECT_EQ(ret2_1.size(), 1);
  EXPECT_EQ(ret2_1[0].number_of_nodes(), 4);

  auto ret2_2 = sep.decompose_into_covers(g2, 2);
  EXPECT_EQ(ret2_2.size(), 2);
  EXPECT_EQ(ret2_2[0].number_of_nodes(), 3);
  EXPECT_EQ(ret2_2[1].number_of_nodes(), 3);

  //----------------------------------------------------------------------------
  auto g3 = Graph(4, {{0, 1}, {0, 2}, {1, 2}, {1, 3}, {2, 3}, {0, 3}});  // K_4
  auto ret3_1 = sep.decompose_into_covers(g3, 1);
  EXPECT_EQ(ret3_1.size(), 1);
  EXPECT_EQ(ret3_1[0].number_of_nodes(), 4);
  auto ret3_2 = sep.decompose_into_covers(g3, 2);
  EXPECT_EQ(ret3_2.size(), 1);
  EXPECT_EQ(ret3_2[0].number_of_nodes(), 4);
  auto ret3_3 = sep.decompose_into_covers(g3, 3);
  EXPECT_EQ(ret3_3.size(), 1);
  EXPECT_EQ(ret3_3[0].number_of_nodes(), 4);

  //----------------------------------------------------------------------------
  auto g4 = Graph(8, {{1, 2}, {1, 4}, {5, 2}, {5, 4}, {7, 2}, {7, 4}, {6, 2}, {6, 4}, {0, 2}, {0, 4}, {3, 2}, {3, 4}});
  auto ret4_1 = sep.decompose_into_covers(g4, 1);
  EXPECT_EQ(ret4_1.size(), 1);
  EXPECT_EQ(ret4_1[0].number_of_nodes(), 8);
  auto ret4_2 = sep.decompose_into_covers(g4, 2);
  EXPECT_EQ(ret4_2.size(), 12);
  for (int i = 0; i < 12; ++i) EXPECT_EQ(ret4_2[i].number_of_nodes(), 2);
  auto ret4_3 = sep.decompose_into_covers(g4, 3);
  EXPECT_EQ(ret4_3.size(), 12);
  for (int i = 0; i < 12; ++i) EXPECT_EQ(ret4_3[i].number_of_nodes(), 2);

  //----------------------------------------------------------------------------
  auto g5 = Graph(
      8, {{1, 2}, {1, 4}, {5, 2}, {5, 4}, {7, 2}, {7, 4}, {6, 2}, {6, 4}, {0, 2}, {0, 4}, {3, 2}, {3, 4}, {1, 5}, {5, 7}, {6, 0}, {0, 3}});
  auto ret5_1 = sep.decompose_into_covers(g5, 1);
  EXPECT_EQ(ret5_1.size(), 1);
  EXPECT_EQ(ret5_1[0].number_of_nodes(), 8);
  auto ret5_2 = sep.decompose_into_covers(g5, 2);
  EXPECT_EQ(ret5_2.size(), 2);
  for (int i = 0; i < 2; ++i) EXPECT_EQ(ret5_2[i].number_of_nodes(), 5);
  auto ret5_3 = sep.decompose_into_covers(g5, 3);
  EXPECT_EQ(ret5_3.size(), 8);
  for (int i = 0; i < 2; ++i) EXPECT_EQ(ret5_3[i].number_of_nodes(), 3);

  //----------------------------------------------------------------------------
  auto g6 = Graph(12, {
                          {3, 4},  {4, 8},  {8, 3},  {0, 5},  {5, 6}, {6, 0},  {1, 2},  {2, 10}, {10, 1},  {7, 9},
                          {9, 11}, {11, 7}, {3, 0},  {3, 5},  {3, 6}, {4, 0},  {4, 5},  {4, 6},  {8, 0},   {8, 5},
                          {8, 6},  {0, 1},  {0, 2},  {0, 10}, {5, 1}, {5, 2},  {5, 10}, {6, 1},  {6, 2},   {6, 10},
                          {1, 7},  {1, 9},  {1, 11}, {2, 7},  {2, 9}, {2, 11}, {10, 7}, {10, 9}, {10, 11},
                      });
  auto ret6_1 = sep.decompose_into_covers(g6, 1);
  EXPECT_EQ(ret6_1.size(), 1);
  EXPECT_EQ(ret6_1[0].number_of_nodes(), 12);
  auto ret6_2 = sep.decompose_into_covers(g6, 2);
  EXPECT_EQ(ret6_2.size(), 1);
  EXPECT_EQ(ret6_2[0].number_of_nodes(), 12);
  auto ret6_3 = sep.decompose_into_covers(g6, 3);
  EXPECT_EQ(ret6_3.size(), 3);
  for (int i = 0; i < 3; ++i) EXPECT_EQ(ret6_3[i].number_of_nodes(), 6);

  //----------------------------------------------------------------------------
  auto g7 = generators::path_graph(10);
  auto ret7 = sep.decompose_into_covers(g7, 3);
  EXPECT_EQ(ret7.size(), 9);
  for (int i = 0; i < 9; ++i) EXPECT_EQ(ret7[i].number_of_nodes(), 2);
}

TEST(VertexSeparatorTest, FindVertexSeparatorWithPrevSeparator) {
  VertexSeparator sep;
  auto g1 = Graph(
      8, {{0, 3}, {0, 5}, {0, 6}, {0, 7}, {1, 2}, {1, 5}, {1, 6}, {2, 3}, {2, 7}, {3, 4}, {4, 5}, {4, 6}, {4, 7}, {5, 6}});
  std::vector<int> a, b;
  auto ret = sep.find_vertex_separator(g1, 3, &a, &b, {1, 3, 7});
  EXPECT_EQ(ret, VI({0, 1, 4}));
  EXPECT_EQ(a, VI({0, 1, 2, 3, 4, 7}));
  EXPECT_EQ(b, VI({0, 1, 4, 5, 6}));

  auto g2 = Graph(5, {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}, {3, 4}});
  ret = sep.find_vertex_separator(g2, 3, &a, &b, {1, 2, 3}, {0});
  EXPECT_TRUE(ret.empty());

  auto g3 = Graph(4, {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}});
  ret = sep.find_vertex_separator(g3, 3, &a, &b, {1, 2, 3}, {0});
  EXPECT_TRUE(ret.empty());

  auto g4 = Graph(5, {{0, 1}, {0, 2}, {0, 3}, {0, 4}, {1, 2}, {1, 4}, {2, 4}});
  ret = sep.find_vertex_separator(g4, 3, &a, &b, {0, 1, 2, 3}, {4});
  EXPECT_EQ(ret, VI({0, 1, 2}));
  EXPECT_EQ(a, VI({0, 1, 2, 3}));
  EXPECT_EQ(b, VI({0, 1, 2, 4}));

  auto g5 = Graph(4, {{0, 1}, {0, 2}, {1, 2}, {1, 3}});
  ret = sep.find_vertex_separator(g5, 2, &a, &b, {}, {0, 3});
  EXPECT_EQ(ret, VI({0, 1}));
  EXPECT_EQ(a, VI({0, 1, 2}));
  EXPECT_EQ(b, VI({0, 1, 3}));
}

std::vector<int> sorted(std::vector<int> xs) {
  std::sort(xs.begin(), xs.end());
  return xs;
}

TEST(VertexSeparatorTest, FindArticulationPoints) {
  VertexSeparator sep;
  auto p5 = generators::path_graph(5);
  EXPECT_EQ(sorted(sep.find_articulation_points(p5)), VI({1, 2, 3}));

  Graph c5(5, {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}});
  EXPECT_EQ(sep.find_articulation_points(c5), VI());

  Graph g1(5, {{0, 1}, {0, 2}, {0, 3}, {0, 4}});
  EXPECT_EQ(sep.find_articulation_points(g1), VI({0}));

  Graph g2(5, {{1, 0}, {1, 2}, {1, 3}, {1, 4}});
  EXPECT_EQ(sep.find_articulation_points(g2), VI({1}));

  Graph g3(7, {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}, {0, 4}, {0, 5}, {0, 6}, {4, 5}, {4, 6}, {5, 6}});
  EXPECT_EQ(sep.find_articulation_points(g3), VI({0}));

  Graph g4(
      10,
      {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}, {1, 4}, {1, 5}, {1, 6}, {4, 5}, {4, 6}, {5, 6}, {6, 9}, {9, 8}, {8, 7}, {7, 6}});
  EXPECT_EQ(sorted(sep.find_articulation_points(g4)), VI({1, 6}));

  Graph g5(
      10,
      {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}, {1, 4}, {1, 5}, {1, 6}, {4, 5}, {4, 6}, {5, 6}, {6, 9}, {9, 8}, {8, 7}, {7, 6}, {2, 9}});
  EXPECT_EQ(sorted(sep.find_articulation_points(g5)), VI());

  Graph g6(
      16,
      {{15, 2}, {2, 1}, {1, 0}, {0, 4}, {4, 5}, {5, 7}, {7, 8}, {8, 9}, {9, 5}, {5, 6}, {6, 0}, {0, 3}, {3, 2}, {0, 14}, {5, 13}, {8, 10}, {8, 11}, {8, 12}});
  EXPECT_EQ(sorted(sep.find_articulation_points(g6)), VI({0, 2, 5, 8}));

  Graph g7(10, {{0, 1}, {1, 2}, {3, 4}, {4, 5}, {6, 7}, {6, 8}});
  EXPECT_EQ(sorted(sep.find_articulation_points(g7)), VI({1, 4, 6}));
}

TEST(VertexSeparatorTest, FindVertexSeparatorWithNonSeparator) {
  util::Random rand(12345);
  VertexSeparator sep;
  Graph g1(6, {{3, 0}, {0, 2}, {0, 4}, {2, 1}, {4, 1}, {1, 5}});

  for (int t = 0; t < 10; ++t) {
    EXPECT_EQ(sorted(sep.find_vertex_separator(g1, 2, nullptr, nullptr, {}, {}, &rand, false, false, {0, 1})), VI({2, 4}));
    EXPECT_EQ(sorted(sep.find_vertex_separator(g1, 2, nullptr, nullptr, {}, {}, &rand, false, false, {0, 4, 1})), VI());
    EXPECT_EQ(sorted(sep.find_vertex_separator(g1, 2, nullptr, nullptr, {}, {}, &rand, false, false, {5, 1, 4, 2})), VI({0}));
    EXPECT_EQ(sorted(sep.find_vertex_separator(g1, 2, nullptr, nullptr, {}, {}, &rand, false, false, {5, 1, 4, 2, 3})), VI({0}));
    EXPECT_EQ(sorted(sep.find_vertex_separator(g1, 1, nullptr, nullptr, {}, {}, &rand, false, false, {0, 1})), VI());
    EXPECT_EQ(sorted(sep.find_vertex_separator(g1, 1, nullptr, nullptr, {}, {}, &rand, false, false, {1})), VI({0}));
    EXPECT_EQ(sorted(sep.find_vertex_separator(g1, 1, nullptr, nullptr, {}, {}, &rand, false, false, {0})), VI({1}));
  }
}
