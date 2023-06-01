#include <gtest/gtest.h>

#include "ds/graph/Graph.hpp"
#include "generators/tree.hpp"

using namespace std;
using namespace ds::graph;

typedef std::vector<int> VI;

template <typename Graph>
inline std::vector<std::vector<int>> create_adj(Graph const& g) {
  int n = g.number_of_nodes();
  std::vector<std::vector<int>> ret(n, std::vector<int>(n));

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if (g.has_vertex(i) && g.has_vertex(j)) {
        ret[i][j] = g.has_edge(i, j) ? 1 : 0;
      } else {
        ret[i][j] = 2;
      }
    }
  }
  return ret;
}

std::vector<std::pair<int, int>> edges = {                   //
    {0, 1}, {1, 2}, {0, 3}, {0, 4}, {0, 5}, {1, 3}, {4, 1},  //
    {1, 5}, {3, 2}, {4, 2}, {5, 2}, {4, 6}, {6, 5}};

//
// GraphTest
//
TEST(GraphTest, BasicOperations) {
  auto G = Graph(7, edges);

  // properties
  EXPECT_EQ(G.number_of_nodes(), 7);
  EXPECT_EQ(G.number_of_edges(), 13);

  // queries
  EXPECT_TRUE(G.has_vertex(0));
  EXPECT_TRUE(G.has_vertex(6));
  EXPECT_FALSE(G.has_vertex(7));
  EXPECT_FALSE(G.has_vertex(-1));

  EXPECT_FALSE(G.has_edge(1, 1));
  EXPECT_TRUE(G.has_edge(1, 3));
  EXPECT_TRUE(G.has_edge(3, 1));
  EXPECT_FALSE(G.has_edge(0, 2));

  EXPECT_EQ(G.degree(0), 4);
  EXPECT_EQ(G.degree(4), 4);

  // adjacency matrix
  EXPECT_EQ(create_adj(G), vector<vector<int>>({
                               {0, 1, 0, 1, 1, 1, 0},
                               {1, 0, 1, 1, 1, 1, 0},
                               {0, 1, 0, 1, 1, 1, 0},
                               {1, 1, 1, 0, 0, 0, 0},
                               {1, 1, 1, 0, 0, 0, 1},
                               {1, 1, 1, 0, 0, 0, 1},
                               {0, 0, 0, 0, 1, 1, 0},
                           }));
}

TEST(GraphTest, InduceAndRelabel) {
  auto G = Graph(7, edges);

  EXPECT_EQ(G.induce_and_relabel({}).number_of_edges(), 0);
  EXPECT_EQ(G.induce_and_relabel({1}).number_of_edges(), 0);
  EXPECT_EQ(G.induce_and_relabel({4, 5}).number_of_edges(), 0);
  EXPECT_EQ(G.induce_and_relabel({5, 6}).number_of_edges(), 1);
  EXPECT_EQ(create_adj(G.induce_and_relabel({6, 4, 1, 3})), vector<vector<int>>({
                                                                {0, 1, 0, 0},
                                                                {1, 0, 1, 0},
                                                                {0, 1, 0, 1},
                                                                {0, 0, 1, 0},
                                                            }));
}

TEST(GraphTest, InduceAndRelabelWithAPSD) {
  auto G = Graph(7, edges);
  G.compute_all_pairs_symmetric_differences();
  auto H = G.induce_and_relabel({6, 3, 0, 2, 1});
  EXPECT_EQ(util::sorted(H.get_symmetric_difference(0, 1)), VI({2, 3, 4}));
  EXPECT_EQ(util::sorted(H.get_symmetric_difference(3, 1)), VI({2}));
  EXPECT_EQ(util::sorted(H.get_symmetric_difference(3, 2)), VI({}));
}

TEST(GraphTest, Complement) {
  auto G = Graph(7, edges);

  EXPECT_EQ(create_adj(G.complement()), vector<vector<int>>({
                                            {0, 0, 1, 0, 0, 0, 1},
                                            {0, 0, 0, 0, 0, 0, 1},
                                            {1, 0, 0, 0, 0, 0, 1},
                                            {0, 0, 0, 0, 1, 1, 1},
                                            {0, 0, 0, 1, 0, 1, 0},
                                            {0, 0, 0, 1, 1, 0, 0},
                                            {1, 1, 1, 1, 0, 0, 0},
                                        }));

  EXPECT_EQ(Graph(63, {}).complement().number_of_edges(), 63 * 62 / 2);
  EXPECT_EQ(Graph(64, {}).complement().number_of_edges(), 64 * 63 / 2);
  EXPECT_EQ(Graph(65, {}).complement().number_of_edges(), 65 * 64 / 2);
  EXPECT_EQ(Graph(127, {}).complement().number_of_edges(), 127 * 126 / 2);
  EXPECT_EQ(Graph(128, {}).complement().number_of_edges(), 128 * 127 / 2);
  EXPECT_EQ(Graph(129, {}).complement().number_of_edges(), 129 * 128 / 2);

  EXPECT_EQ(Graph(63, {}).complement().complement().number_of_edges(), 0);
  EXPECT_EQ(Graph(64, {}).complement().complement().number_of_edges(), 0);
  EXPECT_EQ(Graph(65, {}).complement().complement().number_of_edges(), 0);
  EXPECT_EQ(Graph(127, {}).complement().complement().number_of_edges(), 0);
  EXPECT_EQ(Graph(128, {}).complement().complement().number_of_edges(), 0);
  EXPECT_EQ(Graph(129, {}).complement().complement().number_of_edges(), 0);
}

TEST(GraphTest, ComputeAllPairsSymmetricDifferences) {
  util::set_log_level(util::logging::NONE);

  auto G1 = Graph(5, {{0, 2}, {0, 3}, {0, 4}, {1, 3}, {1, 4}, {2, 4}, {3, 4}});
  G1.compute_all_pairs_symmetric_differences();
  EXPECT_EQ(G1.get_symmetric_difference(0, 1), std::vector<int>({2}));
  EXPECT_EQ(G1.get_symmetric_difference(0, 2), std::vector<int>({3}));
  EXPECT_EQ(G1.get_symmetric_difference(0, 3), std::vector<int>({1, 2}));
  EXPECT_EQ(G1.get_symmetric_difference(0, 4), std::vector<int>({1}));
  EXPECT_EQ(G1.get_symmetric_difference(1, 0), std::vector<int>({2}));
  EXPECT_EQ(G1.get_symmetric_difference(1, 2), std::vector<int>({0, 3}));
  EXPECT_EQ(G1.get_symmetric_difference(1, 3), std::vector<int>({0}));
  EXPECT_EQ(G1.get_symmetric_difference(1, 4), std::vector<int>({0, 2}));
  EXPECT_EQ(G1.get_symmetric_difference(2, 0), std::vector<int>({3}));
  EXPECT_EQ(G1.get_symmetric_difference(2, 1), std::vector<int>({0, 3}));
  EXPECT_EQ(G1.get_symmetric_difference(2, 3), std::vector<int>({1}));
  EXPECT_EQ(G1.get_symmetric_difference(2, 4), std::vector<int>({1, 3}));
  EXPECT_EQ(G1.get_symmetric_difference(3, 0), std::vector<int>({1, 2}));
  EXPECT_EQ(G1.get_symmetric_difference(3, 1), std::vector<int>({0}));
  EXPECT_EQ(G1.get_symmetric_difference(3, 2), std::vector<int>({1}));
  EXPECT_EQ(G1.get_symmetric_difference(3, 4), std::vector<int>({2}));
  EXPECT_EQ(G1.get_symmetric_difference(4, 0), std::vector<int>({1}));
  EXPECT_EQ(G1.get_symmetric_difference(4, 1), std::vector<int>({0, 2}));
  EXPECT_EQ(G1.get_symmetric_difference(4, 2), std::vector<int>({1, 3}));
  EXPECT_EQ(G1.get_symmetric_difference(4, 3), std::vector<int>({2}));

  auto G2 = Graph(4, {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}});
  G2.compute_all_pairs_symmetric_differences();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (i != j) { EXPECT_EQ(G2.get_symmetric_difference(i, j), std::vector<int>()); }
    }
  }
}

TEST(GraphTest, RemoveLeaves) {
  auto G1 = Graph(12, {{1, 11}, {1, 0}, {6, 0}, {6, 5}, {0, 7}, {9, 5}, {5, 7}, {7, 3}, {3, 2}, {5, 4}, {7, 4}, {4, 8}, {8, 10}});
  auto ret1 = G1.remove_leaves();
  EXPECT_EQ(ret1.number_of_nodes(), 5);
  EXPECT_EQ(ret1.number_of_edges(), 6);

  util::Random rand(12345);
  auto ret2 = generators::random_tree(rand, 100).remove_leaves();
  EXPECT_EQ(ret2.number_of_nodes(), 1);
}

TEST(GraphTest, Smooth) {
  auto G1 = Graph(5, {{0, 1}, {0, 2}, {0, 3}, {2, 3}, {1, 4}, {4, 2}});
  auto ret1 = G1.smooth();
  EXPECT_EQ(ret1.number_of_nodes(), 4);
  EXPECT_EQ(ret1.number_of_edges(), 5);

  auto ret2 = ret1.smooth();
  EXPECT_EQ(ret1.number_of_nodes(), 4);
  EXPECT_EQ(ret1.number_of_edges(), 5);

  auto G3 = Graph(6, {{3, 2}, {2, 0}, {0, 1}, {1, 5}, {5, 4}, {4, 3}});  // C_6
  auto ret3 = G3.smooth();
  EXPECT_EQ(ret3.number_of_nodes(), 3);  // C_3
  EXPECT_EQ(ret3.number_of_edges(), 3);

  auto ret4 = ret3.smooth();
  EXPECT_EQ(ret4.number_of_nodes(), 3);
  EXPECT_EQ(ret4.number_of_edges(), 3);

  auto G5 = Graph(7, {{3, 2}, {2, 0}, {0, 1}, {1, 5}, {5, 4}, {4, 3}, {1, 6}});
  auto ret5 = G5.smooth();
  EXPECT_EQ(ret5.number_of_nodes(), 4);
  EXPECT_EQ(ret5.number_of_edges(), 4);

  auto G6 = Graph(
      13, {{6, 2}, {2, 9}, {9, 8}, {6, 3}, {3, 1}, {1, 7}, {7, 8}, {6, 10}, {10, 5}, {5, 8}, {6, 0}, {0, 4}, {4, 11}, {11, 12}, {12, 8}});
  auto ret6 = G6.smooth();
  EXPECT_EQ(ret6.number_of_nodes(), 5);
  EXPECT_EQ(ret6.number_of_edges(), 7);

  auto G7 =
      Graph(11, {{4, 5}, {4, 7}, {5, 7}, {5, 6}, {6, 7}, {8, 9}, {9, 10}, {10, 8}, {7, 3}, {3, 1}, {1, 0}, {0, 2}, {2, 8}});
  auto ret7 = G7.smooth();
  EXPECT_EQ(ret7.number_of_nodes(), 7);
  EXPECT_EQ(ret7.number_of_edges(), 9);

  auto ret8 = generators::path_graph(10).smooth();
  EXPECT_EQ(ret8.number_of_nodes(), 2);
  EXPECT_EQ(ret8.number_of_edges(), 1);

  auto G9 = Graph(9, {{0, 1}, {1, 2}, {2, 0}, {1, 3}, {1, 4}, {1, 5}, {2, 6}, {2, 7}, {2, 8}});
  auto ret9 = G9.smooth();
  EXPECT_EQ(ret9.number_of_nodes(), 9);
  EXPECT_EQ(ret9.number_of_edges(), 9);

  auto G10 = Graph(9, {{0, 1}, {2, 0}, {1, 3}, {1, 4}, {1, 5}, {2, 6}, {2, 7}, {2, 8}});
  auto ret10 = G10.smooth();
  EXPECT_EQ(ret10.number_of_nodes(), 8);
  EXPECT_EQ(ret10.number_of_edges(), 7);

  auto G11 = Graph(5, {{0, 1}, {0, 2}, {1, 3}, {2, 3}, {3, 4}});
  auto ret11 = G11.smooth();
  EXPECT_EQ(ret11.number_of_nodes(), 4);
  EXPECT_EQ(ret11.number_of_edges(), 4);

  auto G12 = Graph(
      8, {{1, 2}, {1, 4}, {5, 2}, {5, 4}, {7, 2}, {7, 4}, {6, 2}, {6, 4}, {0, 2}, {0, 4}, {3, 2}, {3, 4}, {1, 5}, {5, 7}, {6, 0}, {0, 3}});
  auto ret12 = G12.smooth();
  EXPECT_EQ(ret12.number_of_nodes(), 8);
  EXPECT_EQ(ret12.number_of_edges(), 16);
}
