#include <gtest/gtest.h>

#include "ds/graph/Graph.hpp"
#include "ds/graph/TriGraph.hpp"
#include "generators/named.hpp"
#include "generators/random.hpp"
#include "generators/tree.hpp"
#include "readwrite/pace_2023.hpp"

using namespace std;
using namespace ds::graph;

template <typename T>
void test_contract_1(T G) {
  G.add_edge(1, 2, false);
  G.add_edge(2, 3, true);
  EXPECT_EQ(G.red_degree(0), 0);
  EXPECT_EQ(G.red_degree(1), 0);
  EXPECT_EQ(G.red_degree(2), 1);
  EXPECT_EQ(G.red_degree(3), 1);
  EXPECT_EQ(T(G).contract(1, 3), 1);
  EXPECT_EQ(T(G).contract(3, 1), 1);
  EXPECT_EQ(T(G).contract(0, 1), 2);
  EXPECT_EQ(T(G).contract(1, 0), 2);
}

template <typename T>
void test_contract_2(T G) {
  G.add_edge(0, 2, false);
  G.add_edge(2, 3, true);
  G.add_edge(3, 4, true);
  G.add_edge(4, 1, true);
  EXPECT_EQ(G.red_degree(0), 0);
  EXPECT_EQ(G.red_degree(1), 1);
  EXPECT_EQ(G.red_degree(2), 1);
  EXPECT_EQ(G.red_degree(3), 2);
  EXPECT_EQ(G.red_degree(4), 2);
  EXPECT_EQ(T(G).contract(2, 4), 3);
  EXPECT_EQ(T(G).contract(4, 2), 3);
}

//
// TriGraphTest
//
TEST(TriGraphTest, Contract) {
  util::set_log_level(util::logging::NONE);

  auto H1 = Graph(4);
  auto H2 = Graph(5);
  RUN_WITH_TRIGRAPH(H1, test_contract_1);
  RUN_WITH_TRIGRAPH(H2, test_contract_2);
}

TEST(TriGraphTest, VerifyContractionSequence) {
  auto empty0 = Graph(0);
  auto empty1 = Graph(1);
  auto empty2 = Graph(2);
  auto path2 = Graph(2, {{0, 1}});
  auto path3 = Graph(3, {{0, 1}, {1, 2}});
  EXPECT_EQ(verify_contraction_sequence(empty0, {}), 0);
  EXPECT_EQ(verify_contraction_sequence(empty1, {}), 0);
  EXPECT_EQ(verify_contraction_sequence(empty2, {}), -2);
  EXPECT_EQ(verify_contraction_sequence(empty2, {{0, 2}}), -3);
  EXPECT_EQ(verify_contraction_sequence(empty2, {{0, 1}, {0, 1}}), -2);
  EXPECT_EQ(verify_contraction_sequence(empty2, {{0, 1}}), 0);
  EXPECT_EQ(verify_contraction_sequence(path2, {{0, 1}}), 0);
  EXPECT_EQ(verify_contraction_sequence(path3, {{0, 1}, {0, 2}}), 1);
  EXPECT_EQ(verify_contraction_sequence(path3, {{0, 2}, {0, 1}}), 0);
  EXPECT_EQ(verify_contraction_sequence(path3, {{0, 1}, {1, 2}}), -3);
  EXPECT_EQ(verify_contraction_sequence(path3, {{1, 0}, {2, 0}}), -3);

  auto g = Graph(5, {{1, 2}, {1, 3}});
  EXPECT_EQ(verify_contraction_sequence(g, {{1, 4}, {2, 1}, {0, 3}, {0, 2}}), 2);
}

template <typename T>
void test_check_consistency(T G, std::vector<std::pair<int, int>> const& seq) {
  ContractionInfo ci(G.number_of_nodes());
  G.compute_greedy_criteria(10);
  for (auto& p : seq) {
    G.contract_verbose(p.first, p.second, ci);
    G.check_consistency();
  }
}

TEST(TriGraphTest, CheckConsistency) {
  auto g = Graph(
      34, {{0, 1},   {0, 17},  {0, 24},  {0, 20},  {0, 2},   {0, 12},  {0, 3},   {0, 15},  {0, 26},  {0, 18},  {0, 13},
           {1, 2},   {1, 3},   {2, 3},   {2, 31},  {2, 16},  {3, 4},   {3, 5},   {3, 6},   {3, 24},  {3, 20},  {4, 5},
           {4, 6},   {4, 7},   {5, 6},   {5, 8},   {5, 9},   {5, 10},  {5, 11},  {5, 12},  {5, 13},  {6, 8},   {6, 14},
           {6, 15},  {6, 16},  {8, 10},  {8, 15},  {8, 13},  {8, 16},  {9, 11},  {9, 17},  {9, 25},  {9, 10},  {9, 20},
           {9, 12},  {9, 23},  {9, 16},  {10, 25}, {10, 20}, {10, 12}, {10, 15}, {10, 13}, {11, 12}, {12, 25}, {12, 20},
           {12, 23}, {12, 15}, {12, 26}, {12, 16}, {13, 19}, {13, 27}, {13, 26}, {14, 29}, {15, 22}, {15, 20}, {15, 23},
           {15, 26}, {15, 16}, {16, 30}, {16, 31}, {16, 23}, {16, 33}, {17, 18}, {20, 21}, {20, 25}, {20, 26}, {22, 23},
           {23, 26}, {26, 27}, {26, 29}, {26, 32}, {28, 29}, {30, 31}});
  g.compute_all_pairs_symmetric_differences();

  std::vector<std::pair<int, int>> seq1 = {{24, 1},  {33, 30}, {19, 7},  {18, 17}, {33, 31}, {28, 14},
                                           {32, 27}, {32, 21}, {24, 18}, {29, 28}, {33, 2},  {29, 19},
                                           {25, 11}, {33, 24}, {23, 22}, {29, 4},  {8, 6},   {25, 10},
                                           {15, 12}, {26, 0},  {33, 23}, {15, 9},  {33, 16}};
  std::vector<std::pair<int, int>> seq2 = {{28, 14}, {31, 30}, {18, 17}, {21, 19}, {33, 31}, {25, 11}, {29, 27}};

  RUN_WITH_TRIGRAPH(g, test_check_consistency, seq1);
  RUN_WITH_TRIGRAPH(g, test_check_consistency, seq2);
}

template <typename T>
void test_weak_red_potential_1(T G) {
  ContractionInfo ci(G.number_of_nodes());
  G.compute_greedy_criteria(5);
  G.check_consistency();

  EXPECT_EQ(G.contract_verbose(3, 2, ci), 2);
  G.check_consistency();
  EXPECT_EQ(ci.last_merge, 3);
  EXPECT_EQ(ci.last_merged, 2);
  EXPECT_EQ(G.weak_red_potential(0, 1), 1);
  EXPECT_EQ(G.weak_red_potential(1, 0), 1);
}

template <typename T>
void test_weak_red_potential_2(T G) {
  ContractionInfo ci(G.number_of_nodes());
  G.compute_greedy_criteria(5);
  G.check_consistency();

  EXPECT_EQ(G.contract_verbose(2, 3, ci), 2);
  G.check_consistency();
  EXPECT_EQ(ci.last_merge, 2);
  EXPECT_EQ(ci.last_merged, 3);
  EXPECT_EQ(G.weak_red_potential(0, 1), 1);
  EXPECT_EQ(G.weak_red_potential(1, 0), 1);
}

template <typename T>
void test_weak_red_potential_3(T G) {
  ContractionInfo ci(G.number_of_nodes());
  G.compute_greedy_criteria(5);
  G.check_consistency();

  EXPECT_EQ(G.contract_verbose(1, 4, ci), 1);
  G.check_consistency();
  EXPECT_EQ(G.weak_red_potential(2, 3), 0);
  EXPECT_EQ(G.weak_red_potential(3, 2), 0);
}

template <typename T>
void test_weak_red_potential_4(T G) {
  ContractionInfo ci(G.number_of_nodes());
  G.compute_greedy_criteria(3);
  G.check_consistency();

  EXPECT_EQ(G.contract_verbose(2, 6, ci), 3);
  G.check_consistency();

  EXPECT_EQ(G.red_cap_reached().to_vector(), std::vector<int>({2}));
  EXPECT_EQ(G.weak_red_potential(4, 5), 1);
  EXPECT_EQ(G.weak_red_potential(5, 4), 1);
}

TEST(TriGraphTest, WeakRedPotential) {
  auto g1 = Graph(5, {{0, 1}, {1, 2}, {0, 2}, {2, 3}});
  auto g2 = Graph(5, {{0, 2}, {0, 3}, {0, 1}, {1, 4}});
  auto g3 = Graph(7, {{0, 2}, {1, 2}, {3, 2}, {2, 5}, {5, 6}});
  g1.compute_all_pairs_symmetric_differences();
  g2.compute_all_pairs_symmetric_differences();
  g3.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g1, test_weak_red_potential_1);
  RUN_WITH_TRIGRAPH(g1, test_weak_red_potential_2);
  RUN_WITH_TRIGRAPH(g2, test_weak_red_potential_3);
  RUN_WITH_TRIGRAPH(g3, test_weak_red_potential_4);
}

template <typename T>
void test_contract_verbose_1(T G) {
  int red_cap = 2;
  ContractionInfo ci(G.number_of_nodes());
  G.compute_greedy_criteria(red_cap);
  G.check_consistency();

  EXPECT_EQ(G.contract_verbose(5, 3, ci), 2);
  G.check_consistency();
  EXPECT_EQ(G.red_cap_reached().to_vector(), std::vector<int>({5}));

  EXPECT_EQ(G.contract_verbose(6, 1, ci), 2);
  G.check_consistency();
  EXPECT_EQ(G.red_cap_reached().to_vector(), std::vector<int>({4, 5}));

  EXPECT_EQ(G.contract_verbose(2, 4, ci), 3);
  G.check_consistency();
  EXPECT_EQ(G.red_cap_reached().to_vector(), std::vector<int>({2}));
}

template <typename T>
void test_contract_verbose_2(T G) {
  int red_cap = 3;
  ContractionInfo ci(G.number_of_nodes());
  G.compute_greedy_criteria(red_cap);
  G.check_consistency();

  EXPECT_EQ(G.contract_verbose(1, 6, ci), 1);
  G.check_consistency();

  EXPECT_EQ(G.contract_verbose(3, 5, ci), 2);
  G.check_consistency();

  EXPECT_EQ(G.contract_verbose(4, 2, ci), 3);
  G.check_consistency();

  std::vector<std::pair<int, int>> expect_upd = {{0, 1}, {0, 3}, {0, 4}};

  EXPECT_EQ(G.red_cap_reached().to_vector(), std::vector<int>({4}));
  EXPECT_EQ(ci.common_neighbors, std::vector<int>({3}));
  EXPECT_EQ(ci.updated_pairs, expect_upd);
}

template <typename T>
void test_contract_verbose_3(T G, Graph const& g) {
  G.compute_greedy_criteria(5);
  ContractionInfo ci(G.number_of_nodes());
  EXPECT_EQ(G.contract_verbose(3, 4, ci), 1);
  G.check_consistency();
  EXPECT_EQ(G.number_of_nodes(), 4);
  EXPECT_TRUE(G.has_vertex(3));
  EXPECT_FALSE(G.has_vertex(4));

  EXPECT_EQ(G.contract_verbose(1, 3, ci), 1);
  G.check_consistency();
  EXPECT_EQ(G.number_of_nodes(), 3);

  T G2(g);
  G2.compute_greedy_criteria(5);
  EXPECT_EQ(G2.contract_verbose(3, 4, ci), 1);
  EXPECT_EQ(G2.contract_verbose(3, 1, ci), 1);
  G2.check_consistency();

  T G3(g);
  G3.compute_greedy_criteria(5);
  EXPECT_EQ(G3.contract_verbose(3, 4, ci), 1);
  EXPECT_EQ(G3.contract_verbose(0, 1, ci), 2);
  G3.check_consistency();

  T G4(g);
  G4.compute_greedy_criteria(5);
  EXPECT_EQ(G4.contract_verbose(3, 4, ci), 1);
  EXPECT_EQ(G4.contract_verbose(1, 0, ci), 2);
  G4.check_consistency();
}

TEST(TriGraphTest, ContractVerbose) {
  auto g1 = Graph(7, {{0, 2}, {2, 3}, {3, 4}, {4, 1}, {5, 3}, {6, 1}});
  auto g2 = Graph(5, {{1, 2}, {2, 3}, {3, 4}});
  g1.compute_all_pairs_symmetric_differences();
  g2.compute_all_pairs_symmetric_differences();

  RUN_WITH_TRIGRAPH(g1, test_contract_verbose_1);
  RUN_WITH_TRIGRAPH(g1, test_contract_verbose_2);
  RUN_WITH_TRIGRAPH(g2, test_contract_verbose_3, g2);
}

template <typename T>
void test_rollback_1(T G) {
  G.compute_greedy_criteria(2);

  int n = G.number_of_nodes();
  ContractionHistory hist1(n), hist2(n), hist3(n);
  ContractionInfo info(n);

  auto h0 = G;

  G.contract_verbose(1, 3, info, &hist1);
  auto h1 = G;
  G.contract_verbose(1, 0, info, &hist2);
  auto h2 = G;
  G.contract_verbose(4, 1, info, &hist3);

  G.rollback_history(hist3);
  G.check_consistency();
  EXPECT_EQ(G, h2);

  G.rollback_history(hist2);
  G.check_consistency();
  EXPECT_EQ(G, h1);

  G.rollback_history(hist1);
  G.check_consistency();
  EXPECT_EQ(G, h0);
}

TEST(TriGraphTest, Rollback1) {
  util::Random rand(12345);

  for (auto pr : vector<double>({0.2, 0.5})) {
    for (auto n : vector<int>({5, 10, 30})) {
      for (int t = 0; t < 20; ++t) {
        // create a random graph
        auto g = generators::erdos_renyi_graph(n, pr, rand);
        g.compute_all_pairs_symmetric_differences();

        RUN_WITH_TRIGRAPH(g, test_rollback_1);
      }
    }
  }
}

template <typename T>
void test_rollback_2(T G) {
  G.compute_greedy_criteria(2);

  int n = G.number_of_nodes();
  ContractionHistory hist1(n), hist2(n), hist3(n), hist4(n);
  ContractionInfo info(n);

  auto h0 = G;
  G.contract_verbose(8, 6, info, &hist1);

  auto h1 = G;
  G.contract_verbose(9, 7, info, &hist2);
  G.rollback_history(hist2);
  EXPECT_EQ(G, h1);

  G.contract_verbose(11, 7, info, &hist3);
  G.rollback_history(hist3);
  EXPECT_EQ(G, h1);

  EXPECT_EQ(G.weak_red_potential(9, 11), 2);

  G.contract_verbose(11, 9, info, &hist4);
  G.rollback_history(hist4);
  EXPECT_EQ(G, h1);

  G.rollback_history(hist1);
  EXPECT_EQ(G, h0);
}

TEST(TriGraphTest, Rollback2) {
  auto g = generators::durer_graph();
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, test_rollback_2);
}

template <typename T>
void test_hash_1(T G) {
  T h(G);
  h.compute_greedy_criteria(2);

  ContractionInfo info(h.number_of_nodes());
  ContractionHistory history(h.number_of_nodes());

  EXPECT_EQ(G.hash(), 0ULL);
  G.contract(10, 2);
  auto h11 = G.hash();
  G.contract(6, 8);
  auto h12 = G.hash();

  h.contract_verbose(6, 8, info);
  auto h21 = h.hash();
  h.contract_verbose(10, 2, info, &history);
  auto h22 = h.hash();
  h.rollback_history(history);
  auto h23 = h.hash();

  EXPECT_NE(h11, 0ULL);
  EXPECT_NE(h12, 0ULL);
  EXPECT_NE(h21, 0ULL);
  EXPECT_NE(h22, 0ULL);
  EXPECT_NE(h11, h12);
  EXPECT_NE(h11, h21);
  EXPECT_NE(h21, h22);
  EXPECT_EQ(h12, h22);  // different contraction order but ends up with the same trigraph
  EXPECT_EQ(h21, h23);  // rollback must recover the original hash
}

TEST(TriGraphTest, Hash1) {
  auto g = generators::durer_graph();
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, test_hash_1);
}

TEST(TriGraphTest, Hash2) {
  auto gg = generators::path_graph(4000);
  gg.compute_all_pairs_symmetric_differences();

  ds::graph::TriGraph<ds::ArrayBitset12> g(gg);
  ds::graph::TriGraph<ds::ArrayBitset12> h(g);
  h.compute_greedy_criteria(2);

  EXPECT_EQ(h.hash(), 0ULL);
  h.add_edge(0, 1024, true);
  EXPECT_NE(h.hash(), 0ULL);

  h.contract(3998, 3997);
  h.contract(1023, 1025);

  g.contract(1023, 1025);
  g.contract(3998, 3997);
  g.add_edge(1024, 0, true);

  EXPECT_NE(h.hash(), 0ULL);
  EXPECT_EQ(h.hash(), g.hash());
}

template <typename T>
void test_is_free_contraction(T G) {
  G.compute_greedy_criteria(3);
  EXPECT_FALSE(G.is_free_contraction(0, 1));
  EXPECT_FALSE(G.is_free_contraction(3, 4));

  G.make_edge_red(1, 2);
  G.make_edge_red(3, 4);
  EXPECT_TRUE(G.is_free_contraction(0, 1));
  EXPECT_TRUE(G.is_free_contraction(1, 0));
  EXPECT_FALSE(G.is_free_contraction(1, 3));

  G.remove_edge(0, 1);
  EXPECT_FALSE(G.is_free_contraction(1, 3));
  EXPECT_FALSE(G.is_free_contraction(3, 4));

  G.make_edge_black(1, 2);
  G.make_edge_red(2, 3);
  EXPECT_TRUE(G.is_free_contraction(1, 3));
  EXPECT_TRUE(G.is_free_contraction(3, 1));

  G.make_edge_black(2, 3);
  EXPECT_TRUE(G.is_free_contraction(1, 3));
  EXPECT_TRUE(G.is_free_contraction(3, 1));
  EXPECT_FALSE(G.is_free_contraction(2, 3));
  EXPECT_FALSE(G.is_free_contraction(3, 4));
}

TEST(TriGraphTest, IsFreeContraction) {
  auto g = generators::path_graph(5);
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, test_is_free_contraction);
}

template <typename T>
void test_copy_constructor(T g) {
  g.compute_greedy_criteria();
  g.remove_vertex(7);
  g.contract(3, 9);
  g.recompute_greedy_criteria();

  for (int t = 0; t < 10; ++t) {
    T h(g);
    h.remove_vertex(5);
    h.remove_vertex(1);
    h.remove_vertex(8);
    h.remove_vertex(3);
    h.recompute_greedy_criteria();
    h.check_consistency();
    g.check_consistency();
  }
}

TEST(TriGraphTest, CopyConstructor) {
  auto g = readwrite::load_pace_2023("data/tiny-set/tiny005.gr");
  g.compute_all_pairs_symmetric_differences();
  RUN_WITH_TRIGRAPH(g, test_copy_constructor);
}
