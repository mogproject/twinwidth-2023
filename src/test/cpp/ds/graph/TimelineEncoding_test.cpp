#include <gtest/gtest.h>

#include "ds/graph/TimelineEncoding.hpp"
#include "ds/graph/TriGraph.hpp"
#include "generators/named.hpp"
#include "generators/random.hpp"
#include "util/logger.hpp"

using namespace std;
using namespace ds::graph;

typedef std::vector<std::pair<int, int>> VII;

//
// TimelineEncodingTest
//
TEST(TimelineEncodingTest, Init) {
  util::set_log_level(util::logging::NONE);

  ds::graph::Graph g(5, {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 4}, {2, 3}, {2, 4}, {3, 4}});
  g.compute_all_pairs_symmetric_differences();

  VII seq1 = {{3, 0}, {1, 4}, {1, 3}, {2, 1}};
  auto te1 = TimelineEncoding(g, seq1);
  // EXPECT_EQ(te1.vmapping_, std::vector<int>({0, 3, 4, 2, 1}));
  // EXPECT_EQ(te1.vmapping_inv_, std::vector<int>({0, 4, 3, 1, 2}));
  // EXPECT_EQ(te1.parents_, std::vector<int>({2, 3, 3, 4, -1}));

  // EXPECT_TRUE(te1.red_edges_[0].empty());
  // EXPECT_EQ(te1.red_edges_[1].size(), 1);
  // EXPECT_EQ(te1.red_edges_[1][2], std::vector<int>({0}));
  // EXPECT_EQ(te1.red_edges_[2].size(), 1);
  // EXPECT_EQ(te1.red_edges_[2][3], std::vector<int>({1, 0, 1}));  // can be in a different order
  // EXPECT_TRUE(te1.red_edges_[3].empty());
  // EXPECT_TRUE(te1.red_edges_[4].empty());
  EXPECT_EQ(te1.twin_width(), 2);

  VII seq2 = {{4, 0}, {1, 3}, {1, 4}, {2, 1}};
  auto te2 = TimelineEncoding(g, seq2);
  // EXPECT_TRUE(te2.red_edges_[0].empty());
  // EXPECT_TRUE(te2.red_edges_[1].empty());
  // EXPECT_TRUE(te2.red_edges_[2].empty());
  // EXPECT_TRUE(te2.red_edges_[3].empty());
  // EXPECT_TRUE(te2.red_edges_[4].empty());
  EXPECT_EQ(te2.twin_width(), 0);
}

VII verify_update_parent(Graph const& g, VII const& seq, TimelineEncoding& te, int i, int p) {
  te.update_parent(i, p);
  VII new_seq;
  for (auto& s : seq) new_seq.push_back({i == s.second ? p : s.first, s.second});

  auto actual = te.twin_width();
  auto expect = verify_contraction_sequence(g, new_seq);
  EXPECT_EQ(actual, expect);
  EXPECT_EQ(actual, TimelineEncoding(g, new_seq).twin_width());
  EXPECT_EQ(te.contraction_sequence(), new_seq);
  return new_seq;
}

VII verify_update_permutation(Graph const& g, VII const& seq, TimelineEncoding& te, int u, int v) {
  te.update_permutation(u, v);
  auto f = [&](int x) { return x == u ? v : x == v ? u : x; };
  VII new_seq;
  for (auto& s : seq) new_seq.push_back({f(s.first), f(s.second)});

  auto actual = te.twin_width();
  auto expect = verify_contraction_sequence(g, new_seq);
  EXPECT_EQ(actual, expect);
  EXPECT_EQ(actual, TimelineEncoding(g, new_seq).twin_width());
  EXPECT_EQ(te.contraction_sequence(), new_seq);
  return new_seq;
}

TEST(TimelineEncodingTest, UpdateParentWithSmallInstances) {
  // from exact_002.gr
  Graph g(20, {{0, 1},   {0, 2},   {0, 3},   {0, 4},   {0, 5},   {1, 2},   {1, 6},   {1, 7},   {1, 8},   {1, 9},
               {2, 6},   {2, 3},   {2, 10},  {2, 11},  {2, 12},  {3, 6},   {3, 7},   {3, 10},  {3, 4},   {3, 16},
               {4, 10},  {4, 13},  {4, 8},   {4, 11},  {4, 16},  {4, 5},   {5, 16},  {5, 18},  {5, 19},  {5, 9},
               {5, 12},  {6, 7},   {6, 13},  {6, 14},  {6, 15},  {7, 10},  {7, 13},  {7, 8},   {7, 17},  {8, 13},
               {8, 11},  {8, 14},  {8, 17},  {8, 9},   {9, 17},  {9, 19},  {9, 12},  {9, 15},  {10, 13}, {10, 11},
               {10, 18}, {11, 14}, {11, 16}, {11, 18}, {11, 12}, {12, 18}, {12, 15}, {13, 14}, {13, 19}, {14, 16},
               {14, 17}, {14, 19}, {14, 15}, {15, 19}, {16, 17}, {16, 18}, {17, 18}, {17, 19}, {18, 19}});
  VII seq = {{19, 12}, {18, 16}, {15, 5},  {10, 3}, {7, 1},  {17, 9},  {8, 6},   {4, 2},   {13, 7}, {10, 0},
             {11, 10}, {19, 18}, {17, 14}, {19, 4}, {19, 8}, {19, 11}, {19, 13}, {19, 15}, {19, 17}};
  g.compute_all_pairs_symmetric_differences();

  TimelineEncoding te(g, seq);

  seq = verify_update_parent(g, seq, te, 5, 11);
  seq = verify_update_parent(g, seq, te, 5, 15);
  seq = verify_update_parent(g, seq, te, 5, 11);
  seq = verify_update_parent(g, seq, te, 2, 14);

  Graph g2(5, {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 4}, {2, 3}, {2, 4}, {3, 4}});
  VII seq2 = {{2, 0}, {3, 1}, {3, 2}, {4, 3}};
  g2.compute_all_pairs_symmetric_differences();

  TimelineEncoding te2(g2, seq2);
  EXPECT_EQ(te2.twin_width(), 1);
  verify_update_parent(g2, seq2, te2, 0, 4);
  EXPECT_EQ(te2.twin_width(), 0);

  VII seq3 = {{4, 0}, {4, 1}, {3, 2}, {4, 3}};
  TimelineEncoding te3(g2, seq3);
  EXPECT_EQ(te3.twin_width(), 1);
  verify_update_parent(g2, seq3, te3, 0, 2);
  EXPECT_EQ(te3.twin_width(), 2);
}

TEST(TimelineEncodingTest, UpdateParentWithRandomInstances) {
  util::Random rand(12345);

  for (auto pr : vector<double>({0.2, 0.5})) {
    for (auto n : vector<int>({5, 10, 30})) {
      for (int t = 0; t < 20; ++t) {
        // create a random graph
        auto g = generators::erdos_renyi_graph(n, pr, rand);
        g.compute_all_pairs_symmetric_differences();

        vector<int> xs;
        for (int i = 0; i < n; ++i) xs.push_back(i);
        rand.shuffle(xs);  // randomize ordering of n vertices

        VII seq;
        for (int i = 0; i < n - 1; ++i) seq.push_back({xs[rand.randint(i + 1, n - 1)], xs[i]});
        seq = normalize_contraction_sequence(seq);
        for (int i = 0; i < n; ++i) xs[i] = i == n - 1 ? n - 1 : seq[i].second;

        auto te = TimelineEncoding(g, seq);
        EXPECT_EQ(te.twin_width(), verify_contraction_sequence(g, seq));

        // make random changes
        for (int c = 0; c < 20; ++c) {
          int i = rand.randint(0, n - 2);
          int p = rand.randint(i + 1, n - 1);
          seq = verify_update_parent(g, seq, te, xs[i], xs[p]);
        }
      }
    }
  }
}

TEST(TimelineEncodingTest, UpdatePermutationWithSmallInstances) {
  // from exact_002.gr
  Graph g1(20, {{0, 1},   {0, 2},   {0, 3},   {0, 4},   {0, 5},   {1, 2},   {1, 6},   {1, 7},   {1, 8},   {1, 9},
                {2, 6},   {2, 3},   {2, 10},  {2, 11},  {2, 12},  {3, 6},   {3, 7},   {3, 10},  {3, 4},   {3, 16},
                {4, 10},  {4, 13},  {4, 8},   {4, 11},  {4, 16},  {4, 5},   {5, 16},  {5, 18},  {5, 19},  {5, 9},
                {5, 12},  {6, 7},   {6, 13},  {6, 14},  {6, 15},  {7, 10},  {7, 13},  {7, 8},   {7, 17},  {8, 13},
                {8, 11},  {8, 14},  {8, 17},  {8, 9},   {9, 17},  {9, 19},  {9, 12},  {9, 15},  {10, 13}, {10, 11},
                {10, 18}, {11, 14}, {11, 16}, {11, 18}, {11, 12}, {12, 18}, {12, 15}, {13, 14}, {13, 19}, {14, 16},
                {14, 17}, {14, 19}, {14, 15}, {15, 19}, {16, 17}, {16, 18}, {17, 18}, {17, 19}, {18, 19}});
  VII seq = {{19, 12}, {18, 16}, {15, 5},  {10, 3}, {7, 1},  {17, 9},  {8, 6},   {4, 2},   {13, 7}, {10, 0},
             {11, 10}, {19, 18}, {17, 14}, {19, 4}, {19, 8}, {19, 11}, {19, 13}, {19, 15}, {19, 17}};
  g1.compute_all_pairs_symmetric_differences();

  TimelineEncoding te(g1, seq);
  seq = verify_update_permutation(g1, seq, te, 0, 1);
  seq = verify_update_permutation(g1, seq, te, 1, 19);
  seq = verify_update_permutation(g1, seq, te, 10, 5);
  seq = verify_update_permutation(g1, seq, te, 4, 7);
  seq = verify_update_permutation(g1, seq, te, 5, 19);
  seq = verify_update_permutation(g1, seq, te, 19, 5);
  seq = verify_update_permutation(g1, seq, te, 2, 3);
  seq = verify_update_permutation(g1, seq, te, 2, 4);
  seq = verify_update_permutation(g1, seq, te, 2, 5);
  seq = verify_update_permutation(g1, seq, te, 2, 7);

  Graph g2(5, {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 4}, {2, 3}, {2, 4}, {3, 4}});
  VII seq2 = {{2, 0}, {3, 1}, {3, 2}, {4, 3}};
  g2.compute_all_pairs_symmetric_differences();

  TimelineEncoding te2(g2, seq2);
  seq2 = verify_update_permutation(g2, seq2, te2, 0, 1);
  seq2 = verify_update_permutation(g2, seq2, te2, 0, 1);
  seq2 = verify_update_permutation(g2, seq2, te2, 1, 3);
  seq2 = verify_update_permutation(g2, seq2, te2, 4, 3);
  seq2 = verify_update_permutation(g2, seq2, te2, 2, 3);
  seq2 = verify_update_permutation(g2, seq2, te2, 4, 3);
  seq2 = verify_update_permutation(g2, seq2, te2, 4, 3);
  seq2 = verify_update_permutation(g2, seq2, te2, 1, 4);
  seq2 = verify_update_permutation(g2, seq2, te2, 1, 2);
  seq2 = verify_update_permutation(g2, seq2, te2, 3, 2);
  seq2 = verify_update_permutation(g2, seq2, te2, 3, 4);
  seq2 = verify_update_permutation(g2, seq2, te2, 2, 4);
  seq2 = verify_update_permutation(g2, seq2, te2, 3, 2);
}

TEST(TimelineEncodingTest, UpdatePermutationWithRandomInstances) {
  util::Random rand(12345);

  for (auto pr : vector<double>({0.2, 0.5})) {
    for (auto n : vector<int>({5, 10, 30})) {
      for (int t = 0; t < 20; ++t) {
        // create a random graph
        auto g = generators::erdos_renyi_graph(n, pr, rand);
        g.compute_all_pairs_symmetric_differences();

        vector<int> xs;
        for (int i = 0; i < n; ++i) xs.push_back(i);
        rand.shuffle(xs);  // randomize ordering of n vertices

        VII seq;
        for (int i = 0; i < n - 1; ++i) seq.push_back({xs[rand.randint(i + 1, n - 1)], xs[i]});
        seq = normalize_contraction_sequence(seq);
        for (int i = 0; i < n; ++i) xs[i] = i == n - 1 ? n - 1 : seq[i].second;

        auto te = TimelineEncoding(g, seq);
        EXPECT_EQ(te.twin_width(), verify_contraction_sequence(g, seq));

        // make random changes
        for (int c = 0; c < 20; ++c) {
          int i = rand.randint(0, n - 1);
          int j = rand.randint(0, n - 1);
          seq = verify_update_permutation(g, seq, te, xs[i], xs[j]);

          // permutation twice -> same as no permutation
          int tww = te.twin_width();
          te.update_permutation(i, j);
          te.update_permutation(i, j);
          EXPECT_EQ(te.twin_width(), tww);
        }
      }
    }
  }
}
