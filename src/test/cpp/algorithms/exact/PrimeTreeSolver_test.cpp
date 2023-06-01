#include <gtest/gtest.h>

#include "algorithms/exact/PrimeTreeSolver.hpp"
#include "ds/graph/TriGraph.hpp"
#include "generators/tree.hpp"
#include "util/Random.hpp"

using namespace std;
using namespace algorithms;
using namespace algorithms::exact;
using namespace ds::graph;

int const NUM_VERTICES = 100;
int const NUM_ITERATIONS = 100;

Graph random_prime_caterpillar(util::Random& rand, int n, double p) {
  std::vector<int> additional_leaves, labels;
  for (int i = 2; i < n - 2; ++i) {
    // attach random leaf; we don't add more than one leaves as it would make a tree non-prime
    if (rand.random() < p) additional_leaves.push_back(i);
  }
  int nn = n + static_cast<int>(additional_leaves.size());

  // randomize vertices
  for (int i = 0; i < nn; ++i) labels.push_back(i);
  rand.shuffle(labels);

  // construct edge list
  std::vector<std::pair<int, int>> edges;
  for (int i = 1; i < n; ++i) edges.push_back({labels[i - 1], labels[i]});
  for (int i = 0; i < nn - n; ++i) edges.push_back({labels[additional_leaves[i]], labels[n + i]});

  return Graph(nn, edges);
}

// void verify_tww(Graph const& g, int expected_tww_lb, int expected_tww_ub) {
void verify_tww(Graph const& g, int expected_tww) {
  util::set_log_level(util::logging::NONE);
  PrimeTreeSolver solver(g);
  solver.run();
  // EXPECT_GE(solver.twin_width(), expected_tww_lb);
  // EXPECT_LE(solver.twin_width(), expected_tww_ub);
  EXPECT_EQ(solver.twin_width(), expected_tww);

  // make sure the twin-width is consistent with the contraction seqneuce
  int actual_tww = verify_contraction_sequence(g, solver.contraction_sequence());
  EXPECT_EQ(actual_tww, solver.twin_width());
}

//
// PrimeTreeSolverTest
//
TEST(PrimeTreeSolverTest, IsCaterpillar) {
  // empty graph
  EXPECT_TRUE(PrimeTreeSolver(Graph(0)).is_caterpillar());

  // single node
  EXPECT_TRUE(PrimeTreeSolver(Graph(1)).is_caterpillar());

  // paths
  EXPECT_TRUE(PrimeTreeSolver(generators::path_graph(2)).is_caterpillar());
  EXPECT_TRUE(PrimeTreeSolver(generators::path_graph(3)).is_caterpillar());
  EXPECT_TRUE(PrimeTreeSolver(generators::path_graph(5)).is_caterpillar());
  EXPECT_TRUE(PrimeTreeSolver(generators::path_graph(10)).is_caterpillar());
  EXPECT_TRUE(PrimeTreeSolver(generators::path_graph(1000)).is_caterpillar());

  // smallest non-caterpillar
  EXPECT_FALSE(PrimeTreeSolver(Graph(7, {{0, 1}, {1, 2}, {0, 3}, {3, 4}, {0, 5}, {5, 6}})).is_caterpillar());

  // full r-ary trees
  EXPECT_FALSE(PrimeTreeSolver(generators::full_rary_tree(2, 1000)).is_caterpillar());
  EXPECT_FALSE(PrimeTreeSolver(generators::full_rary_tree(3, 1000)).is_caterpillar());
  EXPECT_FALSE(PrimeTreeSolver(generators::full_rary_tree(5, 1000)).is_caterpillar());
  EXPECT_FALSE(PrimeTreeSolver(generators::full_rary_tree(10, 1000)).is_caterpillar());
}

TEST(PrimeTreeSolverTest, IsCaterpillarPathWithRandomAdditions) {
  auto rand = util::Random(0xDEADBEEF);

  for (int t = 0; t < NUM_ITERATIONS; ++t) {
    auto g = random_prime_caterpillar(rand, NUM_VERTICES, 0.5);
    EXPECT_TRUE(PrimeTreeSolver(g).is_caterpillar());
  }
}

TEST(PrimeTreeSolverTest, RunWithRandomCaterpillar) {
  auto rand = util::Random(0xDEADBEEF);
  std::vector<int> ns = {4, 5, 10, 100};

  for (int t = 0; t < NUM_ITERATIONS; ++t) {
    for (auto n : ns) verify_tww(random_prime_caterpillar(rand, n, 0.5), 1);
  }
}

TEST(PrimeTreeSolverTest, RunWithSmallNonCaterpillar) {
  verify_tww(Graph(7, {{0, 1}, {1, 2}, {0, 3}, {3, 4}, {0, 5}, {5, 6}}), 2);
}
