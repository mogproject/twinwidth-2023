#include <gtest/gtest.h>

#include "algorithms/ExactSolver.hpp"
#include "ds/graph/TriGraph.hpp"
#include "generators/random.hpp"
#include "generators/tree.hpp"
#include "util/Random.hpp"

using namespace std;
using namespace algorithms;
using namespace ds::graph;

typedef std::vector<int> VI;

static void verify_tww(Graph const& g, util::Random& rand, int expected_tww_lb, int expected_tww_ub) {
  util::set_log_level(util::logging::NONE);
  ExactSolver solver(g);
  solver.run(rand);
  EXPECT_GE(solver.twin_width(), expected_tww_lb);
  EXPECT_LE(solver.twin_width(), expected_tww_ub);

  // make sure the twin-width is consistent with the contraction seqneuce
  int actual_tww = verify_contraction_sequence(g, solver.contraction_sequence());
  EXPECT_EQ(actual_tww, solver.twin_width());
}

static std::vector<int> flatten(std::vector<std::vector<int>> xss) {
  std::vector<int> ret;
  for (auto& xs : xss) {
    for (auto x : xs) ret.push_back(x);
  }
  return ret;
}

static void join_vertices(Graph& g, std::vector<int> const& a, std::vector<int> const& b) {
  for (auto x : a) {
    for (auto y : b) {
      // full join
      g.add_edge(x, y);
    }
  }
}

TEST(ExactSolverTest, RunWithRAryTrees) {
  util::Random rand(12345);
  int n = 1000;
  std::vector<int> rs = {2, 3, 5, 10};
  for (auto r : rs) verify_tww(generators::full_rary_tree(r, n), rand, 2, 2);
}

TEST(ExactSolverTest, RunWithRandomTrees) {
  util::Random rand(12345);
  int num_iterations = 100;
  std::vector<int> ns = {4, 10, 20, 100};

  for (int t = 0; t < num_iterations; ++t) {
    for (int n : ns) {
      auto g = generators::random_tree(rand, n);
      verify_tww(g, rand, 0, 2);
    }
  }
}

TEST(ExactSolverTest, RunWithSmallInstances) {
  util::Random rand(12345);

  // graph including many prime graphs
  auto g1 = Graph(73);
  std::vector<std::vector<VI>> xsss = {
      {{0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}, {12, 13, 14, 15}},
      {{16, 17, 18, 19}, {20, 21, 22, 23}, {24, 25, 26, 27}, {28, 29, 30, 31, 72}},
      {{32, 33, 34, 35}, {36, 37, 38, 39}, {40, 41, 42, 43}, {44, 45, 46, 47}, {48, 49, 50, 51}, {52, 53, 54, 55}},
      {{56, 57, 58, 59}, {60, 61, 62, 63}, {64, 65, 66, 67}, {68, 69, 70, 71}},
  };
  for (auto& xss : xsss) {
    for (auto& xs : xss) {
      for (std::size_t i = 0; i < xs.size() - 1; ++i) g1.add_edge(xs[i], xs[i + 1]);  // create 18 paths
    }
    for (std::size_t i = 0; i < xss.size() - 1; ++i) join_vertices(g1, xss[i], xss[i + 1]);  // join paths
  }
  for (std::size_t i = 0; i < xsss.size() - 1; ++i) join_vertices(g1, flatten(xsss[i]), flatten(xsss[i + 1]));

  verify_tww(g1, rand, 1, 1);
}

// static bool is_prime(Graph const& g) {
//   modular::MDTree mdtree(g);
//   return mdtree.modular_width() == g.number_of_nodes();
// }

// TEST(ExactSolverTest, SmootingConjecture) {
//   util::Random rand(12345);
//   int num_iterations = 10000;
//   std::vector<int> ns = {10, 20, 30};

//   for (int n : ns) {
//     for (int t = 0; t < num_iterations; ++t) {
//       auto g = generators::erdos_renyi_graph(n, 0.5, rand);
//       if (!g.has_edge(0, 1)) g.add_edge(0, 1);

//       std::vector<std::pair<int, int>> h_edges;
//       for (auto& e : g.edges()) {
//         if (e.first + e.second != 1) h_edges.push_back(e);
//       }
//       h_edges.push_back({0, n});  // subdivide edge 0-1
//       h_edges.push_back({1, n});

//       auto h = Graph(n + 1, h_edges);

//       ExactSolver solver1(g);
//       ExactSolver solver2(h);
//       solver1.run(rand);
//       solver2.run(rand);

//       if (solver1.twin_width() > solver2.twin_width()) {
//         printf("g prime=%s: ", is_prime(g) ? "True" : "False");
//         util::print(g.edges());

//         printf("h prime=%s: ", is_prime(h) ? "True" : "False");
//         util::print(h.edges());
//       }
//       EXPECT_LE(solver1.twin_width(), solver2.twin_width());
//     }
//   }
// }
