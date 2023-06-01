#include <gtest/gtest.h>

#include "algorithms/lowerbound/LBGreedy.hpp"
#include "generators/random.hpp"
#include "generators/tree.hpp"
#include "readwrite/pace_2023.hpp"
#include "util/Random.hpp"
#include "util/util.hpp"

using namespace std;
using namespace ds::graph;
using namespace algorithms::lowerbound;
using namespace algorithms::base;

//
// LBGreedyTest
//
TEST(LBGreedyTest, Run) {
  util::set_log_level(util::logging::NONE);

  util::Random rand(12345);
  auto g = generators::path_graph(4);
  g.compute_all_pairs_symmetric_differences();

  SolverInfo result;
  auto lb_greedy = LBGreedy(g, result);
  lb_greedy.run(rand);
  EXPECT_EQ(result.lower_bound(), 1);
}

TEST(LBGreedyTest, RunWithRandomInstances) {
  util::set_log_level(util::logging::NONE);

  util::Random rand(12345);
  int num_test_iterations = 10;
  int num_greedy_iterations = 20;
  std::vector<int> ns = {10, 50, 100};
  std::vector<double> ps = {0.1, 0.3, 0.5};

  for (int t = 0; t < num_test_iterations; ++t) {
    for (auto n : ns) {
      for (auto p : ps) {
        auto g = generators::erdos_renyi_graph(n, p, rand);
        // printf("n=%d, p=%.1f, edges=", n, p);
        // util::print(g.edges());
        g.compute_all_pairs_symmetric_differences();
        SolverInfo result;
        auto lbg = LBGreedy(g, result);
        lbg.run(rand, num_greedy_iterations);
        auto lb = result.lower_bound();
        EXPECT_GE(lb, 0);
  
        // int core_size = result.core_size();
        // if (core_size == 0) {
        //   EXPECT_EQ(lb, 0);  // not a prime graph
        // } else {
        //   auto h = g.induce_and_relabel(result.core());
        //   h.compute_all_pairs_symmetric_differences();
        //   for (int i = 0; i < h.number_of_nodes() - 1; ++i) {
        //     for (int j = i + 1; j < h.number_of_nodes(); ++j) {
        //       //
        //       EXPECT_GE(h.get_symmetric_difference_size(i, j), lb);
        //     }
        //   }
        // }
      }
    }
  }
}

// void verify_provided_instance(char const* path, int expect) {
//   util::Random rand(12345);
//   auto g = readwrite::load_pace_2023(path);
//   g.compute_all_pairs_symmetric_differences();
//   SolverInfo result;

//   auto lbg = LBGreedy(g, result);
//   int num_greedy_iterations = 20;
//   lbg.run(rand, num_greedy_iterations);
//   EXPECT_EQ(result.lower_bound(), expect);

//   auto h = g.induce_and_relabel(result.core());
//   // util::print(lbg.get_core());
//   h.compute_all_pairs_symmetric_differences();
//   for (int i = 0; i < h.number_of_nodes() - 1; ++i) {
//     for (int j = i + 1; j < h.number_of_nodes(); ++j) {
//       //
//       EXPECT_GE(h.get_symmetric_difference_size(i, j), expect);
//     }
//   }
// }

// TEST(LBGreedyTest, RunWithProvidedInstances) {
//   verify_provided_instance("data/tiny-set/tiny001.gr", 1);
//   verify_provided_instance("data/tiny-set/tiny002.gr", 2);
//   verify_provided_instance("data/tiny-set/tiny005.gr", 2);
//   // verify_provided_instance("data/exact-public/exact_006.gr", 5);
// }
