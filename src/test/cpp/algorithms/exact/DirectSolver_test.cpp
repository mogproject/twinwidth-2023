#include <gtest/gtest.h>

#include "algorithms/exact/DirectSolver.hpp"
#include "ds/graph/TriGraph.hpp"
#include "generators/named.hpp"
#include "generators/tree.hpp"
#include "readwrite/pace_2023.hpp"
#include "util/Random.hpp"

using namespace std;
using namespace algorithms;
using namespace readwrite;
using namespace algorithms::exact;
using namespace ds::graph;

void verify_instance(Graph g, int twin_width) {
  g.compute_all_pairs_symmetric_differences();
  DirectSolver solver(g);
  solver.run();
  EXPECT_EQ(solver.twin_width(), twin_width);
  EXPECT_EQ(verify_contraction_sequence(g, solver.contraction_sequence()), twin_width);
}

//
// DirectSolverTest
//
TEST(DirectSolverTest, RunWithSmallInstances) {
  util::set_log_level(util::logging::NONE);
  for (int n = 0; n < 10; ++n) {
    verify_instance(Graph(n), 0);
    verify_instance(generators::complete_graph(n), 0);
    verify_instance(generators::path_graph(n), n <= 3 ? 0 : 1);
  }
}

TEST(DirectSolverTest, RunWithTinyInstances) {
  util::set_log_level(util::logging::NONE);
  verify_instance(load_pace_2023("data/tiny-set/tiny001.gr"), 1);
  verify_instance(load_pace_2023("data/tiny-set/tiny002.gr"), 2);
  verify_instance(load_pace_2023("data/tiny-set/tiny003.gr"), 0);
  verify_instance(load_pace_2023("data/tiny-set/tiny004.gr"), 0);
  verify_instance(load_pace_2023("data/tiny-set/tiny005.gr"), 3);
  verify_instance(load_pace_2023("data/tiny-set/tiny006.gr"), 0);
  verify_instance(load_pace_2023("data/tiny-set/tiny007.gr"), 2);
  verify_instance(load_pace_2023("data/tiny-set/tiny008.gr"), 4);
  verify_instance(load_pace_2023("data/tiny-set/tiny009.gr"), 1);
  verify_instance(load_pace_2023("data/tiny-set/tiny010.gr"), 2);
}
