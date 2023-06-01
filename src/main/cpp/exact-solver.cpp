#include "algorithms/ExactSolver.hpp"
#include "ds/graph/TriGraph.hpp"
#include "external/CLI11/CLI11.hpp"
#include "readwrite/pace_2023.hpp"
#include "util/Random.hpp"
#include "util/logger.hpp"

using namespace std;

/**
 * @brief Entry point of the program.
 *
 * @param argc argument count
 * @param argv argument strings
 * @return int status code
 */
int main(int argc, char* argv[]) {
  util::timer_start(1);

  // parameters and flags
  string path;
  int seed = 12345;
  bool tww_mode = false;
  int lb_greedy_num_iterations = 20;

  // parse arguments
  CLI::App app{"Exact solver for PACE 2023"};
  app.add_option("path", path, "Input file path");
  app.add_option("--seed", seed, "Random seed");
  app.add_flag("--tww", tww_mode, "Print twin-width instead of contraction sequence");
  app.add_option("--lb-greedy", lb_greedy_num_iterations, "LBGreedy number of iterations");

  CLI11_PARSE(app, argc, argv);

  // load graph
  ds::graph::Graph graph;

  if (path.empty()) {
    graph = readwrite::read_pace_2023(std::cin);
    log_info("Loaded graph (n=%d, m=%d): <stdin>", graph.number_of_nodes(), graph.number_of_edges());
  } else {
    graph = readwrite::load_pace_2023(path.c_str());
    log_info("Loaded graph (n=%d, m=%d): %s", graph.number_of_nodes(), graph.number_of_edges(), path.c_str());
  }

  // create pseudorandom number generator
  util::Random rand(seed);

  // call main solver
  algorithms::ExactSolver solver(graph, lb_greedy_num_iterations);

  while (true) {
    solver.run(rand);

    // verification
    int ret = ds::graph::verify_contraction_sequence(graph, solver.contraction_sequence());
    if (ret < 0) {
      if (ret == -1) {
        log_critical_("Invalid contraction sequence: empty graph with non-empty sequence")
      } else if (ret == -2) {
        log_critical("Invalid contraction sequence: size mismatch (length=%lu, expected=%d)",
                     solver.contraction_sequence().size(), graph.number_of_nodes() - 1);
      } else if (ret == -3) {
        log_critical_("Invalid contraction sequence: contracting removed vertex");
      } else {
        log_critical("Invalid contraction sequence: ret=%d", ret);
      }
      // return 2;
    } else if (ret > solver.twin_width()) {
      log_critical("Contraction sequence suboptimal: tww=%d, lb=%d", ret, solver.twin_width());
      // return 3;
    } else if (ret < solver.twin_width()) {
      log_critical("This will never happen: tww=%d, lb=%d", ret, solver.twin_width());
      // return 4;
    } else {
      log_success("Contraction sequence verified: tww=%d, runtime=%.2fs", ret, util::timer_stop(1));
      break;
    }
  }

  // output results
  if (tww_mode) {
    printf("%d\n", solver.twin_width());
  } else {
    // print contraction sequence
    for (auto p : solver.contraction_sequence()) printf("%d %d\n", p.first + 1, p.second + 1);
  }

  return 0;
}
