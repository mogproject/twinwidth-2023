#pragma once

#include "ds/graph/Graph.hpp"
#include "sat/SATSolver.hpp"

namespace algorithms {
namespace preprocess {
class VertexSeparator {
 public:
  mutable sat::SATSolver solver_;

  VertexSeparator() {}

  /**
   * @brief Returns a list of the articulation points (cut vertices) of the graph.
   *
   * @return articulation points
   */
  std::vector<int> find_articulation_points(ds::graph::Graph const& g) const;

  std::vector<ds::graph::Graph> decompose_into_covers(ds::graph::Graph const& graph, int max_separator_size);

  /**
   * @brief Finds a vertex separator of size at most `max_separator_size`.
   * Removing these vertices from the graph makes it disconnected.
   *
   * @param max_separator_size max size of a separator
   * @return list of vertices in the separator; empty list if solution was not found
   */
  std::vector<int> find_vertex_separator(         //
      ds::graph::Graph const& graph,              //
      int max_separator_size,                     //
      std::vector<int>* part_a = nullptr,         //
      std::vector<int>* part_b = nullptr,         //
      std::vector<int> const& head = {},          //
      std::vector<int> const& tail = {},          //
      util::Random* rand = nullptr,               //
      bool allow_overlap = true,                  //
      bool require_balance = false,               //
      std::vector<int> const& non_separator = {}  //
  ) const;

 private:
  inline int x(int i) const;
  inline int y(int i) const;
  inline int z(int i) const;
};
}  // namespace preprocess
}  // namespace algorithms
