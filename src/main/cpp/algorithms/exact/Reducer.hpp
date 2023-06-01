#pragma once

#include "ds/graph/ContractionHistory.hpp"
#include "ds/graph/ContractionInfo.hpp"
#include "ds/graph/TriGraph.hpp"

namespace algorithms {
namespace exact {

class Reducer {
 public:

  template <typename T>
  std::vector<std::pair<int, int>> reduce_free_contractions(T &trigraph) {
    int nn = trigraph.number_of_nodes();
    std::vector<int> nodes;
    std::vector<std::pair<int, int>> seq;
    ds::graph::ContractionInfo info(nn);

    for (int i = 0; i < nn; ++i) nodes.push_back(i);

    bool updated = true;
    while (updated) {
      updated = false;
      int n = nodes.size();

      for (int i = 0; i < n - 1 && !updated; ++i) {
        for (int j = i + 1; j < n && !updated; ++j) {
          int ii = i;
          int u = nodes[i], v = nodes[j];
          if (u > v) {
            std::swap(u, v);
            ii = j;
          }

          // printf("u=%d, v=%d\n", u, v);
          if (trigraph.is_free_contraction(u, v)) {
            updated = true;
            trigraph.contract_verbose(v, u, info);
            seq.push_back({v, u});

            // remove vertex u
            std::swap(nodes[ii], nodes[n - 1]);
            nodes.pop_back();
          }
        }
      }
    }
    return seq;
  }
};
}  // namespace exact
}  // namespace algorithms
