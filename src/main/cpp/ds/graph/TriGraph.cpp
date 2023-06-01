#include "TriGraph.hpp"

namespace ds {
namespace graph {
namespace impl {
/** Stores the return value of verify_contraction_sequence_impl(). Thread-unsafe. */
static int verify_contraction_sequence_impl_ret;

template <typename T>
void verify_contraction_sequence_impl(T G, std::vector<std::pair<int, int>> const& seq) {
  if (G.number_of_nodes() == 0) {
    verify_contraction_sequence_impl_ret = seq.empty() ? 0 : -1;  // empty graph
    return;
  }

  if (static_cast<int>(seq.size()) != G.number_of_nodes() - 1) {
    verify_contraction_sequence_impl_ret = -2;  // size mismatch
    return;
  }

  int red_deg = 0;
  for (auto& p : seq) {
    if (!G.has_vertex(p.first) || !G.has_vertex(p.second)) {
      verify_contraction_sequence_impl_ret = -3;  // vertex does not exist
      return;
    }
    red_deg = std::max(red_deg, G.contract(p.first, p.second));
  }

  // OK
  verify_contraction_sequence_impl_ret = red_deg;
}
}  // namespace impl

int verify_contraction_sequence(Graph const& graph, std::vector<std::pair<int, int>> const& seq) {
  RUN_WITH_TRIGRAPH(graph, impl::verify_contraction_sequence_impl, seq);
  return impl::verify_contraction_sequence_impl_ret;
}


/**
 * @brief Make a valid elimination ordering, which satisfies u > v for every elimination pair (u, v).
 *
 * @param seq valid contraction sequence
 * @return normalized contraction sequence
 */
std::vector<std::pair<int, int>> normalize_contraction_sequence(std::vector<std::pair<int, int>> const& seq) {
  std::vector<std::pair<int, int>> ret;
  std::unordered_map<int, int> mapping;
  for (int i = 0; i < static_cast<int>(seq.size()); ++i) { mapping[seq[i].second] = seq[i].second; }
  mapping[seq.back().first] = seq.back().first;

  for (auto& p : seq) {
    int i = p.first;
    int j = p.second;
    int u = std::max(mapping[i], mapping[j]);
    int v = std::min(mapping[i], mapping[j]);
    mapping[i] = u;
    mapping[j] = v;
    ret.push_back({u, v});
  }
  return ret;
}
}  // namespace graph
}  // namespace ds
