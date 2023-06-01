#pragma once
#include <chrono>

#include "ds/graph/Graph.hpp"
#include "ds/graph/TriGraph.hpp"
#include "ds/queue/AdaptivePriorityQueue.hpp"
#include "util/Random.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

namespace algorithms {
namespace upperbound {

// Precondition: n^2 fits within 32 bits
template <typename T>
class WeakRedPotential {
 private:
  T trigraph_;
  int n_;  // original number of vertices
  util::Random& rand_;
  int ub_;  // upper-bound
  double volatility_;
  std::vector<int> contractions_;  // contraction sequence

  // internal data structures
  ds::FastSet fs_;
  ds::FastSet fs_frozen_;
  std::vector<int> scores_;
  ds::queue::AdaptivePriorityQueue<double> q_;
  ds::graph::ContractionInfo info_;

 public:
  WeakRedPotential(T const& trigraph, util::Random& rand, double volatility, int upper_bound,
                   std::vector<int> const& frozen_vertices = {})
      : trigraph_(trigraph),
        n_(trigraph.original_graph().number_of_nodes()),
        rand_(rand),
        ub_(upper_bound < 0 ? n_ : upper_bound),
        volatility_(volatility),
        fs_(n_ * n_),
        fs_frozen_(n_),
        scores_(n_ * n_, -1),
        q_(n_ * n_),
        info_(n_) {
    // printf("set cap to %d\n", ub_ - 1);
    // trigraph_.recompute_greedy_criteria(ub_ - 1);
    trigraph_.set_red_degree_cap(ub_ - 1, true);

    // initialize fixed vertices
    for (auto x : frozen_vertices) fs_frozen_.set(x);

    // initialize queue
    for (int i = 0; i < n_ - 1; ++i) {
      if (!trigraph.has_vertex(i)) continue;
      for (int j = i + 1; j < n_; ++j) {
        if (!trigraph.has_vertex(j)) continue;
        update_element(i, j);
      }
    }
  }

  int upper_bound() const { return ub_; }

  std::vector<int> const& contraction_sequence() const { return contractions_; }

  ds::SortedVectorSet const& nodes() const { return trigraph_.nodes(); }

  std::vector<int> unfrozen_nodes() const {
    std::vector<int> ret;
    for (auto i : trigraph_.nodes().to_vector()) {
      if (!fs_frozen_.get(i)) ret.push_back(i);
    }
    return ret;
  }

  int number_of_nodes() const { return trigraph_.number_of_nodes(); }

  int contract(int j, int i) { return trigraph_.contract_verbose(j, i, info_); }

  std::pair<int, int> dequeue() {
    while (!q_.empty()) {
      auto x = q_.top().first;
      q_.pop();
      int i = x / n_;
      int j = x % n_;

      // skip a pair involving a removed vertex
      if (!trigraph_.has_vertex(i) || !trigraph_.has_vertex(j)) continue;

      // skip if the score is too high
      if (score(i, j) >= ub_) continue;

      // some neighbor will exceed the capacity; skip this contraction
      if (!(trigraph_.get_unshared_black_neighbors(i, j) & trigraph_.red_cap_reached()).empty()) continue;

      return {i, j};
    }

    // could not improve the current upper-bound
    return {-1, -1};
  }

  void enqueue() {
    int j = info_.last_merge;

    // (1) j x V
    for (auto v : trigraph_.nodes().to_vector()) {
      if (v != j) update_element(j, v);
    }

    // (2) CN(i,j) x complement of CN(i,j)
    fs_.clear();
    for (auto x : info_.common_neighbors) fs_.set(x);

    std::vector<int> target;
    for (auto v : trigraph_.nodes().to_vector()) {
      if (v != j && !fs_.get(v)) target.push_back(v);
    }

    fs_.clear();
    for (auto u : info_.common_neighbors) {
      for (auto v : target) {
        if (v == u) continue;
        fs_.set(get_id(u, v));
        update_element(u, v);
      }
    }

    // (3) pairs whose ncn/ncr was updated or a new edge was created
    for (auto& p : info_.updated_pairs) {
      if (p.first == j || p.second == j) continue;
      int id = get_id(p.first, p.second);
      if (fs_.get(id)) continue;
      fs_.set(id);
      update_element(p.first, p.second);
    }
  }

 private:
  int score(int i, int j) const { return trigraph_.weak_red_potential(i, j); }

  inline int get_id(int i, int j) const { return std::min(i, j) * n_ + std::max(i, j); }

  void update_element(int i, int j) {
    if (fs_frozen_.get(i) || fs_frozen_.get(j)) return;  // never contract fixed vertices

    int id = get_id(i, j);
    int s = score(i, j);
    if (scores_[id] == s) return;  // no change

    if (s >= ub_) return;  // potential too high

    auto rand_val = rand_.random() * volatility_;
    scores_[id] = s;
    // TODO: define constant
    double free_contraction_bonus = trigraph_.is_free_contraction(i, j) ? -1e5 : 0;
    q_.push(id, s + rand_val + free_contraction_bonus);
  }
};
}  // namespace upperbound
}  // namespace algorithms
