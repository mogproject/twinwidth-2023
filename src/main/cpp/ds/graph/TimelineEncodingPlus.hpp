#pragma once
#include <algorithm>
#include <unordered_map>

#include "TriGraph.hpp"
#include "TimelineEncoding.hpp"

namespace ds {
namespace graph {

/**
 * @brief     Implements the timeline encoding, which efficiently comptues the maximum red degree
 * after making small changes to the contraction sequence.
 *
 * Note: this class is totally thread-unsafe
 *
 * Fields:
 *     vmapping: defines a vertex permutation phi, which takes a Vertex in G and returns the VIndex in H
 *     vmapping_inv: the inverse map of vmapping, mapping the VIndex in H to a Vertex in G
 *
 * Terminology:
 *     Red source  : Can be viewed as a tuple (t, p_t, j) of time t and VIndex pair p_t and j.
 *                   A red edge {p_t,j} is introduced either by the symmetric difference between t and p_t
 *                   or by linking (see below) at time t.
 *     Link        : When linking, a new red source {p_i,j} at time i will be created
 *                   if there is a red edge {i,j} with i<j and p_i != j.
 *     Red interval: Timewise interval between red sources for each vertex pair.
 */
template <typename T>
class TimelineEncodingPlus {
 private:
  int n_orig_;
  int n_active_;
  int n_frozen_;
  int n_unfrozen_;

  T const& trigraph_;  // may have red edges and removed vertices
  std::vector<Vertex> frozen_vertices_;

  std::vector<VIndex> vmapping_;      // phi
  std::vector<Vertex> vmapping_inv_;  // phi inverse
  std::vector<Vertex> parents_;
  std::vector<std::vector<VIndex>> children_;

  /**
   * oit tracks for a given VIndex what intervals there is a red edge on a given VIndex.
   * The presence of overlapping intervals in the oit for a VIndex means that
   * more than one edge is red at a certain time.
   * This is updated based on self.red_edges
   */
  std::vector<ds::tree::OverlapIntervalTree> oit_;

  /**
   * self.oit tracks for a given VIndex what intervals there is a red edge on a given VIndex.
   * The presence of overlapping intervals in the oit for a VIndex means that
   * more than one edge is red at a certain time.
   * This is updated based on red_edges
   */
  ds::tree::MaxElementTree rdeg_;

  /** VIndex whose red degree needs to be updated. */
  std::vector<VIndex> rdeg_updated_;

  /** O(1) set checking for `rdeg_updated`. */
  ds::FastSet rdeg_updated_flags_;

  /** red edge history */
  std::vector<std::unordered_map<VIndex, std::vector<VIndex>>> red_edges_;

  // All these FastSets are just for use in one method at a time, and don't save state.
  // They are initialized here for performance reasons (initializing a FastSet is O(n) ).
  ds::FastSet fs_, fs_a_, fs_b_, fs_c_, fs_frozen_;

 public:
  TimelineEncodingPlus(T const& trigraph, std::vector<int> const& frozen_vertices, std::vector<std::pair<int, int>> const& seq)
      : n_orig_(trigraph.original_graph().number_of_nodes()),
        n_active_(trigraph.number_of_nodes()),
        n_frozen_(frozen_vertices.size()),
        n_unfrozen_(n_active_ - n_frozen_),
        trigraph_(trigraph),
        frozen_vertices_(frozen_vertices),
        vmapping_(n_orig_, -1),  // -1: removed; [n_unfrozen, n_active) if frozen
        vmapping_inv_(n_active_),
        parents_(n_active_, -1),
        children_(n_unfrozen_),
        oit_(n_active_, n_active_ + 1),  // time: [0, n_active + 1]
        rdeg_(n_active_),
        rdeg_updated_flags_(n_active_),
        red_edges_(n_active_),
        fs_(n_active_),
        fs_a_(n_active_),
        fs_b_(n_active_),
        fs_c_(n_active_),
        fs_frozen_(n_orig_) {
    assert(static_cast<int>(seq.size()) == n_unfrozen_ - 1);

    init_vmapping(seq);

    // `parents` is the mapping for which node (parent) each node will contract into.
    for (int i = 0; i < n_unfrozen_ - 1; ++i) { parents_[i] = vmapping_[seq[i].first]; }

    // initial red sources
    for (auto& e : trigraph.red_edges()) {
      // add red edge starting time 0
      add_red_source(vmapping_[e.first], vmapping_[e.second], 0);
    }

    // compute red history
    for (int i = 0; i < n_unfrozen_ - 1; ++i) {
      auto p = parents_[i];
      assert(0 <= p && p < n_unfrozen_);
      children_[p].push_back(i);

      for (auto w : symmetric_difference(i, p, i + 1)) {
        // process edge (p,w) starting time i+1
        add_red_source(p, w, i + 1);
      }
    }
    refresh_red_degrees();

    // for (int i = 0; i < n_active_; ++i ) {
    //   printf("i=%d: ", i);
    //   util::print(red_edges_[i]);
    // }
  }

  int number_of_original_nodes() const { return n_orig_; }
  int number_of_active_nodes() const { return n_active_; }
  int number_of_frozen_nodes() const { return n_frozen_; }
  int number_of_unfrozen_nodes() const { return n_unfrozen_; }

  /**
   * @brief Returns the current maximum red degree.
   *
   * @return int
   */
  int twin_width() const { return rdeg_.max_element(); }

  // O(n log n)
  int get_max_red_degree_at_time(int t) {
    int ret = 0;
    for (int i = 0; i < n_active_; ++i) ret = std::max(ret, oit_[i].get_overlap(t));
    return ret;
  }

  // O(1)
  int get_max_red_degree_by_vertex(Vertex v) { return oit_[vmapping_[v]].max_overlap(); }

  std::vector<std::pair<int, int>> contraction_sequence() const {
    std::vector<std::pair<int, int>> ret;
    for (int i = 0; i < n_unfrozen_ - 1; ++i) ret.push_back({vmapping_inv_[parents_[i]], vmapping_inv_[i]});
    return ret;
  }

  VIndex phi(Vertex v) const { return vmapping_[v]; }

  Vertex phiinv(VIndex i) const { return vmapping_inv_[i]; }

  VIndex get_parent(VIndex i) const { return parents_[i]; }

  void link_parent(Vertex v, Vertex new_p) {
    int i = vmapping_[v];
    link_parent_(i, vmapping_[new_p]);
  }

  void link_parent_(VIndex i, VIndex p) {
    assert(p >= 0);

    // update contraction tree
    parents_[i] = p;
    children_[p].push_back(i);

    auto target = symmetric_difference(i, p, i + 1);
    for (auto& kv : red_edges_[i]) target.push_back(kv.first);

    // add red sources
    for (auto u : target) add_red_source(p, u, i + 1);

    // refresh red degree
    refresh_red_degrees();
  }

  void unlink_parent(Vertex v) {
    int i = vmapping_[v];
    unlink_parent_(i, parents_[i]);
  }

  void unlink_parent_(VIndex i, VIndex p) {
    assert(p >= 0);
    auto target = symmetric_difference(i, p, i + 1);
    for (auto& kv : red_edges_[i]) target.push_back(kv.first);

    // remove red sources
    for (auto u : target) remove_red_source(p, u, i + 1);

    // refresh red degrees
    refresh_red_degrees();

    // update contraction tree
    remove_first(children_[p], i);
    parents_[i] = -1;
  }

  void update_parent(Vertex v, Vertex new_parent) { update_parent_(vmapping_[v], vmapping_[new_parent]); }

  void update_parent_(VIndex i, VIndex new_p) {
    assert(i < new_p);

    int current_p = parents_[i];
    if (current_p == new_p) return;

    unlink_parent_(i, current_p);
    link_parent_(i, new_p);
  }

 private:
  void init_vmapping(std::vector<std::pair<int, int>> const& seq) {
    /*
     * The given `seq` may not be normalized; i.e. there may be a pair (j,i) such that j<i.
     * Then, we need to figure out what the last-standing vertex is.
     */
    if (seq.empty()) return;

    for (int i = 0; i < n_unfrozen_ - 1; ++i) {
      auto v = seq[i].second;
      vmapping_[v] = i;
      vmapping_inv_[i] = v;
    }

    int last_standing = seq.back().first;
    vmapping_[last_standing] = n_unfrozen_ - 1;
    vmapping_inv_[n_unfrozen_ - 1] = last_standing;

    // frozen vertices
    int i = n_unfrozen_;
    for (auto x : frozen_vertices_) {
      vmapping_[x] = i;
      vmapping_inv_[i] = x;
      ++i;
    }
  }

  std::vector<VIndex> symmetric_difference(VIndex i, VIndex j, int atleast) const {
    std::vector<VIndex> ret;
    Vertex ii = vmapping_inv_[i], jj = vmapping_inv_[j];
    for (auto ww : trigraph_.original_graph().get_symmetric_difference(ii, jj)) {
      VIndex w = vmapping_[ww];
      if (w >= atleast) ret.push_back(w);
    }
    return ret;
  }

  void add_rdeg_interval(VIndex i, int t_start, int t_end) {
    if (t_start >= t_end) return;

    oit_[i].add_interval(t_start, t_end);
    if (rdeg_updated_flags_.get(i)) return;

    rdeg_updated_flags_.set(i);
    rdeg_updated_.push_back(i);
  }

  void remove_rdeg_interval(VIndex i, int t_start, int t_end) {
    if (t_start >= t_end) return;

    oit_[i].remove_interval(t_start, t_end);
    if (rdeg_updated_flags_.get(i)) return;

    rdeg_updated_flags_.set(i);
    rdeg_updated_.push_back(i);
  }

  /**
   * @brief Adds a new red source to edge `uv` at time `t`.
   * If the red edge is be transferred to the ancestors in the contraction tree,
   * additional red sources may also be introduced
   * (e.g. red edge {i,j} may be transferred to {p_i,j} if i < j).
   *
   * @param u
   * @param v
   * @param t
   */
  void add_red_source(VIndex u, VIndex v, int t) {
    // printf("add red source: u=%d, v=%d, t=%d\n", u, v, t);

    while (u >= 0 && v >= 0 && u != v) {
      if (u > v) std::swap(u, v);  // make sure u < v

      if (red_edges_[u].find(v) != red_edges_[u].end()) {
        // this edge is red at time u+1
        assert(!red_edges_[u][v].empty());

        int tt = min_element(red_edges_[u][v]);
        red_edges_[u][v].push_back(t);

        add_rdeg_interval(u, t, tt);
        add_rdeg_interval(v, t, tt);
        break;
      }

      // disappears at time u+1
      red_edges_[u][v] = {t};
      add_rdeg_interval(u, t, u + 1);
      add_rdeg_interval(v, t, u + 1);

      // continue if u has a parent
      t = u + 1;
      u = parents_[u];
    }
  }

  void remove_red_source(int u, int v, int t) {
    while (u >= 0 && v >= 0 && u != v) {
      if (u > v) std::swap(u, v);  // make sure u < v

      assert(!red_edges_[u][v].empty());
      remove_first(red_edges_[u][v], t);  // remove first occurence of value t

      if (!red_edges_[u][v].empty()) {
        int tt = min_element(red_edges_[u][v]);
        remove_rdeg_interval(u, t, tt);
        remove_rdeg_interval(v, t, tt);
        break;
      }

      // the last red source was removed
      red_edges_[u].erase(v);

      // remove interval [t, u)
      remove_rdeg_interval(u, t, u + 1);
      remove_rdeg_interval(v, t, u + 1);

      t = u + 1;
      u = parents_[u];
    }
  }

  void refresh_red_degrees() {
    for (auto u : rdeg_updated_) rdeg_.update(u, oit_[u].max_overlap());
    rdeg_updated_.clear();
    rdeg_updated_flags_.clear();
  }
  
  //----------------------------------------------------------------------------
  //    Utilities
  //----------------------------------------------------------------------------
  int min_element(std::vector<int> const& xs) { return *std::min_element(xs.begin(), xs.end()); }

  void remove_first(std::vector<int>& xs, int x) {
    int n = xs.size();
    auto it = std::find(xs.begin(), xs.end(), x);
    assert(it != xs.end());
    int i = std::distance(xs.begin(), it);
    std::swap(xs[i], xs[n - 1]);
    xs.pop_back();
  }
};
}  // namespace graph
}  // namespace ds
