#pragma once
#include <algorithm>
#include <unordered_map>

#include "ds/graph/Graph.hpp"
#include "ds/set/FastSet.hpp"
#include "ds/tree/MaxElementTree.hpp"
#include "ds/tree/OverlapIntervalTree.hpp"

namespace ds {
namespace graph {

// define type aliases not to confuse developers
typedef int Vertex;  // vertex label appearing in G
typedef int VIndex;  // vertex `i` is contracted at time `i`

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
class TimelineEncoding {
 private:
  int n_;
  ds::graph::Graph const& graph_;
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
  ds::FastSet fs_, fs_a_, fs_b_, fs_c_;

 public:
  TimelineEncoding(Graph const& graph, std::vector<std::pair<int, int>> const& seq)
      : n_(graph.number_of_nodes()),
        graph_(graph),
        vmapping_(n_),
        vmapping_inv_(n_),
        parents_(n_, -1),
        children_(n_),
        oit_(n_, n_),
        rdeg_(n_),
        rdeg_updated_flags_(n_),
        red_edges_(n_),
        fs_(n_),
        fs_a_(n_),
        fs_b_(n_),
        fs_c_(n_) {
    assert(static_cast<int>(seq.size()) == n_ - 1);

    init_vmapping(seq);

    // `parents` is the mapping for which node (parent) each node will contract into.
    for (int i = 0; i < n_ - 1; ++i) { parents_[i] = vmapping_[seq[i].first]; }

    // compute red history
    for (int i = 0; i < n_ - 1; ++i) {
      auto p = parents_[i];
      children_[p].push_back(i);

      for (auto w : symmetric_difference(i, p, i + 1)) {
        // process edge (p,w) starting time i
        add_red_source(p, w, i);
      }
    }
    refresh_red_degrees();
  }

  int number_of_nodes() const { return graph_.number_of_nodes(); }

  /**
   * @brief Returns the current maximum red degree.
   *
   * @return int
   */
  int twin_width() const { return rdeg_.max_element(); }

  // O(n log n)
  int get_max_red_degree_at_time(int t) {
    int ret = 0;
    for (int i = 0; i < n_; ++i) ret = std::max(ret, oit_[i].get_overlap(t));
    return ret;
  }

  // O(1)
  int get_max_red_degree_by_vertex(Vertex v) {
    return oit_[vmapping_[v]].max_overlap();
  }

  std::vector<std::pair<int, int>> contraction_sequence() const {
    std::vector<std::pair<int, int>> ret;
    for (int i = 0; i < n_ - 1; ++i) ret.push_back({vmapping_inv_[parents_[i]], vmapping_inv_[i]});
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
    for (auto u : target) add_red_source(p, u, i);

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
    for (auto u : target) remove_red_source(p, u, i);

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

  void update_permutation(Vertex u, Vertex v) {
    if (u == v) return;  // early return

    // initialization
    int x, y, start_index_a = 0, start_index_b = 0;
    bool has_edge_uv;
    std::vector<int> A, B;
    init_update_permutation(u, v, x, y, has_edge_uv, A, B);

    // obtain the list of times where red sources change
    std::vector<int> core_times = {x, y};
    for (auto a : A) core_times.push_back(a);
    for (auto b : B) core_times.push_back(b);

    std::vector<int> target_times;
    for (auto z : core_times) {
      if (z != n_ - 1) target_times.push_back(z);
      for (auto s : children_[z]) {
        if (s != x && s != y && !fs_a_.get(s) && !fs_b_.get(s)) target_times.push_back(s);
      }
    }

    std::sort(target_times.begin(), target_times.end());
    for (auto i : target_times) {
      auto p = parents_[i];
      assert(i < p);

      std::vector<VIndex> to_remove, to_add;

      while (true) {
        if (i == x) {
          if (p == y) {  // Case 1
            break;       // no change
          } else if (fs_a_.get(p)) {
            if (has_edge_uv) {         // Case 2a
              to_remove.push_back(y);  // i = x < y
            } else {                   // Case 2b
              to_add.push_back(y);     // i = x < y
            }
          } else if (fs_b_.get(p)) {
            if (has_edge_uv) {  // Case 3a
              to_add.push_back(y);
            } else {  // Case 3b
              to_remove.push_back(y);
            }
          } else {  // Case 4
            //
          }
        } else if (i == y) {  // Case 5
          //
        } else if (fs_a_.get(i)) {
          if (p == x) {
            if (has_edge_uv) {         // Case 6a
              to_remove.push_back(y);  // i < p = x < y
            } else {                   // Case 6b
              to_add.push_back(y);     // i < p = x < y
            }
          } else if (p == y) {
            if (has_edge_uv) {  // Case 7a
              if (i < x) to_add.push_back(x);
            } else {  // Case 7b
              if (i < x) to_remove.push_back(x);
            }
          } else {
            if (fs_a_.get(p)) {  // Case 8
              //  no change
            } else if (fs_b_.get(p)) {  // Case 9
              //  no change
            } else if (fs_c_.get(p)) {  // Case 10
              if (i < x) to_add.push_back(x);
              if (i < y) to_remove.push_back(y);
            } else {  // Case 11
              if (i < x) to_remove.push_back(x);
              if (i < y) to_add.push_back(y);
            }
            break;
          }
        } else if (fs_b_.get(i)) {
          if (p == x) {
            if (has_edge_uv) {         // Case 12a
              to_add.push_back(y);     // i < p = x < y
            } else {                   // Case 12b
              to_remove.push_back(y);  // i < p = x < y
            }
          } else if (p == y) {
            if (has_edge_uv) {  // Case 13a
              if (i < x) to_remove.push_back(x);
            } else {  // Case 13b
              if (i < x) to_add.push_back(x);
            }
          } else {
            if (fs_a_.get(p)) {  // Case 14
              //  no change
            } else if (fs_b_.get(p)) {  // Case 15
              //  no change
            } else if (fs_c_.get(p)) {  // Case 16
              if (i < x) to_remove.push_back(x);
              if (i < y) to_add.push_back(y);
            } else {  // Case 17
              if (i < x) to_add.push_back(x);
              if (i < y) to_remove.push_back(y);
            }
            break;
          }
        } else if (fs_c_.get(i)) {
          if (p == x) {  // Case 18
            //
          } else if (p == y) {  // Case 19
            //
          } else {
            if (fs_a_.get(p)) {  // Case 20
              if (i < x) to_add.push_back(x);
              if (i < y) to_remove.push_back(y);
            } else if (fs_b_.get(p)) {  // Case 21
              if (i < x) to_remove.push_back(x);
              if (i < y) to_add.push_back(y);
            } else {  // Case 22
              //  no change
            }
            break;
          }
        } else {
          if (p == x) {  // Case 23
            //
          } else if (p == y) {  // Case 24
            //
          } else {
            if (fs_a_.get(p)) {  // Case 25
              if (i < x) to_remove.push_back(x);
              if (i < y) to_add.push_back(y);
            } else if (fs_b_.get(p)) {  // Case 26
              if (i < x) to_add.push_back(x);
              if (i < y) to_remove.push_back(y);
            } else {  // Case 27
              //
            }
            break;
          }
        }

        auto q = (i == x || i == y) ? p : i;

        // mark q's neighbors
        fs_.clear();
        for (auto w : graph_.neighbors(vmapping_inv_[q])) {
          auto ww = vmapping_[w];
          if (ww > i) fs_.set(ww);
        }

        auto is_x_involved = i == x || p == x;

        while (start_index_a < static_cast<int>(A.size()) && A[start_index_a] <= i) ++start_index_a;
        while (start_index_b < static_cast<int>(B.size()) && B[start_index_b] <= i) ++start_index_b;

        for (int ia = start_index_a; ia < static_cast<int>(A.size()); ++ia) {
          auto a = A[ia];
          if (a == p) continue;
          assert(a > i);
          if (fs_.get(a) == is_x_involved) {
            to_add.push_back(a);
          } else {
            to_remove.push_back(a);
          }
        }
        for (int ib = start_index_b; ib < static_cast<int>(B.size()); ++ib) {
          auto b = B[ib];
          if (b == p) continue;
          assert(b > i);
          if (fs_.get(b) == is_x_involved) {
            to_remove.push_back(b);
          } else {
            to_add.push_back(b);
          }
        }
        break;
      }

      for (auto w : to_remove) remove_red_source(p, w, i);
      for (auto w : to_add) add_red_source(p, w, i);
    }

    // update phi
    std::swap(vmapping_[u], vmapping_[v]);
    std::swap(vmapping_inv_[x], vmapping_inv_[y]);

    // update red degree
    refresh_red_degrees();
  }

 private:
  void init_vmapping(std::vector<std::pair<int, int>> const& seq) {
    /*
     * The given `seq` may not be normalized; i.e. there may be a pair (j,i) such that j<i.
     * Then, we need to figure out what the last-standing vertex is.
     * trick: precompute XOR from 0 to n-1
     */
    int const xor_idx[] = {n_ - 1, 1, n_, 0};
    int total_xor = xor_idx[(n_ - 1) % 4];
    for (int i = 0; i < n_; ++i) vmapping_[i] = i;
    for (int i = 0; i < n_ - 1; ++i) {
      auto v = seq[i].second;
      vmapping_[v] = i;
      vmapping_inv_[i] = v;
      total_xor ^= v;  // XOR with vertices in `seq`
    }

    // now, the remaining XOR is the last element
    // @note This can be just `seq.back().first`.
    vmapping_[total_xor] = n_ - 1;
    vmapping_inv_[n_ - 1] = total_xor;
  }

  std::vector<VIndex> symmetric_difference(VIndex i, VIndex j, int atleast) const {
    std::vector<VIndex> ret;
    Vertex ii = vmapping_inv_[i], jj = vmapping_inv_[j];
    for (auto ww : graph_.get_symmetric_difference(ii, jj)) {
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
    while (u >= 0 && v >= 0 && u != v) {
      if (u > v) std::swap(u, v);  // make sure u < v

      if (red_edges_[u].find(v) != red_edges_[u].end()) {
        // this edge is red at time u
        assert(!red_edges_[u][v].empty());

        int tt = min_element(red_edges_[u][v]);
        red_edges_[u][v].push_back(t);

        add_rdeg_interval(u, t, tt);
        add_rdeg_interval(v, t, tt);
        break;
      }

      // disappears at time u
      red_edges_[u][v] = {t};
      add_rdeg_interval(u, t, u);
      add_rdeg_interval(v, t, u);

      // continue if u has a parent
      t = u;
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
      remove_rdeg_interval(u, t, u);
      remove_rdeg_interval(v, t, u);

      t = u;
      u = parents_[u];
    }
  }

  void refresh_red_degrees() {
    for (auto u : rdeg_updated_) rdeg_.update(u, oit_[u].max_overlap());
    rdeg_updated_.clear();
    rdeg_updated_flags_.clear();
  }

  void init_update_permutation(Vertex u, Vertex v, int& x, int& y, bool& has_edge_uv, std::vector<int>& A, std::vector<int>& B) {
    x = vmapping_[u];
    y = vmapping_[v];
    if (y < x) {
      std::swap(u, v);
      std::swap(x, y);
    }

    assert(x < y);  // now, it is guaranteed that x < y

    // partition N(u) union N(v) \ {u,v} into three groups:
    // A := N(u) \ N[v]
    // B := N(v) \ N[u]
    // C := N(u) intersect N(v)
    has_edge_uv = false;
    fs_a_.clear();
    fs_b_.clear();
    fs_c_.clear();

    // TODO: precompute these?
    if (graph_.degree(u) < graph_.degree(v)) {
      for (auto w : graph_.neighbors(u)) {
        if (w == v) {
          has_edge_uv = true;
        } else {
          fs_a_.set(vmapping_[w]);
        }
      }
      for (auto w : graph_.neighbors(v)) {
        if (w == u) continue;
        auto ww = vmapping_[w];
        if (fs_a_.get(ww)) {
          fs_c_.set(ww);
          fs_a_.reset(ww);
        } else {
          fs_b_.set(ww);
          B.push_back(ww);
        }
      }
      for (auto w : graph_.neighbors(u)) {
        if (w == v) continue;
        auto ww = vmapping_[w];
        if (fs_a_.get(ww)) A.push_back(ww);
      }
    } else {
      for (auto w : graph_.neighbors(v)) {
        if (w == u) {
          has_edge_uv = true;
        } else {
          fs_b_.set(vmapping_[w]);
        }
      }
      for (auto w : graph_.neighbors(u)) {
        if (w == v) continue;
        auto ww = vmapping_[w];
        if (fs_b_.get(ww)) {
          fs_c_.set(ww);
          fs_b_.reset(ww);
        } else {
          fs_a_.set(ww);
          A.push_back(ww);
        }
      }
      for (auto w : graph_.neighbors(v)) {
        if (w == u) continue;
        auto ww = vmapping_[w];
        if (fs_b_.get(ww)) B.push_back(ww);
      }
    }
    std::sort(A.begin(), A.end());
    std::sort(B.begin(), B.end());
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
