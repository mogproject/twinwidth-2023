#pragma once
#include <type_traits>

#include "ds/graph/ContractionHistory.hpp"
#include "ds/graph/ContractionInfo.hpp"
#include "ds/graph/Graph.hpp"
#include "ds/set/ArrayBitset.hpp"
#include "ds/set/FastSet.hpp"
#include "ds/set/SortedVectorSet.hpp"
#include "util/Random.hpp"
#include "util/hash_table.hpp"
#include "util/logger.hpp"

namespace ds {
namespace graph {

/**
 * @brief TriGraph implementation.
 *
 * @tparam Set data structure for adjacency sets
 */
template <typename Set>
class TriGraph {
 private:
  bool const dense_ = !std::is_same<Set, SortedVectorSet>::value;
  Graph const* original_graph_;  // pointer to the original graph
  int orig_n_;                   // number of vertices in the original graph
  int n_;                        // number of vertices in the graph
  int m_;                        // number of black or red edges in the graph
  std::vector<Set> adj_;         // adjacency sets
  std::vector<Set> red_adj_;     // adjacency sets for red edges
  std::vector<int> degree_;      // degree
  std::vector<int> red_degree_;  // red degree
  FastSet removed_;              // set of removed vertices

  // data structures for greedy algorithms [O(n^2)-space required]
  int red_degree_cap_;
  std::vector<std::vector<int>> ncn_;  // number of common neighbors
  std::vector<std::vector<int>> ncr_;  // number of common red neighbors
  Set red_cap_reached_;                // set of vertices whose red degree has reached capacity
  SortedVectorSet nodes_;              // active nodes for enumeration

  // internal data structures
  FastSet fs1_, fs2_;  // generic fast set
  uint64_t hash_;      // hash value of this trigraph
  bool greedy_criteria_computed_ = false;

  typedef TriGraph<Set> This;

 public:
  TriGraph() : original_graph_(nullptr), orig_n_(0), n_(0), m_(0), removed_(1), fs1_(1), fs2_(1), hash_(0ULL) {}

  TriGraph(Graph const& graph)
      : original_graph_(&graph),
        orig_n_(graph.number_of_nodes()),
        n_(orig_n_),
        m_(0),
        adj_(orig_n_, Set(orig_n_)),
        red_adj_(orig_n_, Set(orig_n_)),
        degree_(orig_n_),
        red_degree_(orig_n_),
        removed_(orig_n_),
        red_degree_cap_(orig_n_),
        ncn_(orig_n_, std::vector<int>(orig_n_)),
        ncr_(orig_n_, std::vector<int>(orig_n_)),
        red_cap_reached_(orig_n_),
        fs1_(1),
        fs2_(1),
        hash_(0ULL) {
    // set up adjacency sets
    for (int i = 0; i < orig_n_; ++i) {
      for (int j : graph.neighbors(i)) {
        if (i < j) add_edge(i, j, false);
      }
    }

    // set up hash table
    util::initialize_hash_table();

    // log_trace("Created TriGraph: n=%d, m=%d, dense=%s", n_, m_, (dense_ ? "True" : "False"));
  }

  /**
   * equality
   */
  friend bool operator==(This const& lhs, This const& rhs) {
    if (lhs.hash_ != rhs.hash_) return false;
    if (*lhs.original_graph_ != *rhs.original_graph_) return false;
    if (lhs.orig_n_ != rhs.orig_n_) return false;
    if (lhs.n_ != rhs.n_) return false;
    if (lhs.m_ != rhs.m_) return false;
    if (lhs.adj_ != rhs.adj_) return false;
    if (lhs.red_adj_ != rhs.red_adj_) return false;
    if (lhs.degree_ != rhs.degree_) return false;
    if (lhs.red_degree_ != rhs.red_degree_) return false;
    if (lhs.removed_ != rhs.removed_) return false;
    if (lhs.red_degree_cap_ != rhs.red_degree_cap_) return false;
    if (lhs.ncn_ != rhs.ncn_) return false;
    if (lhs.ncr_ != rhs.ncr_) return false;
    if (lhs.red_cap_reached_ != rhs.red_cap_reached_) return false;
    if (lhs.nodes_ != rhs.nodes_) return false;

    return true;
  }

  friend bool operator!=(This const& lhs, This const& rhs) { return !(lhs == rhs); }

  /**
   * copy constructor
   */
  TriGraph(This const& other) {
    original_graph_ = other.original_graph_;
    orig_n_ = other.orig_n_;
    n_ = other.n_;
    m_ = other.m_;
    adj_ = other.adj_;
    red_adj_ = other.red_adj_;
    degree_ = other.degree_;
    red_degree_ = other.red_degree_;
    removed_ = other.removed_;
    red_degree_cap_ = other.red_degree_cap_;
    red_cap_reached_ = other.red_cap_reached_;
    ncn_ = other.ncn_;
    ncr_ = other.ncr_;
    nodes_ = other.nodes_;

    fs1_.resize(other.fs1_.capacity());
    fs2_.resize(other.fs2_.capacity());

    hash_ = other.hash_;
    greedy_criteria_computed_ = other.greedy_criteria_computed_;
  }

  uint64_t hash() const { return hash_; }

  int degree(int i) const { return degree_[i]; }

  int red_degree(int i) const { return red_degree_[i]; }

  int number_of_nodes() const { return n_; }

  int number_of_edges() const { return m_; }

  Graph const& original_graph() const { return *original_graph_; }

  std::vector<int> neighbors(int i) const { return adj_[i].to_vector(); }

  bool has_vertex(int i) const { return 0 <= i && i < orig_n_ && !removed_.get(i); }

  bool has_edge(int i, int j) const {
    if (removed_.get(i) || removed_.get(j) || i == j) return false;
    return (dense_ || degree_[i] <= degree_[j]) ? adj_[i].get(j) : adj_[j].get(i);
  }

  bool has_red_edge(int i, int j) const {
    if (removed_.get(i) || removed_.get(j) || i == j) return false;
    return (dense_ || red_degree_[i] <= red_degree_[j]) ? red_adj_[i].get(j) : red_adj_[j].get(i);
  }

  int get_red_degree_cap() const { return red_degree_cap_; }

  void set_red_degree_cap(int red_degree_cap, bool refresh = false) {
    if (red_degree_cap_ == red_degree_cap) return;

    red_degree_cap_ = red_degree_cap;
    if (refresh) {
      for (int i = 0; i < orig_n_; ++i) refresh_red_cap_reached(i);
    }
  }

  Set const& red_cap_reached() const { return red_cap_reached_; }

  void refresh_red_cap_reached(int i) {
    if (red_degree_[i] >= red_degree_cap_) {
      red_cap_reached_.set(i);
    } else {
      red_cap_reached_.reset(i);
    }
  }

  Set get_unshared_black_neighbors(int i, int j) { return (adj_[i] ^ adj_[j]) - red_adj_[i] - red_adj_[j] - i - j; }

  void add_edge(int i, int j, bool is_red) {
    ++m_;
    adj_[i].set(j);
    adj_[j].set(i);
    ++degree_[i];
    ++degree_[j];

    if (is_red) make_edge_red(i, j);
  }

  inline uint64_t red_edge_hash_mask(int i, int j) const {
    return util::get_hash((std::min(i, j) % 1024) * 1024 + (std::max(i, j) % 1024));
  }

  /**
   * @brief Makes an existing black edge red.
   *
   * @param i endpoint 1
   * @param j endpoint 2
   *
   * Precondition: ij must be an existing black edge
   */
  void make_edge_red(int i, int j) {
    assert(has_edge(i, j) && !has_red_edge(i, j));

    red_adj_[i].set(j);
    red_adj_[j].set(i);
    ++red_degree_[i];
    ++red_degree_[j];

    hash_ ^= red_edge_hash_mask(i, j);
  }

  /**
   * @brief Makes an existing red edge black.
   *
   * @param i endpoint 1
   * @param j endpoint 2
   *
   * Precondition: ij must be an existing red edge
   */
  void make_edge_black(int i, int j) {
    assert(has_red_edge(i, j));

    red_adj_[i].reset(j);
    red_adj_[j].reset(i);
    --red_degree_[i];
    --red_degree_[j];

    hash_ ^= red_edge_hash_mask(i, j);
  }

  void remove_edge(int i, int j) {
    --m_;
    if (has_red_edge(i, j)) make_edge_black(i, j);
    adj_[i].reset(j);
    adj_[j].reset(i);
    --degree_[i];
    --degree_[j];
  }

  void remove_vertex(int i) {
    for (int w : adj_[i].to_vector()) remove_edge(i, w);
    removed_.set(i);
    red_cap_reached_.reset(i);
    nodes_.reset(i);
    --n_;
  }

  void add_vertex(int i) {
    removed_.reset(i);
    nodes_.set(i);
    ++n_;
  }

  /**
   * @brief Contracts vertex i into vertex j.
   *
   * @param j vertex to merge
   * @param i vertex to be merged
   * @return int maximum red degree in the closed neighborhood of j after contraction
   */
  int contract(int j, int i) {
    if (removed_.get(i) || removed_.get(j) || i == j) throw std::invalid_argument("invalid vertex pair");
    // remove edge ij if exists
    if (has_edge(i, j)) remove_edge(i, j);

    // recolor {j, w | w <- N_B(j) - N_B(i)}
    for (int w : ((adj_[j] - red_adj_[j]) - (adj_[i] - red_adj_[i])).to_vector()) { make_edge_red(j, w); }

    // add red edges {j, w| w <- N(i) - N(j)}
    for (int w : (adj_[i] - adj_[j]).to_vector()) add_edge(j, w, true);

    // remove vertex i
    for (int w : adj_[i].to_vector()) remove_edge(i, w);
    removed_.set(i);
    --n_;

    // find a vertex with max red degree
    int ret = red_degree_[j];
    for (int w : adj_[j].to_vector()) ret = std::max(ret, red_degree_[w]);

    return ret;
  }

  int weak_red_potential(int i, int j) const {
    return degree_[i] + degree_[j] - 2 * ncn_[i][j] + ncr_[i][j] - (has_edge(i, j) ? 2 : 0);
  }

  /**
   * @brief
   *
   * @return std::vector<int> const&
   *
   * Valid only when using contract_verbose().
   */
  ds::SortedVectorSet const& nodes() const { return nodes_; }

  std::vector<std::pair<int, int>> edges() const {
    std::vector<std::pair<int, int>> ret;
    for (auto i : nodes().to_vector()) {
      for (auto j : adj_[i].to_vector()) {
        if (i < j) ret.push_back({i, j});
      }
    }
    return ret;
  }

  std::vector<std::pair<int, int>> red_edges() const {
    std::vector<std::pair<int, int>> ret;
    for (auto i : nodes().to_vector()) {
      for (auto j : red_adj_[i].to_vector()) {
        if (i < j) ret.push_back({i, j});
      }
    }
    return ret;
  }

  bool is_compute_greedy_criteria_done() const { return greedy_criteria_computed_; }

  /**
   * @brief
   *
   * Precondition: graph has apsd
   *
   * @param red_degree_cap red degree cap
   */
  void compute_greedy_criteria(int red_degree_cap = -1) {
    red_degree_cap_ = red_degree_cap < 0 ? orig_n_ : red_degree_cap;
    fs1_.resize(orig_n_ * orig_n_);
    fs2_.resize(orig_n_ * orig_n_);

    // initialize the number of common neighbors
    for (int i = 0; i < orig_n_ - 1; ++i) {
      for (int j = i + 1; j < orig_n_; ++j) {
        int diff = original_graph_->get_symmetric_difference_size(i, j);
        ncn_[i][j] = ncn_[j][i] = (degree_[i] + degree_[j] - diff) / 2 - (has_edge(i, j) ? 1 : 0);
        // log_warning("i=%d, j=%d, degi=%d, degj=%d, diff=%d, ncn=%d", i, j, degree_[i], degree_[j], diff, ncn_[i][j]);
      }
    }

    // initialize nodes
    for (int i = 0; i < orig_n_; ++i) nodes_.set(i);
    greedy_criteria_computed_ = true;
  }

  // recompute nodes, ncn, ncr, and red cap reached
  void recompute_greedy_criteria(int red_degree_cap = -1) {
    assert(greedy_criteria_computed_);
    red_degree_cap_ = red_degree_cap < 0 ? orig_n_ : red_degree_cap;

    nodes_.clear();
    for (int i = 0; i < orig_n_; ++i) {
      if (!removed_.get(i)) nodes_.set(i);
    }

    auto vtcs = nodes_.to_vector();
    assert(static_cast<int>(vtcs.size()) == n_);
    for (int ii = 0; ii < n_ - 1; ++ii) {
      auto i = vtcs[ii];
      for (int jj = ii + 1; jj < n_; ++jj) {
        auto j = vtcs[jj];
        auto cmn = adj_[i] & adj_[j];
        ncn_[i][j] = ncn_[j][i] = cmn.size();
        ncr_[i][j] = ncr_[j][i] = (cmn & (red_adj_[i] | red_adj_[j])).size();
      }
    }

    for (int i : vtcs) refresh_red_cap_reached(i);
    // printf("check point: recompute_greedy_criteria\n");
    // check_consistency();
  }

  /**
   * @brief Contracts vertex i into j with verbose information for greedy algorithms.
   *
   * @param j vertex to merge
   * @param i vertex to be merged
   * @param info reference to a ContractionInfo instance
   * @param history pointer to history
   * @return int maximum red degree in the closed neighborhood of j after contraction
   *
   * Precondition:
   *   1. `Graph#compute_all_pairs_symmetric_differences()`
   *   2. `TriGraph#compute_greedy_criteria()`
   */
  int contract_verbose(int j, int i, ContractionInfo& info, ContractionHistory* history = nullptr) {
    // util::print(removed_.to_vector());
    if (removed_.get(i) || removed_.get(j) || i == j) throw std::invalid_argument("invalid vertex pair");
    assert(static_cast<int>(ncn_.size()) == orig_n_);  // make sure compute_greedy_criteria() has been called

    info.clear();
    info.last_merge = j;
    info.last_merged = i;

    // categorize the neighborhood of i and j
    bool edge_ij = has_edge(i, j);
    int edge_ij_red = has_red_edge(i, j) ? 1 : 0;

    auto i_adj_r = red_adj_[i] - j;
    auto i_adj_b = adj_[i] - red_adj_[i] - j;
    auto j_adj_r = red_adj_[j] - i;
    auto j_adj_b = adj_[j] - red_adj_[j] - i;

    auto common_nbrs = (adj_[i] & adj_[j]).to_vector();  // = c1 + c2 + c3 +c4
    auto all_nbrs = ((adj_[i] | adj_[j]) - i - j).to_vector();
    info.common_neighbors = common_nbrs;

    auto a1 = i_adj_b - adj_[j];
    auto a2 = i_adj_r - adj_[j];
    auto b1 = j_adj_b - adj_[i];
    auto b2 = j_adj_r - adj_[i];
    // auto c1 = i_adj_b & j_adj_b;
    // auto c2 = i_adj_b & j_adj_r;
    auto c3 = i_adj_r & j_adj_b;
    auto c4 = i_adj_r & j_adj_r;

    if (history) {
      // set history
      history->merge = j;
      history->merged = i;
      history->removed_black = i_adj_b.to_vector();
      history->removed_red = i_adj_r.to_vector();
      if (edge_ij_red) history->removed_red.push_back(j);
      if (edge_ij && !edge_ij_red) history->removed_black.push_back(j);
      history->new_neighbors = (a1 | a2).to_vector();
      history->recolored = (b1 | c3).to_vector();
    }

    //--------------------------------------------------------------------------
    // (1) Update degrees
    //--------------------------------------------------------------------------
    // do nothing here

    //--------------------------------------------------------------------------
    // (2) Update red degrees
    //--------------------------------------------------------------------------
    // (2a) red-deg(j)
    int red_deg_j_delta = a1.size() + a2.size() + c3.size() + b1.size() - edge_ij_red;
    if (red_deg_j_delta != 0) {
      if (red_degree_[j] + red_deg_j_delta >= red_degree_cap_) {
        red_cap_reached_.set(j);
      } else {
        red_cap_reached_.reset(j);
      }
    }

    // (2b) unshared black neighbors
    for (auto x : (a1 | b1).to_vector()) {
      if (red_degree_[x] + 1 >= red_degree_cap_) red_cap_reached_.set(x);
    }

    // (2c) common red neighbors with two red edges
    for (auto x : c4.to_vector()) {
      if (red_degree_[x] - 1 < red_degree_cap_) red_cap_reached_.reset(x);
    }

    //--------------------------------------------------------------------------
    // (3) Update the number of common neighbors
    //--------------------------------------------------------------------------
    fs1_.clear();

    for (auto a : (a1 | a2).to_vector()) {
      for (auto b : (b1 | b2).to_vector()) {
        add_ncn(a, b, 1);  // j becomes a new common neighbor of a and b
        if (history) history->update_ncn(a, b, 1);
        info.add_pair(a, b);
      }
      for (auto v : adj_[a].to_vector()) {
        if (v != i) {
          add_ncn(j, v, 1);  // a becomes a new common neighbor of j and v
          if (history) history->update_ncn(j, v, 1);
          info.add_pair(j, v);
        }
      }
      if (edge_ij) {
        add_ncn(j, a, -1);  // i was a common neighbor
        if (history) history->update_ncn(j, a, -1);
        info.add_pair(j, a);
      }
    }

    for (int xi = 0; xi < static_cast<int>(common_nbrs.size()); ++xi) {
      auto x = common_nbrs[xi];
      for (int yi = xi + 1; yi < static_cast<int>(common_nbrs.size()); ++yi) {
        auto y = common_nbrs[yi];
        add_ncn(x, y, -1);  // i was a common neighbor of x and y
        if (history) history->update_ncn(x, y, -1);
        info.add_pair(x, y);
      }
      if (edge_ij) {
        add_ncn(j, x, -1);  // i was a common neighbor of j and x
        if (history) history->update_ncn(j, x, -1);
        info.add_pair(x, j);
      }
    }

    //--------------------------------------------------------------------------
    // (4) Update the number of common red neighbors
    //--------------------------------------------------------------------------
    fs1_.clear();

    // (4a) take care of the red edges incident to i (possibly including edge ij)
    fs2_.clear();
    for (auto x : red_adj_[i].to_vector()) {
      for (auto y : adj_[i].to_vector()) {
        if (x == y) continue;
        int id = get_id(x, y);
        if (!fs2_.get(id)) {
          fs2_.set(id);  // avoid double-counting
          add_ncr(x, y, -1);
          if (history) history->update_ncr(x, y, -1);
          info.add_pair(x, y);
        }
      }
    }

    // (4b) take care of the new red edges
    fs2_.clear();
    for (auto x : (a1 | a2 | c3 | b1).to_vector()) {
      for (auto y : all_nbrs) {
        // y --- j --- x ==> y --- j === x or
        // y --- i --- x ==> y --- j === x or
        // y === i --- x ==> y === j === x (j becomes a new common red neighbor of x and y)
        if (x == y || (has_red_edge(j, y) && has_edge(j, x))) continue;
        int id = get_id(x, y);
        if (!fs2_.get(id)) {
          fs2_.set(id);
          add_ncr(x, y, 1);
          if (history) history->update_ncr(x, y, 1);
          info.add_pair(x, y);
        }
      }
      for (auto y : adj_[x].to_vector()) {
        // j --- x --- y ===> j === x --- y
        if (y == j || y == i || (has_red_edge(x, y) && has_edge(j, x))) continue;
        add_ncr(y, j, 1);  // there may be more than one x for j-y pairs
        if (history) history->update_ncr(y, j, 1);
        info.add_pair(y, j);
      }
    }

    //--------------------------------------------------------------------------
    // (5) Make changes
    //--------------------------------------------------------------------------
    // remove edge ij if exists
    if (has_edge(i, j)) remove_edge(i, j);

    // recolor {j, w | w <- N_B(j) - N_B(i)}
    for (int w : (b1 | c3).to_vector()) make_edge_red(j, w);

    // add red edges {j, w| w <- N(i) - N(j)}
    for (int w : (a1 | a2).to_vector()) {
      add_edge(j, w, true);
      info.add_pair(j, w);
    }

    // remove vertex i
    remove_vertex(i);

    //--------------------------------------------------------------------------
    // (6) Return max red degree in the neighborhood
    //--------------------------------------------------------------------------
    // find a vertex with max red degree
    int ret = red_degree_[j];
    // printf("ret first %d\n", ret);
    for (int w : adj_[j].to_vector()) {
      // printf("ret checking w=%d, deg=%d\n", w, red_degree_[w]);
      ret = std::max(ret, red_degree_[w]);
    }
    // for (int ii = 0; ii < orig_n_; ++ii) printf("%d->%d, ", ii, red_degree_[ii]);
    // printf("\n");
    // check_consistency();

    return ret;
  }

  void rollback_history(ContractionHistory const& history) {
    int i = history.merged, j = history.merge;
    add_vertex(i);

    // add edges
    for (auto w : history.removed_black) add_edge(i, w, false);
    for (auto w : history.removed_red) {
      add_edge(i, w, true);
      refresh_red_cap_reached(w);
    }

    // remove edges
    for (auto w : history.new_neighbors) {
      remove_edge(j, w);
      refresh_red_cap_reached(w);
    }

    // recolor edges
    for (auto w : history.recolored) {
      make_edge_black(j, w);
      refresh_red_cap_reached(w);
    }

    // revert ncn/ncr
    for (auto& kv : history.ncn_updated) {
      auto p = history.decode_id(kv.first);
      add_ncn(p.first, p.second, -kv.second);
    }
    for (auto& kv : history.ncr_updated) {
      auto p = history.decode_id(kv.first);
      add_ncr(p.first, p.second, -kv.second);
    }

    // red cap
    refresh_red_cap_reached(i);
    refresh_red_cap_reached(j);
  }

  //==================================================================================================
  // Debugging
  //==================================================================================================

  /**
   * @brief Assert that the properties `red_degree_cap_`, `ncn_`, `ncr_` are consistent
   * with the current graph.
   */
  void check_consistency() const {
    // printf("Checking...: n=%d, orig_n=%d should equal %d\n", n_, orig_n_, original_graph_->number_of_nodes());

    assert(n_ <= orig_n_);
    assert(orig_n_ == original_graph_->number_of_nodes());
    assert(!is_compute_greedy_criteria_done() || n_ == static_cast<int>(nodes().size()));

    for (int i = 0; i < orig_n_; ++i) {
      assert(degree_[i] == static_cast<int>(adj_[i].size()));
      assert(red_degree_[i] == static_cast<int>(red_adj_[i].size()));

      for (int j = 0; j < orig_n_; ++j) {
        if (i == j) assert(!adj_[i].get(i));
        assert(adj_[i].get(j) == adj_[j].get(i));
      }
    }

    if (is_compute_greedy_criteria_done()) {
      for (int i = 0; i < orig_n_; ++i) {
        if (static_cast<int>(ncn_[i].size()) != orig_n_) {
          printf("i=%d, orig_n=%d, ncn_sz=%d\n", i, orig_n_, static_cast<int>(ncn_[i].size()));
        }
        if (static_cast<int>(ncr_[i].size()) != orig_n_) {
          printf("i=%d, orig_n=%d, ncr_sz=%d\n", i, orig_n_, static_cast<int>(ncr_[i].size()));
        }
        assert(static_cast<int>(ncn_[i].size()) == orig_n_);
        assert(static_cast<int>(ncr_[i].size()) == orig_n_);

        for (int j = 0; j < orig_n_; ++j) {
          assert(0 <= ncn_[i][j] && ncn_[i][j] < orig_n_);
          assert(0 <= ncr_[i][j] && ncr_[i][j] < orig_n_);
        }
      }
    } else {
      return;
    }

    for (int i = 0; i < orig_n_; ++i) {
      if (!has_vertex(i)) continue;

      assert((red_degree_[i] >= red_degree_cap_) == red_cap_reached_.get(i));

      for (int j = i + 1; j < orig_n_; ++j) {
        if (!has_vertex(j)) continue;
        assert(ncn_[i][j] == ncn_[j][i]);  // symmetry
        assert(ncr_[i][j] == ncr_[j][i]);  // symmetry

        int ncn = 0;
        int ncr = 0;
        for (int k = 0; k < orig_n_; ++k) {
          if (has_edge(i, k) && has_edge(j, k)) {
            ++ncn;
            if (has_red_edge(i, k) || has_red_edge(j, k)) ++ncr;
          }
        }

        if (ncn_[i][j] != ncn) { printf("ncn_[i][j]=%d, ncn=%d, i=%d, j=%d\n", ncn_[i][j], ncn, i, j); }
        assert(ncn_[i][j] == ncn);
        // log_debug("ncr_[i][j]=%d, ncr=%d, i=%d, j=%d", ncr_[i][j], ncr, i, j);
        assert(ncr_[i][j] == ncr);
      }
    }
  }

  //==================================================================================================
  // For reduction rules
  //==================================================================================================
  bool is_free_contraction(int i, int j) const {
    assert(has_vertex(i) && has_vertex(j));
    assert(static_cast<int>(ncn_.size()) == orig_n_);  // make sure compute_greedy_criteria() has been called

    if (degree_[i] > degree_[j]) std::swap(i, j);
    if (ncn_[i][j] != degree_[i] - (has_edge(i, j) ? 1 : 0)) return false;  // early exit

    // Precondition: deg(i) <= deg(j)
    // Condition 1: $(N(i) \triangle N(j)) \setminus N_R(j) \setminus \{i,j\} = \varnothing$
    if (!((adj_[i] ^ adj_[j]) - red_adj_[j] - i - j).empty()) return false;

    // Condition 2: $N_R(u) \cap N_B(v) = \varnothing$
    if (!(red_adj_[i] & (adj_[j] - red_adj_[j])).empty()) return false;
    return true;
  }

  void add_ncr(int i, int j, int d) {
    assert(0 <= i && i < orig_n_);
    assert(0 <= j && j < orig_n_);
    assert(i != j);
    ncr_[i][j] += d;
    ncr_[j][i] += d;
  }

 private:
  int get_id(int i, int j) { return std::min(i, j) * orig_n_ + std::max(i, j); }

  void add_ncn(int i, int j, int d) {
    assert(0 <= i && i < orig_n_);
    assert(0 <= j && j < orig_n_);
    assert(i != j);
    ncn_[i][j] += d;
    ncn_[j][i] += d;
  }

  void add_pair_updated(std::vector<std::pair<int, int>>& ls, int x, int y) {
    int id = get_id(x, y);
    if (!fs1_.get(id)) {
      ls.push_back({std::min(x, y), std::max(x, y)});
      fs1_.set(id);
    }
  }
};

//==================================================================================================
// Functions
//==================================================================================================

/**
 * @brief Verifies a contraction sequence and returns the max red degree.
 *
 * @param graph graph
 * @param seq contraction sequence
 * @return int -1 if the sequence is invalid; returns the max red degree otherwise
 */
int verify_contraction_sequence(Graph const& graph, std::vector<std::pair<int, int>> const& seq);

/**
 * @brief Make a valid elimination ordering, which satisfies u > v for every elimination pair (u, v).
 *
 * @param seq valid contraction sequence
 * @return normalized contraction sequence
 */
std::vector<std::pair<int, int>> normalize_contraction_sequence(std::vector<std::pair<int, int>> const& seq);
}  // namespace graph
}  // namespace ds

//==================================================================================================
// Constructor macros
//==================================================================================================
/*
 * To use this macro, one must define a template function like this.
 *
 * ```
 * template <typename Set>
 * void f(ds::graph::Graph const& G, ...) { ... }
 * ```
 *
 * where T is the type name ds::graph::TriGraph< ... >.
 * Also, it is required that `graph_` must be instantiated outside the macro.
 */
#define RUN_WITH_TRIGRAPH(graph_, f, ...)                                 \
  ({                                                                      \
    auto n = (graph_).number_of_nodes();                                  \
    auto m = (graph_).number_of_edges();                                  \
    if (n <= (1 << 6)) {                                                  \
      f(ds::graph::TriGraph<ds::ArrayBitset6>(graph_), ##__VA_ARGS__);    \
    } else if (n <= (1 << 7)) {                                           \
      f(ds::graph::TriGraph<ds::ArrayBitset7>(graph_), ##__VA_ARGS__);    \
    } else if (n <= (1 << 8)) {                                           \
      f(ds::graph::TriGraph<ds::ArrayBitset8>(graph_), ##__VA_ARGS__);    \
    } else if (n <= (1 << 9)) {                                           \
      f(ds::graph::TriGraph<ds::ArrayBitset9>(graph_), ##__VA_ARGS__);    \
    } else if (n <= (1 << 10)) {                                          \
      f(ds::graph::TriGraph<ds::ArrayBitset10>(graph_), ##__VA_ARGS__);   \
    } else if (n <= (1 << 11)) {                                          \
      f(ds::graph::TriGraph<ds::ArrayBitset11>(graph_), ##__VA_ARGS__);   \
    } else if (n <= (1 << 12)) {                                          \
      f(ds::graph::TriGraph<ds::ArrayBitset12>(graph_), ##__VA_ARGS__);   \
    } else if (n <= (1 << 13) && m <= (1 << 5) * n) {                     \
      f(ds::graph::TriGraph<ds::ArrayBitset13>(graph_), ##__VA_ARGS__);   \
    } else if (n <= (1 << 14) && m <= (1 << 5) * n) {                     \
      f(ds::graph::TriGraph<ds::ArrayBitset14>(graph_), ##__VA_ARGS__);   \
    } else {                                                              \
      f(ds::graph::TriGraph<ds::SortedVectorSet>(graph_), ##__VA_ARGS__); \
    }                                                                     \
  })
