#pragma once
#include <unordered_set>

#include "algorithms/base/SolverInfo.hpp"
#include "ds/graph/TriGraph.hpp"
#include "util/Random.hpp"

namespace algorithms {
namespace exact {

//==============================================================================
//  Signal Handling
//==============================================================================
namespace branch {

constexpr int const ALGORITHM_ID = 30;
extern bool volatile solver_ignore_alarm;
extern bool volatile solver_terminate_flag;

void solver_alarm_handler(int sig);
void set_timeout(int time_limit_sec);
void reset_timeout();
}  // namespace branch

//==============================================================================
//  Main Class
//==============================================================================
template <typename T>
class BranchSolver {
 private:
  constexpr static std::size_t const CACHE_SIZE = 10000000;

  T trigraph_;
  base::SolverInfo& result_;
  int n_;
  long long counter_;
  long long counter_limit_;
  std::vector<std::pair<int, int>> new_candidates_;
  ds::FastSet removed_candidates_;
  std::vector<std::pair<int, int>> contractions_;
  int lb_;
  std::unordered_set<uint64_t> hash_seen_;
  std::vector<uint64_t> hash_seen_vec_;
  util::Random hash_seen_rand_;
  bool timed_out_;
  std::vector<int> frozen_vertices_;
  ds::FastSet fs_frozen_;

  // internal data structures
  ds::FastSet fs_;

 public:
  BranchSolver(T const& trigraph, base::SolverInfo& result, std::vector<int> const& frozen_vertices = {})
      : trigraph_(trigraph),
        result_(result),
        n_(trigraph.original_graph().number_of_nodes()),
        removed_candidates_(n_ * n_),
        hash_seen_rand_(0),  // random used only for cache eviction
        frozen_vertices_(frozen_vertices),
        fs_frozen_(n_),
        fs_(n_ * n_) {
    if (!trigraph_.is_compute_greedy_criteria_done()) trigraph_.compute_greedy_criteria(result_.lower_bound());
    for (auto x : frozen_vertices) fs_frozen_.set(x);
  }

  void run(int time_limit_sec = 0, long long counter_limit = 100000000L) {
    assert(trigraph_.is_compute_greedy_criteria_done());

    log_info("%s BranchSolver started: timeout=%ds, counter_limit=%lld", result_.to_string().c_str(), time_limit_sec, counter_limit);
    util::timer_start(branch::ALGORITHM_ID);

    // set alarm
    if (time_limit_sec > 0) branch::set_timeout(time_limit_sec);

    timed_out_ = false;
    counter_limit_ = counter_limit;

    while (!timed_out_ && !result_.resolved()) {
      // log_trace("BranchSolver checking: lb=%d", lb_);
      trigraph_.set_red_degree_cap(result_.lower_bound(), true);

      if (search()) break;
      if (result_.update_lower_bound(result_.lower_bound() + 1)) {
        log_warning("%s BranchSolver found new LB: lb=%d, counter=%lld", result_.to_string().c_str(), result_.lower_bound(), counter_);
      }
    }

    if (time_limit_sec > 0) branch::reset_timeout();
    if (timed_out_) {
      log_error("%s BranchSolver timed out: runtime=%.2fs", result_.to_string().c_str(), util::timer_stop(branch::ALGORITHM_ID));
    } else {
      log_info("%s BranchSolver finished: runtime=%.2fs", result_.to_string().c_str(), util::timer_stop(branch::ALGORITHM_ID));
    }
  }

  bool timed_out() const { return timed_out_; }

 private:
  std::vector<int> unfrozen_nodes() const {
    std::vector<int> ret;
    for (auto i : trigraph_.nodes().to_vector()) {
      if (!fs_frozen_.get(i)) ret.push_back(i);
    }
    return ret;
  }

  bool search() {
    // reset counter to estimate the complexity
    counter_ = 0;
    trigraph_.set_red_degree_cap(result_.lower_bound());
    hash_seen_.clear();
    hash_seen_vec_.clear();
    lb_ = result_.lower_bound();

    // initialize candidates
    std::vector<std::pair<int, int>> candidates;
    auto vs = unfrozen_nodes();

    for (auto i: vs) {
      for (auto j: vs) {
        if (i >= j) continue;
        auto wrp = trigraph_.weak_red_potential(i, j);
        if (wrp <= lb_) candidates.push_back({i, j});
      }
    }

    // call recursive function
    return search_recurse(candidates, 0);
  }

  void add_cache(uint64_t hash) {
    if (hash_seen_vec_.size() >= CACHE_SIZE) {
      // random eviction
      int idx = hash_seen_rand_.randint(0, static_cast<int>(hash_seen_vec_.size()) - 1);
      auto hash_to_remove = hash_seen_vec_[idx];

      // override this index
      hash_seen_vec_[idx] = hash;
      hash_seen_.erase(hash_to_remove);
    } else {
      hash_seen_vec_.push_back(hash);
    }
    hash_seen_.insert(hash);
  }

  bool has_cache(uint64_t hash) const { return hash_seen_.find(hash) != hash_seen_.end(); }

  bool search_recurse(std::vector<std::pair<int, int>> const& candidates, int depth) {
    // printf("depth=%d, candidates=", depth); 
    // util::print(candidates);

    int num_frozen = static_cast<int>(frozen_vertices_.size());
    int n_start = trigraph_.number_of_nodes();
    if (n_start <= lb_ + 1 || n_start <= num_frozen + 1) {
      // found optimal solution
      if (n_start > num_frozen + 1) {
        auto vs = unfrozen_nodes();
        for (std::size_t i = 0; i < vs.size() - 1; ++i) contractions_.push_back({vs[vs.size() - 1], vs[i]});
      }

      if (result_.update_upper_bound(lb_, contractions_)) {
        log_warning("%s BranchSolver found new UB: ub=%d, counter=%lld", result_.to_string().c_str(), lb_, counter_);
      }
      return true;  // found a feasible solution
    }

    if (counter_limit_ > 0) {
      counter_ += static_cast<long long>(candidates.size());

      if (counter_ > counter_limit_) {
        log_trace("%s BranchSolver giving up: depth=%d, counter=%lld", result_.to_string().c_str(), depth, counter_);
        return true;  // too complex; stop the search
      }
    }

    // time out
    if (branch::solver_terminate_flag) {
      timed_out_ = true;
      return true;
    }

    //--------------------------------------------------------------------------
    //    Apply Reduction Rules
    //--------------------------------------------------------------------------
    std::vector<ds::graph::ContractionHistory> reduction_history;
    auto cand = reduce_free_contractions(candidates, reduction_history);
    int n_reduced = trigraph_.number_of_nodes();

    if (!reduction_history.empty() && (n_reduced <= lb_ + 1 || n_reduced <= num_frozen + 1)) {
      // found optimal solution after free contractions
      if (n_reduced > num_frozen + 1) {
        auto vs = unfrozen_nodes();
        for (std::size_t i = 0; i < vs.size() - 1; ++i) contractions_.push_back({vs[vs.size() - 1], vs[i]});
      }

      if (result_.update_upper_bound(lb_, contractions_)) {
        log_warning("%s BranchSolver found new UB [reduction]: ub=%d, counter=%lld", result_.to_string().c_str(), lb_, counter_);
      }

      // rollback reductions
      while (!reduction_history.empty()) {
        trigraph_.rollback_history(reduction_history.back());
        reduction_history.pop_back();
      }

      return true;  // found a feasible solution
    }

    // try all candidates at the current level
    bool ret = false;
    for (auto& p : cand) {
      int i = p.first, j = p.second;

      assert(trigraph_.weak_red_potential(i, j) <= lb_);  // debug

      // some neighbor will exceed the capacity; skip this contraction
      if (!(trigraph_.get_unshared_black_neighbors(i, j) & trigraph_.red_cap_reached()).empty()) continue;

      // contract chosen vertices
      ds::graph::ContractionInfo contraction_info(n_);
      ds::graph::ContractionHistory history(n_);
      // printf("Contracting: j=%d, j=%d\n", j, i);
      // printf("nodes: ");
      // util::print(trigraph_.nodes().to_vector());
      trigraph_.contract_verbose(j, i, contraction_info, &history);

      // for (int r = 0; r < depth; ++r) printf("  "); printf("(%d,%d) [%llx]\n", j, i, trigraph_.hash());

      if (has_cache(trigraph_.hash())) {
        //
        // for (int r = 0; r < depth; ++r) printf("  "); printf("(%d,%d) [%llx] cache hit\n", j, i, trigraph_.hash());
      } else {
        contractions_.push_back({j, i});

        // update candidates
        auto next_candidates = update_candidates(contraction_info, cand);

        // recursive call
        ret = search_recurse(next_candidates, depth + 1);

        // set cache
        if (!ret) {
          add_cache(trigraph_.hash());
          // for (int r = 0; r < depth; ++r) printf("  "); printf("(%d,%d) [%llx] cache write\n", j, i, trigraph_.hash());
        }

        contractions_.pop_back();
      }

      // rollback
      trigraph_.rollback_history(history);
      // for (int r = 0; r < depth; ++r) printf("  "); printf("(%d,%d) [%llx] rollback\n", j, i, trigraph_.hash());

      if (ret) break;
    }

    // rollback reductions
    while (!reduction_history.empty()) {
      // printf("rollback: %d, %d\n", reduction_history.back().merge, reduction_history.back().merged);
      trigraph_.rollback_history(reduction_history.back());
      if (!ret) contractions_.pop_back();
      reduction_history.pop_back();
    }

    // false if infeasible
    return ret;
  }

  void update_element(int i, int j) {
    if (fs_frozen_.get(i) || fs_frozen_.get(j)) return;   // never contract frozen vertices

    if (i > j) std::swap(i, j);

    int score = trigraph_.weak_red_potential(i, j);
    removed_candidates_.set(get_id(i, j));
    if (score <= lb_) new_candidates_.push_back({i, j});
  }

  inline int get_id(int i, int j) const { return std::min(i, j) * n_ + std::max(i, j); }

  std::vector<std::pair<int, int>> update_candidates(ds::graph::ContractionInfo const& info,
                                                     std::vector<std::pair<int, int>> const& candidates) {
    new_candidates_.clear();
    removed_candidates_.clear();

    enqueue(info);  // will update `new_candidates_` and `removed_candidates_`

    // find next candidates
    std::vector<std::pair<int, int>> next_candidates = new_candidates_;
    for (auto& q : candidates) {
      if (q.first == info.last_merged || q.second == info.last_merged) continue;  // removed
      if (removed_candidates_.get(get_id(q.first, q.second))) continue;
      next_candidates.push_back(q);
    }
    return next_candidates;
  }

  void enqueue(ds::graph::ContractionInfo const& info) {
    int j = info.last_merge;

    // (1) j x V
    for (auto v : trigraph_.nodes().to_vector()) {
      if (v != j) update_element(j, v);
    }

    // (2) CN(i,j) x complement of CN(i,j)
    fs_.clear();
    for (auto x : info.common_neighbors) fs_.set(x);

    std::vector<int> target;
    for (auto v : trigraph_.nodes().to_vector()) {
      if (v != j && !fs_.get(v)) target.push_back(v);
    }

    fs_.clear();
    for (auto u : info.common_neighbors) {
      for (auto v : target) {
        if (v == u) continue;
        fs_.set(get_id(u, v));
        update_element(u, v);
      }
    }

    // (3) pairs whose ncn/ncr was updated or a new edge was created
    for (auto& p : info.updated_pairs) {
      if (p.first == j || p.second == j) continue;
      int id = get_id(p.first, p.second);
      if (fs_.get(id)) continue;
      fs_.set(id);
      update_element(p.first, p.second);
    }
  }

  // TODO: integrate with the Reducer class
  std::vector<std::pair<int, int>> reduce_free_contractions(std::vector<std::pair<int, int>> const& candidates,
                                                            std::vector<ds::graph::ContractionHistory>& history) {
    std::vector<std::pair<int, int>> cand = candidates;
    bool updated = true;
    int i = -1, j = -1;

    while (updated) {
      updated = false;

      for (auto& p : cand) {
        if (!trigraph_.has_vertex(p.first) || !trigraph_.has_vertex(p.second)) continue;

        // Reduction Rule 1
        // printf("(%d, %d)\n", p.first, p.second);
        if (trigraph_.is_free_contraction(p.first, p.second)) {
          i = std::min(p.first, p.second);
          j = std::max(p.first, p.second);
          updated = true;
          break;
        }
      }

      if (updated) {
        // printf("cand before: ");
        // util::print(cand);
        // printf("j=%d, i=%d\n", j, i);
        contractions_.push_back({j, i});
        history.push_back(ds::graph::ContractionHistory(n_));

        ds::graph::ContractionInfo info(n_);
        trigraph_.contract_verbose(j, i, info, &history.back());
        cand = update_candidates(info, cand);

        // printf("cand after: ");
        // util::print(cand);
      }
    }
    return cand;
  }
};
}  // namespace exact
}  // namespace algorithms
