#pragma once

#include "algorithms/base/SolverInfo.hpp"
#include "algorithms/exact/BranchSolver.hpp"
#include "algorithms/exact/DirectSolver.hpp"
#include "algorithms/preprocess/VertexSeparator.hpp"
#include "algorithms/upperbound/GreedySolver.hpp"
#include "ds/cache/SimpleCache.hpp"

namespace algorithms {
namespace upperbound {

template <typename T>
class UBSeparate {
  T trigraph_;                // initial TriGraph
  base::SolverInfo &result_;  // result for the original instance
  std::vector<int> articulation_points_;
  preprocess::VertexSeparator sep_;
  ds::FastSet fs_;            // general FastSet
  std::vector<T> trigraphs_;  // stack of TriGraph's
  ds::cache::SimpleCache<uint64_t, std::pair<uint64_t, std::vector<std::pair<int, int>>>> cache_table_;

  // parameters
  int divide_threshold_ = 20;

  int greedy_num_iterations_ = 100;
  int greedy_time_limit_ = 5;
  int greedy_volatility_rate_ = 30;
  int permissiveness_ = 1;

  int branch_max_n_ = 20;
  int branch_time_limit_ = 2;
  long long branch_counter_limit_ = 10000000L;

  // constant
  std::vector<std::pair<int, int>> const NO_SOLUTION = {{-1, -1}};

 public:
  UBSeparate(T trigraph, base::SolverInfo &result, std::vector<int> const &articulation_points)
      : trigraph_(trigraph),
        result_(result),
        articulation_points_(articulation_points),
        fs_(trigraph.number_of_nodes()),
        cache_table_(123456789, 1000000) {
    greedy_num_iterations_ = std::min(100, 10000 / trigraph.number_of_nodes());
    // @note An AP of the original graph may not be an AP of a graph after contractions.
    // But an AP of a graph after contractions cannot be an AP of the original graph.
  }

  void run(int num_iterations, util::Random &rand, int permissiveness = 1) {
    log_info("%s UBSeparate started: n=%d, num_iter=%d", result_.to_string().c_str(), trigraph_.number_of_nodes(), num_iterations);
    util::timer_start(11);
    permissiveness_ = permissiveness;

    for (int t = 0; t < num_iterations && !result_.resolved(); ++t) {
      rand.shuffle(articulation_points_);
      trigraphs_.push_back(trigraph_);  // create a fresh copy of the original trigraph
      trigraphs_.back().set_red_degree_cap(red_degree_cap(), true);

      // recursive call
      // printf("=======================\n");
      auto ret = run_iteration_rec(rand);

      trigraphs_.clear();
      if (ret == NO_SOLUTION) continue;

      // verify and update upper bound
      int width = ds::graph::verify_contraction_sequence(trigraph_.original_graph(), ret);
      if (width < 0) {
        log_critical("UBSeparate critical error: ret=%d, seq_size=%d", width, static_cast<int>(ret.size()));
        util::print(ret);
        assert(false);
        return;
      } else {
        if (result_.update_upper_bound(width, ret)) {
          log_warning("%s UBSeparate found new UB: ub=%d", result_.to_string().c_str(), width);
        }
      }

      // local search
      log_trace("%s UBSeparate local search: ub=%d, t=%d", result_.to_string().c_str(), width, t);
      LocalSearch ls(trigraph_.original_graph(), result_, ret);
      // `result_` will be updated if LocalSearch finds a solution better than result_.upper_bound()
      ls.search_red_shift(rand);
    }

    log_info("%s UBSeparate finished: runtime=%.2fs", result_.to_string().c_str(), util::timer_stop(11));
  }

 private:
  int red_degree_cap() const {
    if (result_.upper_bound() < 0) return trigraph_.number_of_nodes() - 1;
    return result_.upper_bound() - 1 + permissiveness_;
  }

  // recursion call
  /**
   * @brief
   *
   * @param rand
   * @param head
   * @param tail
   * @return std::vector<std::pair<int, int>>
   */
  std::vector<std::pair<int, int>> run_iteration_rec(util::Random &rand, std::vector<int> const &head = {},
                                                     std::vector<int> const &tail = {}) {
    int n = trigraphs_.back().number_of_nodes();  // number of active vertices
    int num_tail = tail.size();
    if (n == num_tail) return {};  // nothing to do

    std::vector<int> s, a, b, left_head, right_head;

    //--------------------------------------------------------------------------
    //    Divide
    //--------------------------------------------------------------------------
    if (n > divide_threshold_) {
      int sep_size_max = result_.has_upper_bound() ? result_.upper_bound() - 1 : result_.lower_bound() * 2 + 1;
      for (int sep_size = 1; s.empty() && sep_size <= sep_size_max; ++sep_size) {
        s = find_separators(trigraphs_.back(), rand, sep_size, head, tail, &a, &b);
      }
    }
    // printf("nodes: ");
    // util::print(trigraphs_.back().nodes().to_vector());
    // printf("s: ");
    // util::print(s);
    // printf("a: ");
    // util::print(a);
    // printf("b: ");
    // util::print(b);

    //--------------------------------------------------------------------------
    //    Base Case
    //--------------------------------------------------------------------------
    if (s.empty()) {
      // printf("base case\n");
      return find_upper_bound(trigraphs_.back(), rand, tail);
    }

    //--------------------------------------------------------------------------
    //    Compute Next Heads
    //--------------------------------------------------------------------------
    // left-head = head & a; right-head = (head & b) | s | {surviver in (a \ s)}
    int left_surviver = -1;
    fs_.clear();
    for (auto x : s) fs_.set(x);
    for (auto x : a) {
      if (!fs_.get(x)) left_surviver = std::max(left_surviver, x);
    }
    assert(left_surviver >= 0);

    fs_.clear();
    for (auto x : head) fs_.set(x);
    for (auto x : a) {
      if (fs_.get(x)) left_head.push_back(x);
    }
    for (auto x : b) {
      if (fs_.get(x)) right_head.push_back(x);
    }
    util::extend(right_head, s);
    right_head.push_back(left_surviver);

    //--------------------------------------------------------------------------
    //    Left Recursion
    //--------------------------------------------------------------------------
    std::vector<std::pair<int, int>> contractions;
    int num_contract_target = static_cast<int>(a.size() - s.size());
    if (num_contract_target > 1) {
      create_left_trigraph(a);
      contractions = run_iteration_rec(rand, left_head, s);
      trigraphs_.pop_back();

      if (contractions == NO_SOLUTION) return NO_SOLUTION;
    }

    //--------------------------------------------------------------------------
    //    Right Recursion
    //--------------------------------------------------------------------------
    // printf("surviver=%d\ncontractions=", left_surviver);
    // util::print(contractions);
    assert(contractions.empty() || left_surviver == contractions.back().first);

    create_right_trigraph(contractions);
    auto right_contractions = run_iteration_rec(rand, right_head, tail);

    if (right_contractions == NO_SOLUTION) return NO_SOLUTION;

    // concatinate contraction sequences
    util::extend(contractions, right_contractions);
    return contractions;
  }

  void create_left_trigraph(std::vector<int> const &induce_on) {
    // printf("create_left_trigraph s\n");
    fs_.clear();
    for (auto x : induce_on) fs_.set(x);

    auto &trigraph = trigraphs_.back();

    // create a copy of the trigraph
    assert(trigraph.is_compute_greedy_criteria_done());
    T ret(trigraph);

    for (auto x : trigraph.nodes().to_vector()) {
      if (!fs_.get(x)) ret.remove_vertex(x);
    }

    ret.recompute_greedy_criteria(red_degree_cap());
    trigraphs_.push_back(ret);
    // printf("create_left_trigraph f\n");
  }

  void create_right_trigraph(std::vector<std::pair<int, int>> const &left_contractions) {
    // printf("create_right_trigraph s\n");
    if (left_contractions.empty()) return;  // no change

    auto &trigraph = trigraphs_.back();
    // printf("------\nedges: ");
    // util::print(trigraph.edges());
    // printf("red edges: ");
    // util::print(trigraph.red_edges());

    for (auto &p : left_contractions) {
      trigraph.contract(p.first, p.second);
      // printf("contract (%d, %d), width=%d\n", p.first, p.second, width);
      // auto width = trigraph.contract(p.first, p.second);
      // assert(width <= result_.upper_bound() - 1 + permissiveness_);
    }
    trigraph.recompute_greedy_criteria(red_degree_cap());
    // printf("create_right_trigraph f\n");
  }

  ds::graph::Graph compact_graph(T const &trigraph, std::vector<int> &active_nodes, std::unordered_map<int, int> &label_map) {
    int nn = trigraph.number_of_nodes();
    active_nodes = trigraph.nodes().to_vector();
    for (int i = 0; i < nn; ++i) label_map.insert({active_nodes[i], i});

    ds::graph::Graph ret(nn);

    for (auto u : active_nodes) {
      for (int v : trigraph.neighbors(u)) {
        if (v <= u) continue;
        ret.add_edge(label_map.at(u), label_map.at(v));
      }
    }
    return ret;
  }

  void cache_set(T const &trigraph, std::vector<int> const &tail, std::vector<std::pair<int, int>> const &contractions) {
    cache_table_.set(cache_key(trigraph, tail), {trigraph.hash(), contractions});
  }

  std::vector<std::pair<int, int>> cache_get(T const &trigraph, std::vector<int> const &tail) {
    auto key = cache_key(trigraph, tail);
    auto p = cache_table_.get(key);
    return trigraph.hash() == p.first ? p.second : std::vector<std::pair<int, int>>({});
  }

  uint64_t cache_key(T const &trigraph, std::vector<int> const &tail) {
    uint64_t ret = 0;
    for (auto x : trigraph.nodes().to_vector()) ret ^= util::get_hash(x % 1024);
    for (auto x : tail) ret ^= util::get_hash((x % 1024) * 1024);
    return ret;
  }

 public:
  std::vector<int> find_separators(T const &trigraph, util::Random &rand, int separator_size, std::vector<int> const &head,
                                   std::vector<int> const &tail, std::vector<int> *a, std::vector<int> *b) {
    // printf("find_separators s\n");
    // printf("nodes: ");
    // util::print(trigraph.nodes().to_vector());
    // printf("head: ");
    // util::print(head);
    // printf("tail: ");
    // util::print(tail);

    if (separator_size == 1) return find_cut_vertex(trigraph, rand, head, tail, a, b);

    std::vector<int> result_a, result_b, head_relabeled, tail_relabeled, ret;
    std::vector<int> active_nodes;
    std::unordered_map<int, int> label_map;

    auto g = compact_graph(trigraph, active_nodes, label_map);

    for (auto x : head) head_relabeled.push_back(label_map.at(x));
    for (auto x : tail) tail_relabeled.push_back(label_map.at(x));

    auto s = sep_.find_vertex_separator(g, separator_size, &result_a, &result_b, {}, tail_relabeled, &rand, true, false, head_relabeled);

    for (auto x : s) ret.push_back(active_nodes[x]);
    a->clear();
    for (auto x : result_a) a->push_back(active_nodes[x]);
    b->clear();
    for (auto x : result_b) b->push_back(active_nodes[x]);

    // printf("find_separators f\n");
    return ret;
  }

 private:
  std::vector<int> find_cut_vertex(T const &trigraph, util::Random &rand, std::vector<int> const &head,
                                   std::vector<int> const &tail, std::vector<int> *a, std::vector<int> *b) {
    // traverse from a vertex
    int orig_n = trigraph_.number_of_nodes();
    int n = trigraph.number_of_nodes();
    auto vs = trigraph.nodes().to_vector();

    ds::FastSet visited(orig_n), head_fs(orig_n);
    for (auto x : head) head_fs.set(x);

    for (auto ap : articulation_points_) {
      // check if this vertex is a cut vertex of the current graph
      if (!trigraph.has_vertex(ap)) continue;  // vertex does not exist
      if (head_fs.get(ap)) continue;           // head cannot be a separator
      if (trigraph.degree(ap) <= 1) continue;  // cut vertex must have at least 2 neighbors

      // obtain the set of vertices reachable from one of ap's neighbors avoiding ap
      int start_v = trigraph.neighbors(ap)[0];
      std::stack<int> st;
      st.push(start_v);
      visited.clear();
      int visited_cnt = 2;
      visited.set(ap);
      visited.set(start_v);
      bool contains_tail = false;

      while (!st.empty()) {
        auto u = st.top();
        st.pop();
        contains_tail |= (!tail.empty() && u == tail[0]);

        for (auto v : trigraph.neighbors(u)) {
          if (!visited.get(v)) {
            ++visited_cnt;
            visited.set(v);
            // contains_tail |= (!tail.empty() && v == tail[0]);
            st.push(v);
          }
        }
      }

      if (visited_cnt < n) {
        // pick ap as a separator
        a->clear();
        b->clear();
        for (auto x : trigraph.nodes().to_vector()) {
          if (x == ap) {
            a->push_back(x);
            b->push_back(x);
          } else if (visited.get(x) == contains_tail) {
            b->push_back(x);
          } else {
            a->push_back(x);
          }
        }

        return {ap};
      }
    }
    return {};
  }

  std::vector<std::pair<int, int>> find_upper_bound(T const &trigraph, util::Random &rand, std::vector<int> const &tail) {
    // printf("find_upper_bound s\n");
    if (trigraph.number_of_nodes() - static_cast<int>(tail.size()) <= 1) return {};

    auto cache_ret = cache_get(trigraph, tail);
    if (!cache_ret.empty()) {
      // cache hit
      return cache_ret;
    }

    // check the max red degree so far
    int max_red_degree = 0;
    for (auto i: trigraph.nodes().to_vector()) max_red_degree = std::max(max_red_degree, trigraph.red_degree(i));

    base::SolverInfo info(-1, std::max(max_red_degree, result_.lower_bound()), result_.upper_bound());  // temporary info
    auto prev_log_level = util::logging::log_level;
    int n = trigraph.number_of_nodes();

    //==========================================================================
    // (1) GreedySolver
    //==========================================================================

    upperbound::GreedySolver<T> gsolver(trigraph, info, tail, true);

    util::set_log_level(util::logging::CRITICAL);
    gsolver.run(rand, greedy_num_iterations_, greedy_time_limit_, greedy_volatility_rate_, permissiveness_);
    util::set_log_level(prev_log_level);
    if (info.resolved()) {
      cache_set(trigraph, tail, info.contraction_sequence());
      return info.contraction_sequence();
    }

    //==========================================================================
    // (2) BranchSolver
    //==========================================================================

    if (n <= branch_max_n_) {
      exact::BranchSolver<T> ub_branch(trigraph, info, tail);

      util::set_log_level(util::logging::CRITICAL);
      ub_branch.run(branch_time_limit_, branch_counter_limit_);
      util::set_log_level(prev_log_level);
    }

    if (result_.has_upper_bound() && info.upper_bound() >= result_.upper_bound() + permissiveness_) {
      return NO_SOLUTION;  // failed to get a better bound
    }
    // printf("find_upper_bound f\n");
    if (info.contraction_sequence().empty()) return NO_SOLUTION;

    // cache solution
    cache_set(trigraph, tail, info.contraction_sequence());
    return info.contraction_sequence();
  }
};
}  // namespace upperbound
}  // namespace algorithms
