#pragma once
#include <unordered_map>
#include <vector>
#include <stdexcept>

namespace sat {
namespace card {

typedef std::pair<int, int> IntPair;
struct IntPairHash {
  int operator()(IntPair pval) const { return pval.first * pval.second; }
};
struct IntPairEqual {
  bool operator()(IntPair p1, IntPair p2) const { return p1.first == p2.first && p1.second == p2.second; }
};
typedef std::unordered_map<IntPair, int, IntPairHash, IntPairEqual> Pair2IntMap;

/**
 * @brief Sequential counter implementation taken from PySAT.
 *
 * @see https://github.com/pysathq/pysat/blob/master/cardenc/seqcounter.hh
 */
class SeqCounter {
 public:
  static void encode_atmost(int &top_id, std::vector<std::vector<int>> &clauses, std::vector<int> const &lits, int bound) {
    int n = lits.size();

    // consider special cases
    if (bound >= n) {
      return;  // always satisfied
    } else if (bound == n - 1) {
      encode_atmost_n_minus_one(clauses, lits);
      return;
    } else if (bound == 1) {
      encode_atmost_one(clauses, lits);
      return;
    } else if (bound == 0) {
      encode_atmost_zero(clauses, lits);
      return;
    } else if (bound < 0) {
      throw std::invalid_argument("infeasible bound");
    }

    // initialize the map
    Pair2IntMap p2i_map;

    for (int j = 0; j < n - bound; ++j) {
      // k = 0, eq 19:
      int s0j = mk_yvar(top_id, p2i_map, std::make_pair(0, j));
      clauses.push_back({-lits[j], s0j});

      // main part, i.e. when 0 <= k < tval - 1:
      for (int k = 0; k < bound - 1; ++k) {
        // eq 18:
        int skj = mk_yvar(top_id, p2i_map, std::make_pair(k, j));
        if (j < n - bound - 1) {
          int skj1 = mk_yvar(top_id, p2i_map, std::make_pair(k, j + 1));
          clauses.push_back({-skj, skj1});
        }

        // eq 19:
        int sk1j = mk_yvar(top_id, p2i_map, std::make_pair(k + 1, j));
        clauses.push_back({-lits[j + k + 1], -skj, sk1j});
      }

      // k = tval - 1, eq 18:
      int stj = mk_yvar(top_id, p2i_map, std::make_pair(bound - 1, j));
      if (j < n - bound - 1) {
        int stj1 = mk_yvar(top_id, p2i_map, std::make_pair(bound - 1, j + 1));
        clauses.push_back({-stj, stj1});
      }

      // k = tval - 1, eq 19:
      clauses.push_back({-lits[j + bound], -stj});
    }
  }

  static void encode_atleast(int &top_id, std::vector<std::vector<int>> &clauses, std::vector<int> const &lits, int bound) {
    int n = lits.size();

    // consider special cases
    if (bound <= 0) {
      return;  // always satisfied
    } else if (bound == 1) {
      encode_atleast_one(clauses, lits);
      return;
    } else if (bound == n - 1) {
      encode_atleast_n_minus_one(clauses, lits);
      return;
    } else if (bound == n) {
      encode_atleast_n(clauses, lits);
      return;
    } else if (bound > n) {
      throw std::invalid_argument("infeasible bound");
    }

    // convert to the atmost bound
    std::vector<int> negs;
    for (auto x: lits) negs.push_back(-x);
    encode_atmost(top_id, clauses, negs, n - bound);
  }

  //----------------------------------------------------------------------------
  //    Common encoding
  //----------------------------------------------------------------------------
  static void encode_atleast_one(std::vector<std::vector<int>> &clauses, std::vector<int> const &lits) {
    clauses.push_back(lits);
  }

  static void encode_atleast_n(std::vector<std::vector<int>> &clauses, std::vector<int> const &lits) {
    for (auto x : lits) clauses.push_back({x});
  }

  static void encode_atmost_zero(std::vector<std::vector<int>> &clauses, std::vector<int> const &lits) {
    for (auto x : lits) clauses.push_back({-x});
  }

  static void encode_atmost_n_minus_one(std::vector<std::vector<int>> &clauses, std::vector<int> const &lits) {
    std::vector<int> cl;
    for (auto x : lits) cl.push_back(-x);
    clauses.push_back(cl);
  }

  static void encode_atleast_n_minus_one(std::vector<std::vector<int>> &clauses, std::vector<int> const &lits) {
    int n = lits.size();
    for (int i = 0; i < n - 1; ++i) {
      for (int j = i + 1; j < n; ++j) clauses.push_back({lits[i], lits[j]});
    }
  }

  static void encode_atmost_one(std::vector<std::vector<int>> &clauses, std::vector<int> const &lits) {
    int n = lits.size();
    for (int i = 0; i < n - 1; ++i) {
      for (int j = i + 1; j < n; ++j) clauses.push_back({-lits[i], -lits[j]});
    }
  }

  static void encode_equals_one(std::vector<std::vector<int>> &clauses, std::vector<int> const &lits) {
    encode_atleast_one(clauses, lits);
    encode_atmost_one(clauses, lits);
  }

 private:
  template <typename PH, typename PT>
  static int mk_yvar(int &top_id, PH &vset, PT const &np) {
    typename PH::iterator ppos = vset.find(np);
    int nid = -1;

    if (ppos == vset.end()) {
      nid = ++top_id;
      vset.insert(make_pair(np, nid));
    } else {
      nid = ppos->second;
    }

    return nid;
  }
};
}  // namespace card
}  // namespace sat
