#pragma once

#include <sstream>
#include <vector>

#include "util/logger.hpp"

namespace algorithms {
namespace base {
class SolverInfo {
 private:
  int graph_id_;
  int lower_bound_;
  int upper_bound_;

  // std::vector<int> core_;
  std::vector<std::pair<int, int>> contractions_;

 public:
  SolverInfo(int graph_id = 0, int lower_bound = 0, int upper_bound = -1)
      : graph_id_(graph_id), lower_bound_(lower_bound), upper_bound_(upper_bound) {}

  //
  //    Getters
  //
  int lower_bound() const { return lower_bound_; }
  bool has_upper_bound() const { return upper_bound_ >= 0; }
  int upper_bound() const { return upper_bound_; }
  // std::vector<int> core() const { return core_; }
  // int core_size() const { return core_.size(); }
  std::vector<std::pair<int, int>> contraction_sequence() const { return contractions_; }

  bool resolved() const { return upper_bound_ >= 0 && lower_bound_ >= upper_bound_; }

  //
  //    Setters
  //
  bool update_lower_bound(int lower_bound) {
    if (lower_bound > lower_bound_) {
      lower_bound_ = lower_bound;
      // core_ = core;
      return true;
    }
    // if (lower_bound == lower_bound_) { core_ = core; }
    return false;
  }

  bool update_upper_bound(int upper_bound, std::vector<std::pair<int, int>> const& contractions) {
    if (upper_bound >= 0 && (upper_bound_ < 0 || upper_bound < upper_bound_)) {
      upper_bound_ = upper_bound;
      contractions_ = contractions;
      return true;
    }
    return false;
  }

  void update_exact(int upper_bound, std::vector<std::pair<int, int>> const& contractions) {
    update_lower_bound(upper_bound);
    update_upper_bound(upper_bound, contractions);
  }

  //
  //    Utilities
  //
  std::string to_string() const {
    std::stringstream ss;
    ss << "(G_";
    if (graph_id_ < 0) {
      ss << "*";
    } else {
      ss << graph_id_;
    }

    ss << ") [L:" << lower_bound_ << ",U:";
    if (upper_bound_ < 0) {
      ss << "?";
    } else {
      ss << upper_bound_;
    }
    ss << "]";

    return ss.str();
  }
};
}  // namespace base
}  // namespace algorithms
