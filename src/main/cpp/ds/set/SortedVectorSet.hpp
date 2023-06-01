#pragma once

#include <algorithm>
#include <vector>

#include "ds/set/basic_set.hpp"

namespace ds {

// class SortedVectorSet : public basic_set<int> {
class SortedVectorSet {
 private:
  std::vector<int> data_;

 public:
  SortedVectorSet(int not_use = 0) {}

  friend bool operator==(SortedVectorSet const& lhs, SortedVectorSet const& rhs) { return lhs.data_ == rhs.data_; }

  friend bool operator!=(SortedVectorSet const& lhs, SortedVectorSet const& rhs) { return !(lhs == rhs); }

  std::size_t size() const { return data_.size(); }

  int capacity() const { return -1; }

  void set(int x) {
    auto it = std::lower_bound(data_.begin(), data_.end(), x);
    if (it == data_.end() || *it != x) data_.insert(it, x);
  }

  void reset(int x) {
    auto it = std::lower_bound(data_.begin(), data_.end(), x);
    if (it != data_.end() && *it == x) data_.erase(it);
  }

  bool get(int x) const {
    auto it = std::lower_bound(data_.begin(), data_.end(), x);
    return (it != data_.end() && *it == x);
  }

  bool empty() const { return data_.empty(); }

  void clear() { data_.clear(); }

  std::vector<int> to_vector() const { return data_; }

  int front() const { return empty() ? -1 : data_.front(); }

  int pop_front() {
    if (empty()) return -1;
    int ret = data_.front();
    data_.erase(data_.begin());
    return ret;
  }

  int back() const { return empty() ? -1 : data_.back(); }

  int pop_back() {
    if (empty()) return -1;
    int ret = data_.back();
    data_.pop_back();
    return ret;
  }

  //----------------------------------------------------------------------------
  //  Set/union
  //----------------------------------------------------------------------------
  SortedVectorSet& operator|=(int x) {
    set(x);
    return *this;
  }

  //----------------------------------------------------------------------------
  //  Reset/set minus
  //----------------------------------------------------------------------------
  SortedVectorSet& operator-=(int x) {
    reset(x);
    return *this;
  }

  // clang-format off
  friend SortedVectorSet operator-(SortedVectorSet const& lhs, int x) { SortedVectorSet ret(lhs); ret -= x; return ret; }
  friend SortedVectorSet operator-(SortedVectorSet const& lhs, SortedVectorSet const& rhs) { return set_minus(lhs, rhs); }
  friend SortedVectorSet operator|(SortedVectorSet const& lhs, int x) { SortedVectorSet ret(lhs); ret |= x; return ret; }
  friend SortedVectorSet operator|(SortedVectorSet const& lhs, SortedVectorSet const& rhs) { return Union(lhs, rhs); }
  friend SortedVectorSet operator&(SortedVectorSet const& lhs, SortedVectorSet const& rhs) { return intersect(lhs, rhs); }
  friend SortedVectorSet operator^(SortedVectorSet const& lhs, SortedVectorSet const& rhs) { return symmetric_difference(lhs, rhs); }
  // clang-format on

  static SortedVectorSet set_minus(SortedVectorSet const& s, SortedVectorSet const& t) {
    SortedVectorSet result;
    std::set_difference(s.data_.begin(), s.data_.end(), t.data_.begin(), t.data_.end(), std::back_inserter(result.data_));
    return result;
  }

  static SortedVectorSet symmetric_difference(SortedVectorSet const& s, SortedVectorSet const& t) {
    SortedVectorSet result;
    std::set_symmetric_difference(s.data_.begin(), s.data_.end(), t.data_.begin(), t.data_.end(),
                                  std::back_inserter(result.data_));
    return result;
  }

  static SortedVectorSet intersect(SortedVectorSet const& s, SortedVectorSet const& t) {
    SortedVectorSet result;
    std::set_intersection(s.data_.begin(), s.data_.end(), t.data_.begin(), t.data_.end(), std::back_inserter(result.data_));
    return result;
  }

  static SortedVectorSet Union(SortedVectorSet const& s, SortedVectorSet const& t) {
    SortedVectorSet result;
    std::set_union(s.data_.begin(), s.data_.end(), t.data_.begin(), t.data_.end(), std::back_inserter(result.data_));
    return result;
  }
};

}  // namespace ds
