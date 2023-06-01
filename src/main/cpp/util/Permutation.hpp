#pragma once

#include "Random.hpp"

namespace util {
class Permutation {
 private:
  std::vector<int> data_;
  std::vector<int> data_inv_;

 public:
  Permutation(int n) : data_(n), data_inv_(n) {
    for (int i = 0; i < n; ++i) data_[i] = data_inv_[i] = i;
  }

  void shuffle(util::Random &rand) {
    int n = data_.size();
    rand.shuffle(data_);
    for (int i = 0; i < n; ++i) data_inv_[data_[i]] = i;
  }

  int forward(int index) const { return data_[index]; }
  int backward(int index) const { return data_inv_[index]; }
};
}  // namespace util
