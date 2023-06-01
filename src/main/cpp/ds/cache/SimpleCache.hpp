#pragma once

#include "util/Random.hpp"
#include "util/util.hpp"
#include <unordered_map>
#include <vector>

namespace ds {
namespace cache {
template <typename Key, typename Value>
class SimpleCache {
 private:
  util::Random rand_;
  std::size_t capacity_;
  std::vector<Key> keys_;
  std::unordered_map<Key, Value> values_;

 public:
  SimpleCache(uint32_t seed, std::size_t capacity) : rand_(seed), capacity_(capacity) {}
  void set(Key const& key, Value const& value) {
    while (keys_.size() >= capacity_) {
      // random eviction
      int idx = rand_.randint(0, static_cast<int>(keys_.size()) - 1);
      Key key_to_remove = keys_[idx];

      // override this index
      keys_[idx] = key;
      values_.erase(key_to_remove);
    }                                                         

    values_[key] = value;
  }

  Value get(Key const& key) const {
    if (util::contains(values_, key)) return values_.at(key);
    return Value({});
  }
};
}  // namespace cache
}  // namespace ds
