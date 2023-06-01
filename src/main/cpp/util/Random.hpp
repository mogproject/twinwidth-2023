#pragma once
#include <cassert>

#include <algorithm>
#include <random>
#include <stdexcept>

namespace util {
/**
 * @brief Helper class for handling pseudo random numbers.
 */
class Random {
 public:
  /**
   * @brief Constructs a new Random object.
   *
   * @param seed seed for the pseudo random number generator
   */
  Random(uint32_t seed) : gen_(seed), real_dist_(0, 1) {}

  /**
   * @brief Returns a random floating point number in the range [0.0, 1,0).
   *
   * @return random floating point number
   */
  double random() { return real_dist_(gen_); }

  /**
   * @brief Gives a new seed for the current generator.
   *
   * @param seed new seed
   */
  void seed(uint32_t seed) { gen_.seed(seed); }

  /**
   * @brief Returns a random integer N such that a <= N <= b.
   *
   * @tparam result type
   * @param a lower-bound
   * @param b upper_bound
   * @return random integer
   */
  template <typename Int>
  Int randint(Int a, Int b);

  /**
   * @brief Shuffles the given vector.
   *
   * @param xs given vector
   */
  template <typename T>
  void shuffle(std::vector<T>& xs) {
    std::shuffle(xs.begin(), xs.end(), gen_);
  }

  // TODO: implement Vitter's algorithm
  /**
   * @brief Randomly sample k elements from the given vector, without replacement.
   *
   * @param xs given population
   * @param k number of samples
   * @return randomly sampled elements
   */
  template <typename T>
  std::vector<T> sample(std::vector<T> const& xs, std::size_t k) {
    auto n = xs.size();
    if (n <= k) return xs;  // k is large enough so return everything

    auto idx = sampleint(n, k);
    std::vector<T> ret;
    for (auto i : idx) ret.push_back(xs[i]);
    return ret;
  }

  template <typename T>
  int weighted_choice(std::vector<T> const& weights) {
    return weighted_choice(weights.begin(), weights.end());
  }

  /**
   * @brief
   *
   * @tparam Iter iterator type
   * @param it_begin start iterator
   * @param it_end end iterator
   * @return int chosen offset from it_begin
   */
  template <typename Iter>
  int weighted_choice(Iter it_begin, Iter it_end) {
    if (it_begin == it_end) throw std::invalid_argument("empty vector");

    std::vector<double> acc;

    for (auto it = it_begin; it != it_end; ++it) acc.push_back((it == it_begin ? 0.0 : acc.back()) + *it);

    auto r = random() * acc.back();
    int index = std::lower_bound(acc.begin(), acc.end(), r) - acc.begin();
    assert(0 <= index && index < (it_end - it_begin));
    return index;
  }

 public:
  /**
   * @brief Randomly sample k integers from the range [0, n), without replacement.
   *
   * @tparam result type
   * @param n size of the population
   * @param k number of samples
   * @return randomly sampled elements
   */
  template <typename Int>
  std::vector<Int> sampleint(Int n, Int k);

 private:
  std::default_random_engine gen_;
  std::uniform_real_distribution<> real_dist_;
};
}  // namespace util
