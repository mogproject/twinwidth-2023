#include <gtest/gtest.h>

#include "util/Random.hpp"

using namespace std;

//
// RandomTest
//
TEST(RandomTest, WeightedChoice) {
  util::Random rand(12345);
  std::vector<int> uniform = {3, 3, 3, 3, 3};
  std::vector<int> biased1 = {0, 29, 1, 0, 72, 5, 0};
  std::vector<double> biased2 = {0, 0, 0, 0, 1e-30};
  
  for (int t = 0; t < 20; ++t) {
    int ret = rand.weighted_choice(uniform);
    EXPECT_GE(ret, 0);
    EXPECT_LE(ret, 4);

    ret = rand.weighted_choice(biased1);
    EXPECT_GE(ret, 1);
    EXPECT_LE(ret, 5);
    EXPECT_NE(ret, 3);

    ret = rand.weighted_choice(biased1.begin() + 4, biased1.begin() + 6);
    EXPECT_GE(ret, 0);
    EXPECT_LE(ret, 1);

    ret = rand.weighted_choice(biased2);
    EXPECT_EQ(ret, 4);
  }
}

TEST(RandomTest, RandInt) {
  util::Random rand(12345);
  uint64_t min_val = -1, max_val = 0;
  for (int t = 0; t < 100; ++t) {
    auto x = rand.randint<uint64_t>(0, -1);
    min_val = std::min(min_val, x);
    max_val = std::max(max_val, x);
  }
  EXPECT_LT(min_val, 1ULL << 63);
  EXPECT_GT(max_val, 1ULL << 63);
}
