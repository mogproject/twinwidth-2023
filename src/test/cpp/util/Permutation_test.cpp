#include <gtest/gtest.h>

#include "util/Permutation.hpp"

using namespace std;

TEST(PermutationTest, Shuffle) {
  util::Random rand(12345);
  util::Permutation perm(10);

  for (int t = 0; t < 100; ++t) {
    perm.shuffle(rand);
    for (int i = 0; i < 10; ++i) {
      EXPECT_EQ(perm.backward(perm.forward(i)), i);
      EXPECT_EQ(perm.forward(perm.backward(i)), i);
    }
  }
}
