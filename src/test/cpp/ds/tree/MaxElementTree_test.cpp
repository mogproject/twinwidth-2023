#include <gtest/gtest.h>

#include "ds/tree/MaxElementTree.hpp"
#include "util/Random.hpp"
#include <algorithm>

using namespace std;
using namespace ds::tree;

//
// MaxElementTreeTest
//
TEST(MaxElementTreeTest, SmallInput) {
  MaxElementTree t(10);

  EXPECT_EQ(t.max_element(), 0);

  t.update(0, 50);
  EXPECT_EQ(t.max_element(), 50);

  t.update(9, 51);
  EXPECT_EQ(t.max_element(), 51);
  t.update(9, 50);
  EXPECT_EQ(t.max_element(), 50);
  t.update(9, 49);
  EXPECT_EQ(t.max_element(), 50);
  t.update(0, 1);
  EXPECT_EQ(t.max_element(), 49);
  t.update(9, 2);
  EXPECT_EQ(t.max_element(), 2);
  t.update(5, 2);
  EXPECT_EQ(t.max_element(), 2);
  t.update(5, 1);
  EXPECT_EQ(t.max_element(), 2);
  t.update(5, 10);
  EXPECT_EQ(t.max_element(), 10);
  t.update(6, 12);
  EXPECT_EQ(t.max_element(), 12);
  t.update(6, 9);
  EXPECT_EQ(t.max_element(), 10);
}

TEST(MaxElementTreeTest, RandomInput) {
  util::Random rand(12345);

  vector<int> ns = {1, 5, 7, 8, 10, 20};
  for (auto n : ns) {
    MaxElementTree t(n);
    vector<int> xs(n);

    for (int j = 0; j < 100; ++j) {
      int i = rand.randint(0, n - 1);
      int x = rand.randint(0, 1000000000);
      t.update(i, x);
      xs[i] = x;
      EXPECT_EQ(t.max_element(), *std::max_element(xs.begin(), xs.end()));
    }
  }
}
