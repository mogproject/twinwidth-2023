#include <gtest/gtest.h>

#include "ds/tree/OverlapIntervalTree.hpp"
#include "util/Random.hpp"
#include <algorithm>

using namespace std;
using namespace ds::tree;

typedef std::vector<int> VI;

std::vector<int> get_overlaps(OverlapIntervalTree &t) {
  std::vector<int> ret;
  for (int i = 0; i < t.n(); ++i) ret.push_back(t.get_overlap(i));
  return ret;
}

//
// OverlapIntervalTree
//
TEST(OverlapIntervalTreeTest, SmallInput) {
  OverlapIntervalTree t(10);
  EXPECT_EQ(t.max_overlap(), 0);

  t.add_interval(0, 10);
  EXPECT_EQ(t.max_overlap(), 1);
  EXPECT_EQ(get_overlaps(t), VI({1, 1, 1, 1, 1, 1, 1, 1, 1, 1}));

  t.add_interval(0, 10);
  EXPECT_EQ(t.max_overlap(), 2);
  EXPECT_EQ(get_overlaps(t), VI({2, 2, 2, 2, 2, 2, 2, 2, 2, 2}));

  t.add_interval(3, 6);
  EXPECT_EQ(t.max_overlap(), 3);
  EXPECT_EQ(get_overlaps(t), VI({2, 2, 2, 3, 3, 3, 2, 2, 2, 2}));

  t.add_interval(6, 7);
  EXPECT_EQ(t.max_overlap(), 3);
  EXPECT_EQ(get_overlaps(t), VI({2, 2, 2, 3, 3, 3, 3, 2, 2, 2}));

  t.add_interval(4, 7);
  EXPECT_EQ(t.max_overlap(), 4);
  EXPECT_EQ(get_overlaps(t), VI({2, 2, 2, 3, 4, 4, 4, 2, 2, 2}));

  t.add_interval(9, 10);
  EXPECT_EQ(t.max_overlap(), 4);
  EXPECT_EQ(get_overlaps(t), VI({2, 2, 2, 3, 4, 4, 4, 2, 2, 3}));

  t.remove_interval(0, 10);
  EXPECT_EQ(t.max_overlap(), 3);
  EXPECT_EQ(get_overlaps(t), VI({1, 1, 1, 2, 3, 3, 3, 1, 1, 2}));

  t.remove_interval(3, 6);
  EXPECT_EQ(t.max_overlap(), 3);
  EXPECT_EQ(get_overlaps(t), VI({1, 1, 1, 1, 2, 2, 3, 1, 1, 2}));

  t.remove_interval(6, 7);
  EXPECT_EQ(t.max_overlap(), 2);
  EXPECT_EQ(get_overlaps(t), VI({1, 1, 1, 1, 2, 2, 2, 1, 1, 2}));

  t.add_interval(1, 2);
  EXPECT_EQ(t.max_overlap(), 2);
  EXPECT_EQ(get_overlaps(t), VI({1, 2, 1, 1, 2, 2, 2, 1, 1, 2}));

  t.add_interval(1, 2);
  EXPECT_EQ(t.max_overlap(), 3);
  EXPECT_EQ(get_overlaps(t), VI({1, 3, 1, 1, 2, 2, 2, 1, 1, 2}));

  t.add_interval(1, 2);
  EXPECT_EQ(t.max_overlap(), 4);
  EXPECT_EQ(get_overlaps(t), VI({1, 4, 1, 1, 2, 2, 2, 1, 1, 2}));

  t.add_interval(1, 2);
  EXPECT_EQ(t.max_overlap(), 5);
  EXPECT_EQ(get_overlaps(t), VI({1, 5, 1, 1, 2, 2, 2, 1, 1, 2}));
}

TEST(OverlapIntervalTreeTest, SmallInput2) {
  OverlapIntervalTree t(5);
  t.add_interval(0, 2);
  t.add_interval(2, 3);
  EXPECT_EQ(t.max_overlap(), 1);
  t.remove_interval(0, 2);
  EXPECT_EQ(t.max_overlap(), 1);
  t.remove_interval(2, 3);
  EXPECT_EQ(t.max_overlap(), 0);
}

TEST(OverlapIntervalTreeTest, RandomInput) {
  util::Random rand(12345);

  vector<int> ns = {1, 5, 6, 7, 8, 9, 20};
  for (auto n : ns) {
    OverlapIntervalTree t(n);
    vector<int> xs(n);
    vector<pair<int, int>> rs;

    for (int x = 0; x < 100; ++x) {
      int i = rand.randint(0, n - 1);
      int j = rand.randint(i + 1, n);
      rs.push_back({i, j});

      t.add_interval(i, j);
      for (int p = i; p < j; ++p) ++xs[p];
      EXPECT_EQ(t.max_overlap(), *std::max_element(xs.begin(), xs.end()));
      EXPECT_EQ(get_overlaps(t), xs);
    }

    rand.shuffle(rs);

    for (int p = 0; p < 100; ++p) {
      int i = rs[p].first;
      int j = rs[p].second;
      t.remove_interval(i, j);
      for (int q = i; q < j; ++q) --xs[q];
      EXPECT_EQ(t.max_overlap(), *std::max_element(xs.begin(), xs.end()));
      EXPECT_EQ(get_overlaps(t), xs);
    }
  }
}
