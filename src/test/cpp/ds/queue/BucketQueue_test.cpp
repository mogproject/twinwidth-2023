#include <gtest/gtest.h>

#include "ds/queue/BucketQueue.hpp"
#include "util/Random.hpp"
#include "util/util.hpp"

using namespace std;
using namespace ds::queue;

//
// BucketQueueTest
//
TEST(BucketQueueTest, RemoveMin) {
  auto q = BucketQueue({3, 2, 2, 1, 2});

  EXPECT_EQ(q.size(), 5);

  EXPECT_EQ(q.min_value(), 1);
  EXPECT_EQ(q.get_key_with_min_value(), 3);
  EXPECT_EQ(q.get_value(0), 3);
  EXPECT_EQ(q.get_value(1), 2);
  EXPECT_EQ(q.get_value(2), 2);
  EXPECT_EQ(q.get_value(3), 1);
  EXPECT_EQ(q.get_value(4), 2);

  q.remove_min();
  EXPECT_EQ(q.min_value(), 2);
  EXPECT_TRUE(q.get_key_with_min_value() == 1 || q.get_key_with_min_value() == 2 || q.get_key_with_min_value() == 4);

  q.remove_min();
  EXPECT_EQ(q.min_value(), 2);
  EXPECT_TRUE(q.get_key_with_min_value() == 1 || q.get_key_with_min_value() == 2 || q.get_key_with_min_value() == 4);

  q.remove_min();
  EXPECT_EQ(q.min_value(), 2);
  EXPECT_TRUE(q.get_key_with_min_value() == 1 || q.get_key_with_min_value() == 2 || q.get_key_with_min_value() == 4);

  q.remove_min();
  EXPECT_EQ(q.min_value(), 3);
  EXPECT_TRUE(q.get_key_with_min_value() == 0);

  q.remove_min();
  EXPECT_EQ(q.size(), 0);

  EXPECT_THROW(q.remove_min(), std::invalid_argument);
  EXPECT_THROW(q.get_key_with_min_value(), std::invalid_argument);
  EXPECT_THROW(q.min_value(), std::invalid_argument);
}

TEST(BucketQueueTest, Update) {
  auto q = BucketQueue({3, 2, 2, 1, 2});

  EXPECT_THROW(q.update(0, 2), std::invalid_argument);
  EXPECT_THROW(q.update(0, -100), std::invalid_argument);
  EXPECT_THROW(q.update(3, -2), std::invalid_argument);

  q.update(0, 1);   // [4, 2, 2, 1, 2]
  q.update(3, -1);  // [4, 2, 2, 0, 2]
  q.update(2, -2);  // [4, 2, 0, 0, 2]

  EXPECT_EQ(q.min_value(), 0);
  q.remove_min();
  EXPECT_EQ(q.min_value(), 0);
  q.remove_min();
  EXPECT_EQ(q.min_value(), 2);
  q.remove_min();
  EXPECT_EQ(q.min_value(), 2);
  q.remove_min();
  EXPECT_EQ(q.min_value(), 4);

  EXPECT_THROW(q.update(1, 1), std::invalid_argument);  // already removed

  q = BucketQueue({0, 8, 9, 0, 0, 0, 7, 5, 9, 7});
  q.update(2, -2);
  q.update(3, 0);
  q.update(6, -2);
  q.update(3, 5);
  q.update(6, -1);
  q.update(5, 0);
  q.update(2, -5);
  q.update(7, 0);
  q.update(3, -1);
  q.update(2, 4);
  q.update(7, -5);
  q.update(6, 0);
  q.update(6, -4);
  q.update(7, 9);
  q.update(0, 1);
  EXPECT_EQ(q.min_value(), 0);

  q = BucketQueue({5, 9, 1, 5, 6, 0, 4, 9, 2, 5});
  EXPECT_EQ(q.remove_min(), 5);
  EXPECT_EQ(q.remove_min(), 2);
  q.update(9, -2);
  EXPECT_EQ(q.remove_min(), 8);
  EXPECT_EQ(q.remove_min(), 9);
  EXPECT_EQ(q.remove_min(), 6);
  EXPECT_EQ(q.remove_min(), 3);
  q.update(4, -2);
  q.update(7, -6);
  q.update(4, -3);
  EXPECT_EQ(q.min_value(), 1);

  q = BucketQueue({5, 9, 1, 5, 6, 0, 4, 9, 2, 5});
  EXPECT_EQ(q.remove_min(), 5);
  EXPECT_EQ(q.remove_min(), 2);
  q.update(9, -2);
  EXPECT_EQ(q.remove_min(), 8);
  q.update(3, 0);
  EXPECT_EQ(q.remove_min(), 9);
  EXPECT_EQ(q.remove_min(), 6);
  EXPECT_EQ(q.remove_min(), 3);
  q.update(4, -2);
  q.update(7, -6);
  q.update(4, -3);
  EXPECT_EQ(q.remove_min(), 4);
  EXPECT_EQ(q.min_value(), 3);

  q = BucketQueue({9, 3, 3, 6, 4, 1, 4, 6, 6, 3});
  EXPECT_EQ(q.remove_min(), 5);
  EXPECT_EQ(q.remove_min(), 1);
  EXPECT_EQ(q.remove_min(), 2);
  EXPECT_EQ(q.remove_min(), 9);
  EXPECT_EQ(q.remove_min(), 4);
  q.update(6, -2);
  q.update(3, -3);
  q.update(3, 1);
  EXPECT_EQ(q.min_value(), 2);

  q = BucketQueue({9, 3, 3, 6, 0, 1, 4, 6, 6, 3});
  q.update(4, 4);
  EXPECT_EQ(q.remove_min(), 5);
  EXPECT_EQ(q.remove_min(), 9);
  EXPECT_EQ(q.remove_min(), 1);
  EXPECT_EQ(q.remove_min(), 2);
  q.update(0, -2);
  EXPECT_EQ(q.remove_min(), 4);
  q.update(6, -2);
  q.update(3, -3);
  q.update(3, 1);
  EXPECT_EQ(q.min_value(), 2);

  q = BucketQueue({3, 7, 9, 5, 2, 4, 0, 7, 5, 2});
  q.update(3, 2);
  EXPECT_EQ(q.remove_min(), 6);
  EXPECT_EQ(q.remove_min(), 4);
  q.update(3, -6);
  q.update(5, -4);
  EXPECT_EQ(q.remove_min(), 5);
  EXPECT_EQ(q.min_value(), 1);
}

TEST(BucketQueueTest, RandomInput) {
  util::Random rand(12345);

  int n = 10;
  std::map<int, int> m;
  std::vector<int> xs;
  auto cmp = [](auto const& l, auto const& r) { return l.second < r.second; };

  for (int t = 0; t < 100; ++t) {
    m.clear();
    xs.clear();

    // create random input
    for (int i = 0; i < n; ++i) {
      int x = rand.randint(0, n - 1);
      xs.push_back(x);
      m[i] = x;
    }
    // printf("input: %s\n", util::to_string(xs).c_str());

    BucketQueue q(xs);
    EXPECT_EQ(q.size(), m.size());

    for (int j = 0; j < 100 && !m.empty(); ++j) {
      // pick random operation
      double r = rand.random();
      if (r < 0.4) {
        // update
        int i = rand.randint(0, n - 1);
        if (util::contains(m, i)) {
          int y = rand.randint(0, n - 1);
          // printf("update: %d, %d\n", i, y - m[i]);
          q.update(i, y - m[i]);
          m[i] = y;
        }
      } else if (r < 0.7) {
        // remove min
        int actual_min_val = std::min_element(m.begin(), m.end(), cmp)->second;
        EXPECT_EQ(q.min_value(), actual_min_val);

        auto k = q.remove_min();
        m.erase(k);
        // printf("remove: %d\n", k);
      } else {
        // remove element
        int i = rand.randint(0, n - 1);
        if (util::contains(m, i)) {
          int val = m[i];
          EXPECT_EQ(q.remove(i), val);
          m.erase(i);
        }
      }
    }
  }
}

TEST(BucketQueueTest, Remove) {
  auto q = BucketQueue({3, 2, 2, 1, 2});
  EXPECT_EQ(q.remove(1), 2);
  EXPECT_EQ(q.remove(2), 2);
  EXPECT_EQ(q.remove(4), 2);
  EXPECT_EQ(q.remove_min(), 3);
  EXPECT_EQ(q.remove_min(), 0);

  q = BucketQueue({3, 2, 2, 1, 2});
  EXPECT_EQ(q.remove(1), 2);
  EXPECT_EQ(q.remove(2), 2);
  q.update(0, -3);
  q.update(3, 3);
  q.update(4, 1);
  EXPECT_EQ(q.remove_min(), 0);
}

TEST(BucketQueueTest, GetRandomKeyWithMinValue) {
  util::Random rand(12345);
  auto q = BucketQueue({4, 1, 1, 2, 4, 4, 2, 1, 4, 4, 4, 1, 4, 4, 4, 4});

  int x = q.get_random_key_with_min_value(rand);
  EXPECT_EQ(q.get_value(x), 1);

  for (int i = 1; i < 4; ++i) q.remove(std::min(0, i) * 4 + std::max(0, i));
  q.update(1 * 4 + 2, -1);
  q.update(1 * 4 + 3, -1);

  int y = q.get_random_key_with_min_value(rand);
  EXPECT_EQ(y, 1 * 4 + 3);
  EXPECT_EQ(q.get_value(y), 0);

  q.update(2 * 4 + 3, -1);

  int z = q.get_random_key_with_min_value(rand);
  EXPECT_EQ(z, 2 * 4 + 3);
  EXPECT_EQ(q.get_value(z), 0);
}
