#include <gtest/gtest.h>

#include "ds/queue/AdaptivePriorityQueue.hpp"
#include "util/Random.hpp"
#include <algorithm>

using namespace std;
using namespace ds::queue;

//
// AdaptivePriorityQueueTest
//
TEST(AdaptivePriorityQueueTest, Top) {
  auto q = AdaptivePriorityQueue<int>(10);
  std::vector<int> xs = {1, 2, 3, 2, 3, 0, 4, 4, 1, 2};
  for (int i = 0; i < 10; ++i) q.push(i, xs[i]);

  EXPECT_EQ(q.size(), 10);
  EXPECT_EQ(q.top(), make_pair(5, 0));
  EXPECT_EQ(q.size(), 10);
  q.pop();

  EXPECT_EQ(q.size(), 9);
  EXPECT_TRUE(q.top() == make_pair(0, 1) || q.top() == make_pair(8, 1));
  q.pop();
  EXPECT_EQ(q.size(), 8);
  EXPECT_TRUE(q.top() == make_pair(0, 1) || q.top() == make_pair(8, 1));
  q.pop();
  EXPECT_EQ(q.size(), 7);
  EXPECT_TRUE(q.top() == make_pair(1, 2) || q.top() == make_pair(3, 2) || q.top() == make_pair(9, 2));
  q.pop();
  EXPECT_EQ(q.size(), 6);
  EXPECT_TRUE(q.top() == make_pair(1, 2) || q.top() == make_pair(3, 2) || q.top() == make_pair(9, 2));
  q.pop();
  EXPECT_EQ(q.size(), 5);
  EXPECT_TRUE(q.top() == make_pair(1, 2) || q.top() == make_pair(3, 2) || q.top() == make_pair(9, 2));
  q.pop();
  EXPECT_EQ(q.size(), 4);
  EXPECT_TRUE(q.top() == make_pair(2, 3) || q.top() == make_pair(4, 3));
  q.pop();
  EXPECT_EQ(q.size(), 3);
  EXPECT_TRUE(q.top() == make_pair(2, 3) || q.top() == make_pair(4, 3));
  q.pop();
  EXPECT_EQ(q.size(), 2);
  EXPECT_TRUE(q.top() == make_pair(6, 4) || q.top() == make_pair(7, 4));
  q.pop();
  EXPECT_EQ(q.size(), 1);
  EXPECT_TRUE(q.top() == make_pair(6, 4) || q.top() == make_pair(7, 4));
  q.pop();
  EXPECT_EQ(q.size(), 0);

  EXPECT_THROW(q.top(), std::invalid_argument);
  EXPECT_THROW(q.pop(), std::invalid_argument);
}

std::vector<std::pair<int, int>> get_sorted_result(AdaptivePriorityQueue<int> &q) {
  std::vector<std::pair<int, int>> result;
  int i = 0;
  int b = 0;
  while (!q.empty()) {
    auto p = q.top();
    result.push_back({p.second, p.first});
    if (i++ == 0) {
      b = p.second;
    } else {
      EXPECT_LE(b, p.second);
      b = p.second;
    }
    q.pop();
  }
  std::sort(result.begin(), result.end());
  return result;
}

TEST(AdaptivePriorityQueueTest, Update) {
  auto q1 = AdaptivePriorityQueue<int>(10);
  std::vector<int> xs = {1, 2, 3, 2, 3, 0, 4, 4, 1, 2};
  for (int i = 0; i < 10; ++i) q1.push(i, xs[i]);

  q1.push(5, 3);
  q1.push(0, 4);
  q1.push(8, 2);
  q1.push(2, 3);
  q1.push(9, 0);
  q1.push(9, 3);
  q1.push(9, 1);

  std::vector<std::pair<int, int>> exp1 = {{1, 9}, {2, 1}, {2, 3}, {2, 8}, {3, 2},
                                           {3, 4}, {3, 5}, {4, 0}, {4, 6}, {4, 7}};
  EXPECT_EQ(get_sorted_result(q1), exp1);

  q1.clear();
  for (int i = 0; i < 5; ++i) q1.push(i, 4);

  q1.pop();
  q1.pop();
  q1.push(4, 0);
  EXPECT_EQ(q1.top(), make_pair(4, 0));

  AdaptivePriorityQueue<int> q2(8);
  for (int i = 0; i < 8; ++i) q2.push(i, 0);

  q2.push(1, 1);
  q2.push(3, 2);
  q2.push(2, 2);
  q2.push(0, 2);
  q2.push(1, 2);
  q2.push(3, 1);
  q2.push(7, 1);
  q2.push(4, 1);
  q2.push(1, 0);
  q2.push(0, 0);
  q2.push(5, 1);
  q2.push(6, 1);
  q2.push(3, 2);
  q2.push(2, 0);

  std::vector<std::pair<int, int>> exp2 = {
      {0, 0}, {0, 1}, {0, 2}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, {2, 3},
  };
  EXPECT_EQ(get_sorted_result(q2), exp2);
}

TEST(AdaptivePriorityQueueTest, RandomInput) {
  util::Random rand(12345);

  vector<int> ks = {5, 10, 20};
  vector<int> ns = {5, 10, 100};
  int num_iterations = 1000;

  for (auto k_size : ks) {
    for (auto n : ns) {
      for (int j = 0; j < n; ++j) {
        // printf("Round %d\n", j);
        vector<int> xs(n, 0);
        AdaptivePriorityQueue<int> q(n);
        for (int i = 0; i < n; ++i) q.push(i, xs[i]);

        for (int z = 0; z < num_iterations; ++z) {
          int i = rand.randint(0, n - 1);
          int k = rand.randint(0, k_size - 1);
          // printf("xs[%d] = %d\n", i, k);
          q.push(i, k);
          xs[i] = k;
        }

        for (int k = 0; k < k_size; ++k) {
          // printf("k=%d\n", k);
          int multiplicity = std::count(xs.begin(), xs.end(), k);
          for (int t = 0; t < multiplicity; ++t) {
            auto p = q.top();
            q.pop();
            // printf("p = (%d, %d)\n", p.first, p.second);
            EXPECT_EQ(xs[p.first], k);
            EXPECT_EQ(p.second, k);
          }
        }
      }
    }
  }
}
