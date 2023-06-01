// #include <gtest/gtest.h>

// #include "algorithms/BaseSolver.hpp"

// using namespace std;
// using namespace algorithms;

// long long count = 0;

// struct ShortWorker : public BaseSolver {
//   bool run_main() {
//     count = 12345;
//     return true;
//   }
// };

// struct LongWorker : public BaseSolver {
//   bool run_main() {
//     while (!job_interrupted()) ++count;
//     return !job_interrupted();
//   }
// };

// TEST(BaseSolverTest, CheckTimeout) {
//   count = 0;
//   ShortWorker sw;
//   EXPECT_TRUE(sw.run_with_time_limit(1));  // returns immediately
//   EXPECT_EQ(count, 12345);

//   LongWorker lw;
//   EXPECT_FALSE(lw.run_with_time_limit(1));  // waits for 1 second
//   EXPECT_GT(count, 12345);
// }
