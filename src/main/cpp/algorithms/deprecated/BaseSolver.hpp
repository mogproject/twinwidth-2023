#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <thread>
#include <vector>

namespace algorithms {

class BaseSolver {
 private:
  std::condition_variable run_main_cv_;
  std::mutex run_main_cv_m_, run_main_ret_m_;
  std::atomic<bool> job_interrupted_;
  bool job_done_;

 public:
  BaseSolver() {}

  /**
   * @brief Main function.
   *
   * @return true job finished successfully
   * @return false job timed out
   */
  virtual bool run_main() = 0;

  /**
   * @brief Returns true if either the time limit has already passed or the job has finished.
   *
   * @return true if the job should terminate immediately
   */
  inline bool job_interrupted() const { return job_interrupted_; }

  bool run_with_time_limit(int time_limit_sec) {
    job_interrupted_ = false;
    job_done_ = false;

    std::thread watcher(&BaseSolver::watch_run_main, this, time_limit_sec);
    std::thread worker([&]() {
      job_done_ = run_main();
      run_main_cv_.notify_one();
    });
    watcher.join();
    worker.join();

    return job_done_;
  }

 private:
  void watch_run_main(int time_limit_sec) {
    auto now = std::chrono::system_clock::now();
    auto until = now + std::chrono::seconds(time_limit_sec);

    std::unique_lock<std::mutex> lock(run_main_cv_m_);
    run_main_cv_.wait_until(lock, until);

    job_interrupted_ = true;
  }
};
}  // namespace algorithms
