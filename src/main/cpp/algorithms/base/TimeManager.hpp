#pragma once

#include <chrono>

namespace algorithms {
namespace base {
class TimeManager {
 private:
  bool enabled_;
  std::chrono::time_point<std::chrono::system_clock> time_limit_;

 public:
  TimeManager(int time_limit_sec = 0) : enabled_(time_limit_sec > 0) {
    if (time_limit_sec <= 0) return;

    auto now = std::chrono::system_clock::now();
    time_limit_ = now + std::chrono::seconds(time_limit_sec);
  }

  bool is_time_over() const { return enabled_ && std::chrono::system_clock::now() >= time_limit_; }

  int adjust_time_limit(int time_limit) const {
    if (!enabled_) return time_limit;

    auto now = std::chrono::system_clock::now();
    int adjusted = static_cast<int>(std::chrono::duration_cast<std::chrono::seconds>(time_limit_ - now).count());

    if (adjusted <= 0) return 1;
    return time_limit > 0 ? std::min(time_limit, adjusted) : adjusted;
  }
};
}  // namespace base
}  // namespace algorithms
