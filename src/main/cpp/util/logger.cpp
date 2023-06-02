#include "logger.hpp"
#include <chrono>
#include <cstdio>
#include <unordered_map>

namespace util {
static void print_datetime() {
  auto now = std::chrono::system_clock::now();
  long long int ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
  std::time_t time = std::chrono::system_clock::to_time_t(now);

  char s[20] = "";
  std::strftime(s, sizeof(s), "%F %T", std::localtime(&time));
  fprintf(stderr, "%s,%03lld", s, ms);
}

static void print_reset() { fprintf(stderr, "\x1b[0m\n"); }

static void print_newline() { fprintf(stderr, "\n"); }

static void print_log(char const* color, char const* level) {
  if (color) fprintf(stderr, "%s", color);
  print_datetime();
  fprintf(stderr, " [%-8s] ", level);
}

/*
  Color definitions.
  ANSI ECMA-48 color codes (semicolon-separated):
  bold=1, half-bright=2, underscore=4, doubly-underlined=21, normal-intensity=22
  foreground:30+color, background:40+color
  colors: black=0, red=1, green=2, brown=3, blue=4, magenta=5, cyan=6, white=7
*/
void print_trace_prefix() { print_log("\x1b[36m", "TRACE"); }
void print_debug_prefix() { print_log("\x1b[2;37m", "DEBUG"); }
void print_info_prefix() { print_log(nullptr, "INFO"); }
void print_warning_prefix() { print_log("\x1b[33m", "WARNING"); }
void print_error_prefix() { print_log("\x1b[31m", "ERROR"); }
void print_critical_prefix() { print_log("\x1b[1;31m", "CRITICAL"); }
void print_success_prefix() { print_log("\x1b[1;32m", "SUCCESS"); }

void print_trace_postfix() { print_reset(); }
void print_debug_postfix() { print_reset(); }
void print_info_postfix() { print_newline(); }
void print_warning_postfix() { print_reset(); }
void print_error_postfix() { print_reset(); }
void print_critical_postfix() { print_reset(); }
void print_success_postfix() { print_reset(); }

void set_log_level(int log_level) { util::logging::log_level = log_level; }

static std::unordered_map<int, std::chrono::time_point<std::chrono::system_clock>> timer_started;

void timer_start(int key) {
#if LOGGING_ON
  timer_started[key] = std::chrono::system_clock::now();
#endif
}

double timer_stop(int key) {
#if LOGGING_ON
  auto timer_stopped = std::chrono::system_clock::now();
  auto elapsed_cnt = std::chrono::duration_cast<std::chrono::milliseconds>(timer_stopped - timer_started[key]).count();
  return static_cast<double>(elapsed_cnt) / 1000.0;
#else
  return 0.0;
#endif
}

namespace logging {
int log_level = TRACE;
}
}  // namespace util
