#include "SATSolver.hpp"

namespace sat {
bool volatile sat_ignore_alarm = false;
kissat *volatile sat_solver = nullptr;

void sat_alarm_handler(int sig) {
  // printf("caught alarm: ignore=%d, sat_solver=%d\n", sat_ignore_alarm, (bool)sat_solver);
  switch (sig) {
    case SIGALRM: {
      if (!sat_ignore_alarm && sat_solver) {
        kissat_terminate(sat_solver);
        sat_ignore_alarm = true;
      }
      break;
    }
    default: {
      // do nothing
    }
  }
}  // namespace sat
}  // namespace sat
