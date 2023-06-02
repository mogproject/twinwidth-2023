#include <signal.h>
#include <unistd.h>

#include "LocalSearchPlus.hpp"

namespace algorithms {
namespace upperbound {

//==============================================================================
//  Signal Handling
//==============================================================================
namespace localsearchplus {

bool volatile solver_ignore_alarm = false;
bool volatile solver_terminate_flag = false;

void solver_alarm_handler(int sig) {
  switch (sig) {
    case SIGALRM: {
      if (!solver_ignore_alarm) {
        solver_terminate_flag = true;
        solver_ignore_alarm = true;
      }
      break;
    }
    default: {
      // do nothing
    }
  }
}

void set_timeout(int time_limit_sec) {
  if (time_limit_sec > 0) {
    solver_terminate_flag = false;
    solver_ignore_alarm = false;
    signal(SIGALRM, solver_alarm_handler);
    alarm(time_limit_sec);
  }
}

void reset_timeout() {
  solver_ignore_alarm = true;
  alarm(0);  // cancel previous alarm
  signal(SIGALRM, nullptr);
}
}  // namespace localsearchplus

}  // namespace upperbound
}  // namespace algorithms
