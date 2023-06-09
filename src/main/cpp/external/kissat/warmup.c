#include "backtrack.h"
#include "decide.h"
#include "internal.h"
#include "print.h"
#include "propbeyond.h"
#include "terminate.h"
#include "warmup.h"

void
kissat_warmup (kissat * solver)
{
  assert (!solver->level);
  assert (solver->watching);
  assert (!solver->inconsistent);
  assert (GET_OPTION (warmup));
  START (warmup);
  INC (warmups);
#ifndef QUIET
  const statistics *stats = &solver->statistics_;
  uint64_t propagations = stats->propagations;
  uint64_t decisions = stats->decisions;
#endif
  while (solver->unassigned)
    {
      if (TERMINATED (warmup_terminated_1))
	break;
      kissat_decide (solver);
      kissat_propagate_beyond_conflicts (solver);
    }
  assert (!solver->inconsistent);
#ifndef QUIET
  decisions = stats->decisions - decisions;
  propagations = stats->propagations - propagations;

  kissat_very_verbose (solver,
		       "warming-up needed %" PRIu64 " decisions and %"
		       PRIu64 " propagations", decisions, propagations);

  if (solver->unassigned)
    kissat_verbose (solver, "reached decision level %u "
		    "during warming-up saved phases", solver->level);
  else
    kissat_verbose (solver, "all variables assigned at decision level %u "
		    "during warming-up saved phases", solver->level);
#endif
  kissat_backtrack_without_updating_phases (solver, 0);
  STOP (warmup);
}
