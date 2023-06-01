#include "backbone.h"
#include "backtrack.h"
#include "internal.h"
#include "print.h"
#include "probe.h"
#include "substitute.h"
#include "sweep.h"
#include "vivify.h"

#include <inttypes.h>

bool
kissat_probing (kissat * solver)
{
  if (!solver->enabled_.probe)
    return false;
  if (solver->waiting_.probe.reduce > solver->statistics_.reductions)
    return false;
  return solver->limits_.probe.conflicts <= CONFLICTS;
}

static void
probe (kissat * solver)
{
  kissat_backtrack_propagate_and_flush_trail (solver);
  assert (!solver->inconsistent);
  STOP_SEARCH_AND_START_SIMPLIFIER (probe);
  kissat_phase (solver, "probe", GET (probings),
		"probing limit hit after %" PRIu64 " conflicts",
		solver->limits_.probe.conflicts);
  kissat_substitute (solver);
  kissat_binary_clauses_backbone (solver);
  kissat_vivify (solver);
  kissat_sweep (solver);
  kissat_substitute (solver);
  kissat_binary_clauses_backbone (solver);
  STOP_SIMPLIFIER_AND_RESUME_SEARCH (probe);
}

int
kissat_probe (kissat * solver)
{
  assert (!solver->inconsistent);
  INC (probings);
  assert (!solver->probing);
  solver->probing = true;
  probe (solver);
  UPDATE_CONFLICT_LIMIT (probe, probings, NLOGN, true);
  solver->waiting_.probe.reduce = solver->statistics_.reductions + 1;
  solver->last.probe = solver->statistics_.search_ticks;
  assert (solver->probing);
  solver->probing = false;
  return solver->inconsistent ? 20 : 0;
}
