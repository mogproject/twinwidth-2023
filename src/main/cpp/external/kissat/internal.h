#ifndef _internal_h_INCLUDED
#define _internal_h_INCLUDED

#include "arena.h"
#include "array.h"
#include "assign.h"
#include "averages.h"
#include "check.h"
#include "clause.h"
#include "cover.h"
#include "extend.h"
#include "smooth.h"
#include "flags.h"
#include "format.h"
#include "frames.h"
#include "heap.h"
#include "kimits.h"
#include "kissat.h"
#include "literal.h"
#include "mode.h"
#include "options.h"
#include "phases.h"
#include "profile.h"
#include "proof.h"
#include "queue.h"
#include "random.h"
#include "reluctant.h"
#include "rephase.h"
#include "stack.h"
#include "statistics.h"
#include "literal.h"
#include "value.h"
#include "vector.h"
#include "watch.h"

typedef struct datarank datarank;

struct datarank
{
  unsigned data;
  unsigned rank;
};

typedef struct import import;

struct import
{
  unsigned lit:30;
  bool imported:1;
  bool eliminated:1;
};

typedef struct termination termination;

struct termination
{
#ifdef COVERAGE
  volatile uint64_t flagged;
#else
  volatile bool flagged;
#endif
  volatile void *state;
  int (*volatile terminate) (void *);
};

// *INDENT-OFF*

typedef STACK (value) eliminated;
typedef STACK (import) imports;
typedef STACK (datarank) dataranks;
typedef STACK (watch) statches;
typedef STACK (watch *) patches;

// *INDENT-ON*

struct kitten;

struct kissat
{
#if !defined(NDEBUG) || defined(METRICS)
  bool backbone_computing;
#endif
#ifdef LOGGING
  bool compacting;
#endif
  bool extended;
  bool inconsistent;
  bool iterating;
  bool probing;
#ifndef QUIET
  bool sectioned;
#endif
  bool stable;
#if !defined(NDEBUG) || defined(METRICS)
  bool vivifying;
#endif
  bool watching;

  bool large_clauses_watched_after_binary_clauses;

  termination termination_;

  unsigned vars;
  unsigned size;
  unsigned active;

  ints export_;
  ints units;
  imports import;
  extensions extend;
  unsigneds witness;

  assigned *assigned_;
  flags *flags_;

  mark *marks;

  value *values;
  phases phases_;

  eliminated eliminated_;
  unsigneds etrail;

  links *links_;
  queue queue_;

  heap scores;
  double scinc;

  unsigned level;
  frames frames_;

  unsigned_array trail;
  unsigned *propagate;

  unsigned best_assigned;
  unsigned target_assigned;
  unsigned unflushed;
  unsigned unassigned;

  unsigneds delayed;

#if defined(LOGGING) || !defined(NDEBUG)
  unsigneds resolvent;
#endif
  unsigned resolvent_size;
  unsigned antecedent_size;

  dataranks ranks;

  unsigneds analyzed;
  unsigneds levels;
  unsigneds minimize;
  unsigneds poisoned;
  unsigneds promote;
  unsigneds removable;
  unsigneds shrinkable;

  clause conflict;

  bool clause_satisfied;
  bool clause_shrink;
  bool clause_trivial;

  unsigneds clause_;
  unsigneds shadow;

  arena arena_;
  vectors vectors_;
  reference first_reducible;
  reference last_irredundant;
  watches *watches_;

  sizes sorter;

  generator random;
  averages averages_[2];
  reluctant reluctant_;

  bounds bounds_;
  delays delays_;
  enabled enabled_;
  effort last;
  limited limited_;
  limits limits_;
  waiting waiting_;
  unsigned walked;

  statistics statistics_;
  mode mode_;

  uint64_t ticks;

  format format_;

  statches antecedents[2];
  statches gates[2];
  patches xorted[2];
  unsigneds resolvents;
  bool resolve_gate;

  struct kitten *kitten;
#ifdef METRICS
  uint64_t *gate_eliminated;
#else
  bool gate_eliminated;
#endif
  unsigneds sweep;

#if !defined(NDEBUG) || !defined(NPROOFS)
  unsigneds added;
  unsigneds removed;
#endif

#if !defined(NDEBUG) || !defined(NPROOFS) || defined(LOGGING)
  ints original;
  size_t offset_of_last_original_clause;
#endif

#ifndef QUIET
  profiles profiles_;
#endif

#ifndef NOPTIONS
  options options_;
#endif

#ifndef NDEBUG
  checker *checker;
#endif

#ifndef NPROOFS
  proof *proof_;
#endif
};

#define VARS (solver->vars)
#define LITS (2*solver->vars)

#define SCORES (&solver->scores)

static inline unsigned
kissat_assigned (kissat * solver)
{
  assert (VARS >= solver->unassigned);
  return VARS - solver->unassigned;
}

#define all_variables(IDX) \
  unsigned IDX = 0, IDX ## _END = solver->vars; \
  IDX != IDX ## _END; \
  ++IDX

#define all_literals(LIT) \
  unsigned LIT = 0, LIT ## _END = LITS; \
  LIT != LIT ## _END; \
  ++LIT

#define all_clauses(C) \
  clause *       C         = (clause*) BEGIN_STACK (solver->arena_), \
         * const C ## _END = (clause*) END_STACK (solver->arena_), \
	 * C ## _NEXT; \
  C != C ## _END && (C ## _NEXT = kissat_next_clause (C), true); \
  C = C ## _NEXT

#endif
