#ifndef _propdense_h_INCLUDED
#define _propdense_h_INCLUDED
#include <stdbool.h>

struct kissat;

#include <limits.h>

bool kissat_dense_propagate (struct kissat *);

#endif
