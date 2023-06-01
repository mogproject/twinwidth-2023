#ifndef _format_h_INCLUDED
#define _format_h_INCLUDED

#include "utilities.h"

#include <stdbool.h>
#include <stdint.h>

#define NUM_FORMAT_STRINGS 8
#define FORMAT_STRING_SIZE 128

typedef struct format format;

struct format
{
  unsigned pos;
  char str[NUM_FORMAT_STRINGS][FORMAT_STRING_SIZE];
};

char *kissat_next_format_string (format *);

char const *kissat_format_bytes (format *, uint64_t bytes);
char const *kissat_format_count (format *, uint64_t);
char const *kissat_format_ordinal (format *, uint64_t);
char const *kissat_format_signs (format *, unsigned size, word);
char const *kissat_format_time (format *, double seconds);
char const *kissat_format_value (format *, bool boolean, int value);

#define FORMAT_BYTES(BYTES) \
  kissat_format_bytes (&solver->format_, BYTES)

#define FORMAT_COUNT(WORD) \
  kissat_format_count (&solver->format_, WORD)

#define FORMAT_ORDINAL(WORD) \
  kissat_format_ordinal (&solver->format_, WORD)

#define FORMAT_SIGNS(SIZE, SIGNS) \
  kissat_format_signs (&solver->format_, SIZE, SIGNS)

#define FORMAT_TIME(SECONDS) \
  kissat_format_time (&solver->format_, SECONDS)

#define FORMAT_VALUE(BOOLEAN,VALUE) \
  kissat_format_value (&solver->format_, BOOLEAN, VALUE)

#endif
