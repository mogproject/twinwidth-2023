#pragma once
#include <signal.h>
#include <unistd.h>

#include <cassert>
#include <unordered_map>
#include <vector>

extern "C" {
#include "external/kissat/internal.h"
}

#include "sat/card/SeqCounter.hpp"
#include "util/Random.hpp"
#include "util/logger.hpp"

namespace sat {

// alarm handlers
extern bool volatile sat_ignore_alarm;
extern kissat *volatile sat_solver;  // must be a global variable

void sat_alarm_handler(int sig);

namespace status {
/** SAT status. */
enum SATStatus { UNSOLVED = 0, SATISFIABLE = 10, INCONSISTENT = 20, INCONSISTENT_AND_CORE_COMPUTED = 21 };
}  // namespace status

/**
 * @brief Adapter for kissat solver with ID management.
 *
 * Note: This must be a singleton.
 */
class SATSolver {
 private:
  int status_;
  std::vector<int> witness_;

  // for ID management
  int top_id_;
  std::unordered_map<long long, int> id_manager_;

  // number of clauses
  int num_clauses_;

 public:
  SATSolver(util::Random *rand = nullptr) : status_(0), top_id_(0), num_clauses_(0) {
    if (sat_solver) kissat_release(sat_solver);
    sat_solver = kissat_init();
    if (rand) sat_solver->random = rand->randint(0, 2147483647);
  }

  ~SATSolver() {
    if (sat_solver) {
      kissat_release(sat_solver);
      sat_solver = nullptr;
    }
  }

  void restart(util::Random *rand = nullptr) {
    if (sat_solver) kissat_release(sat_solver);
    sat_solver = kissat_init();

    // set random seed

    // FIXME: something bad happens?
    if (rand) kissat_options_set(&sat_solver->options_, "seed", rand->randint(0, 0x7fffffff));

    id_manager_.clear();
    top_id_ = 0;
    num_clauses_ = 0;
  }

  void add_clause(std::vector<int> const &clause) {
    for (auto x : clause) kissat_add(sat_solver, x);
    kissat_add(sat_solver, 0);
    ++num_clauses_;
  }

  void add_clauses(std::vector<std::vector<int>> const &clauses) {
    for (auto &clause : clauses) add_clause(clause);
    num_clauses_ += clauses.size();
  }

  void add_equals_one(std::vector<int> const &lits) {
    std::vector<std::vector<int>> clauses;
    card::SeqCounter::encode_equals_one(clauses, lits);
    add_clauses(clauses);
  }

  void add_atmost(std::vector<int> const &lits, int bound, int top_id = -1, std::vector<int> additional_lits = {}) {
    top_id_ = std::max(top_id_, top_id);
    std::vector<std::vector<int>> clauses;
    card::SeqCounter::encode_atmost(top_id_, clauses, lits, bound);

    if (!additional_lits.empty()) {
      for (std::size_t i = 0; i < clauses.size(); ++i) {
        for (auto x : additional_lits) clauses[i].push_back(x);
      }
    }
    add_clauses(clauses);
  }

  void add_atleast(std::vector<int> const &lits, int bound, int top_id = -1, std::vector<int> additional_lits = {}) {
    top_id_ = std::max(top_id_, top_id);
    std::vector<std::vector<int>> clauses;
    card::SeqCounter::encode_atleast(top_id_, clauses, lits, bound);

    if (!additional_lits.empty()) {
      for (std::size_t i = 0; i < clauses.size(); ++i) {
        for (auto x : additional_lits) clauses[i].push_back(x);
      }
    }
    add_clauses(clauses);
  }

  bool get_witness(int var) {
    assert(status_ == status::SATISFIABLE);
    if (var < 0) {
      return kissat_value(sat_solver, -var) < 0;
    } else {
      return kissat_value(sat_solver, var) > 0;
    }
  }

  /**
   * @brief Issues a new variable.
   *
   * @param key unique key for the variable
   * @return int variable id
   */
  int id(long long key) {
    if (id_manager_.find(key) == id_manager_.end()) id_manager_[key] = ++top_id_;
    return id_manager_[key];
  }

  /**
   * @brief Solves the current instance.
   *
   * @param time_limit_sec time limit in seconds
   * @return int SAT status
   */
  int solve(int time_limit_sec = 0) {
    // log_trace("SATSolver solving: num_vars=%d, num_clauses=%d, time_limit=%ds", top_id_, num_clauses_, time_limit_sec);
    if (time_limit_sec > 0) {
      sat_ignore_alarm = false;
      signal(SIGALRM, sat_alarm_handler);
      alarm(time_limit_sec);
    }

    status_ = kissat_solve(sat_solver);

    if (time_limit_sec > 0) {
      sat_ignore_alarm = true;
      alarm(0);  // cancel previous alarm
      signal(SIGALRM, nullptr);
    }
    return status_;
  }
};
}  // namespace sat
