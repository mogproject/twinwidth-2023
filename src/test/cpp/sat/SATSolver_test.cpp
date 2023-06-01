#include <gtest/gtest.h>

#include "sat/SATSolver.hpp"

using namespace std;
using namespace sat;

TEST(SATSolverTest, SolveSatisfiable) {
  SATSolver solver;
  EXPECT_EQ(solver.solve(), sat::status::SATISFIABLE);

  solver.restart();
  solver.add_clause({1});
  EXPECT_EQ(solver.solve(), sat::status::SATISFIABLE);

  solver.restart();
  solver.add_clause({-1, -2, -3});
  solver.add_clause({1, 2, 3});
  solver.add_clause({-3, 2});
  EXPECT_EQ(solver.solve(), sat::status::SATISFIABLE);
}

TEST(SATSolverTest, SolveInconsistent) {
  SATSolver solver;

  solver.add_clause({1});
  solver.add_clause({-1});
  EXPECT_EQ(solver.solve(), sat::status::INCONSISTENT);

  solver.restart();
  solver.add_clause({-1, -2, -3});
  solver.add_clause({1, 2, 3});
  solver.add_clause({-3, 2});
  solver.add_clause({3, -2});
  solver.add_clause({1, -2});
  solver.add_clause({3, -1});
  EXPECT_EQ(solver.solve(), sat::status::INCONSISTENT);
}

TEST(SATSolverTest, Id) {
  SATSolver solver;

  EXPECT_EQ(solver.id(1000000000000L), 1);
  EXPECT_EQ(solver.id(2000000000000L), 2);
  EXPECT_EQ(solver.id(2000000000000L), 2);
  EXPECT_EQ(solver.id(1L), 3);
  EXPECT_EQ(solver.id(1L), 3);
  EXPECT_EQ(solver.id(1000000000000L), 1);
  EXPECT_EQ(solver.id(4294967297L), 4);  // 2^32 + 1
}

TEST(SATSolverTest, GetWitness) {
  SATSolver solver;

  solver.add_clause({3});
  solver.add_clause({-3, -2});
  solver.add_clause({2, 1});
  EXPECT_EQ(solver.solve(), sat::status::SATISFIABLE);
  EXPECT_TRUE(solver.get_witness(1));
  EXPECT_FALSE(solver.get_witness(2));
  EXPECT_TRUE(solver.get_witness(3));
}

TEST(SATSolverTest, AddAtmost) {
  SATSolver solver;

  // encode vertex cover
  std::vector<std::pair<int, int>> edges = {{1, 2}, {1, 5}, {2, 3}, {2, 4}, {3, 5}, {4, 5}};

  for (auto &p : edges) solver.add_atleast({p.first, p.second}, 1, 5);
  solver.add_atmost({1, 2, 3, 4, 5}, 0);
  EXPECT_EQ(solver.solve(), sat::status::INCONSISTENT);

  solver.restart();
  for (auto &p : edges) solver.add_atleast({p.first, p.second}, 1, 5);
  solver.add_atmost({1, 2, 3, 4, 5}, 1);
  EXPECT_EQ(solver.solve(), sat::status::INCONSISTENT);

  solver.restart();
  for (auto &p : edges) solver.add_atleast({p.first, p.second}, 1, 5);
  solver.add_atmost({1, 2, 3, 4, 5}, 2);
  EXPECT_EQ(solver.solve(), sat::status::SATISFIABLE);
  EXPECT_FALSE(solver.get_witness(1));
  EXPECT_TRUE(solver.get_witness(2));
  EXPECT_FALSE(solver.get_witness(3));
  EXPECT_FALSE(solver.get_witness(4));
  EXPECT_TRUE(solver.get_witness(5));

  solver.restart();
  for (auto &p : edges) solver.add_atleast({p.first, p.second}, 1, 5);
  solver.add_atmost({1, 2, 3, 4, 5}, 3);
  EXPECT_EQ(solver.solve(), sat::status::SATISFIABLE);

  solver.restart();
  for (auto &p : edges) solver.add_atleast({p.first, p.second}, 1, 5);
  solver.add_atmost({1, 2, 3, 4, 5}, 4);
  EXPECT_EQ(solver.solve(), sat::status::SATISFIABLE);

  solver.restart();
  for (auto &p : edges) solver.add_atleast({p.first, p.second}, 1, 5);
  solver.add_atmost({1, 2, 3, 4, 5}, 5);
  EXPECT_EQ(solver.solve(), sat::status::SATISFIABLE);

  solver.restart();
  for (auto &p : edges) solver.add_atleast({p.first, p.second}, 1, 5);
  solver.add_atmost({1, 2, 3, 4, 5}, 6);
  EXPECT_EQ(solver.solve(), sat::status::SATISFIABLE);

  // another vertex cover instance
  std::vector<std::pair<int, int>> edges2 = {{1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}};

  for (int d = 0; d <= 6; ++d) {
    solver.restart();
    for (auto &p : edges2) solver.add_atleast({p.first, p.second}, 1, 6);
    solver.add_atmost({1, 2, 3, 4, 5, 6}, d);
    EXPECT_EQ(solver.solve(), d < 3 ? sat::status::INCONSISTENT : sat::status::SATISFIABLE);
  }
}

TEST(SATSolverTest, AddEqualsOne) {
  SATSolver solver;

  solver.add_equals_one({1, 2, 3});
  solver.add_equals_one({4, 5, 6});
  solver.add_equals_one({7, 8, 9});

  solver.add_equals_one({1, 4, 7});
  solver.add_equals_one({2, 5, 8});
  solver.add_equals_one({3, 6, 9});

  solver.add_clause({-1});
  solver.add_clause({-2});
  solver.add_clause({-4});

  EXPECT_EQ(solver.solve(), sat::status::SATISFIABLE);

  solver.restart();
  solver.add_equals_one({1, 2, 3});
  solver.add_equals_one({4, 5, 6});
  solver.add_equals_one({7, 8, 9});

  solver.add_equals_one({1, 4, 7});
  solver.add_equals_one({2, 5, 8});
  solver.add_equals_one({3, 6, 9});

  solver.add_clause({-1});
  solver.add_clause({-2});
  solver.add_clause({-4});
  solver.add_clause({-5});

  EXPECT_EQ(solver.solve(), sat::status::INCONSISTENT);
}
