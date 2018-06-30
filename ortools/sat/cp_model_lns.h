// Copyright 2010-2017 Google
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OR_TOOLS_SAT_CP_MODEL_LNS_H_
#define OR_TOOLS_SAT_CP_MODEL_LNS_H_

#include <vector>

#include "ortools/base/integral_types.h"
#include "ortools/sat/cp_model.pb.h"

namespace operations_research {
namespace sat {

// Base class for a CpModelProto neighborhood generator.
class NeighborhoodGenerator {
 public:
  explicit NeighborhoodGenerator(CpModelProto const* model,
                                 bool focus_on_decision_variables,
                                 const std::string& name);
  virtual ~NeighborhoodGenerator() {}

  // Generates a "local" subproblem for the given seed.
  //
  // The difficulty will be in [0, 1] and is related to the asked neighborhood
  // size (and thus local problem difficulty). A difficulty of 0.0 means empty
  // neighborhood and a difficulty of 1.0 means the full problem. The algorithm
  // should try to generate a neighborhood according to this difficulty which
  // will be dynamically adjusted depending on whether or not we can solve the
  // subproblem in a given time limit.
  //
  // The given initial_solution should contains a feasible solution to the
  // initial CpModelProto given to this class. Any solution to the returned
  // CPModelProto should also be valid solution to the same initial model.
  //
  // This function should be thread-safe.
  virtual CpModelProto Generate(const CpSolverResponse& initial_solution,
                                int64 seed, double difficulty) const = 0;

  // Returns a short description of the generator.
  std::string name() const { return name_; }

 protected:
  // Indicates if the variable can be frozen. It happens if the variable is non
  // constant, and if it is a decision variable, or if
  // focus_on_decision_variables is false.
  bool IsActive(int var) const;

  // Indicates if a variable is fixed in the model.
  bool IsConstant(int var) const;

  // Returns an LNS fragment which will relax all inactive variables and all
  // variables in relaxed_variables.
  CpModelProto RelaxGivenVariables(
      const CpSolverResponse& initial_solution,
      const std::vector<int>& relaxed_variables) const;

  // TODO(user): for now and for convenience, we generate the
  // variable-constraint graph even if not all subclass will need it.
  const CpModelProto& model_proto_;

  const std::string name_;

  // Variable-Constraint graph.
  std::vector<std::vector<int>> constraint_to_var_;
  std::vector<std::vector<int>> var_to_constraint_;

  // The set of active variables, that is the list of non constant variables
  // if focus_on_decision_variables_ is false, or the list of non constant
  // decision variables otherwise.
  // It is stored both as a list and as a set (using a Boolean vector).
  std::vector<bool> active_variables_set_;
  std::vector<int> active_variables_;
};

// Pick a random subset of variables.
class SimpleNeighborhoodGenerator : public NeighborhoodGenerator {
 public:
  SimpleNeighborhoodGenerator(CpModelProto const* model,
                              bool focus_on_decision_variables,
                              const std::string& name)
      : NeighborhoodGenerator(model, focus_on_decision_variables, name) {}
  CpModelProto Generate(const CpSolverResponse& initial_solution, int64 seed,
                        double difficulty) const final;
};

// Pick a random subset of variables that are constructed by a BFS in the
// variable <-> constraint graph. That is, pick a random variable, then all the
// variable connected by some constraint to the first one, and so on. The
// variable of the last "level" are selected randomly.
class VariableGraphNeighborhoodGenerator : public NeighborhoodGenerator {
 public:
  explicit VariableGraphNeighborhoodGenerator(CpModelProto const* model,
                                              bool focus_on_decision_variables,
                                              const std::string& name)
      : NeighborhoodGenerator(model, focus_on_decision_variables, name) {}
  CpModelProto Generate(const CpSolverResponse& initial_solution, int64 seed,
                        double difficulty) const final;
};

// Pick a random subset of constraint and relax all of their variables. We are a
// bit smarter than this because after the first contraint is selected, we only
// select constraints that share at least one variable with the already selected
// constraints. The variable from the "last" constraint are selected randomly.
class ConstraintGraphNeighborhoodGenerator : public NeighborhoodGenerator {
 public:
  explicit ConstraintGraphNeighborhoodGenerator(
      CpModelProto const* model, bool focus_on_decision_variables,
      const std::string& name)
      : NeighborhoodGenerator(model, focus_on_decision_variables, name) {}
  CpModelProto Generate(const CpSolverResponse& initial_solution, int64 seed,
                        double difficulty) const final;
};

class ObjectiveGraphNeighborhoodGenerator : public NeighborhoodGenerator {
 public:
  explicit ObjectiveGraphNeighborhoodGenerator(CpModelProto const* model,
                                               bool focus_on_decision_variables,
                                               int max_variables_per_constraint,
                                               const std::string& name)
      : NeighborhoodGenerator(model, focus_on_decision_variables, name),
        max_variables_per_constraint_(max_variables_per_constraint) {}
  CpModelProto Generate(const CpSolverResponse& initial_solution, int64 seed,
                        double difficulty) const final;

 private:
  const int max_variables_per_constraint_;
};


}  // namespace sat
}  // namespace operations_research

#endif  // OR_TOOLS_SAT_CP_MODEL_LNS_H_