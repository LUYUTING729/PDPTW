#ifndef AMDAHL_SRC_DOMAIN_SOLVER_EXACT_TWO_IND_CHECKER_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_EXACT_TWO_IND_CHECKER_H_

#include "domain/model/VrpProblem.h"
#include "domain/model/VrpSolution.h"
#include "domain/solver/exact/TwoIndexFormulation.h"

void twoIndSolve(const VrpConfig& vrpConfig, const VrpProblem* vrpProblem, const VrpSolution* initSolution,
                              bool freeNodes, int freeNodesNum, const std::vector<int>& freeRouteIndices,
                              double timeLimit, bool pricingSubProblem,
                              const PricingSubProblemData& pricingSubProblemData,
                              PricingSubProblemSolution& pricingSubProblemSolution, VrpSolution &vrpSolution,
                              const std::string& resultFolder);

double twoIndexFormulationSolveZ(const VrpConfig& vrpConfig, const VrpProblem* vrpProblem, const VrpSolution* initSolution,
                              bool freeNodes, int freeNodesNum, const std::vector<int>& freeRouteIndices,
                              double timeLimit, bool pricingSubProblem,
                              const PricingSubProblemData& pricingSubProblemData,
                              PricingSubProblemSolution& pricingSubProblemSolution, VrpSolution &vrpSolution,
                              const std::string& resultFolder, std::vector<std::vector<int> >& sol);

#endif