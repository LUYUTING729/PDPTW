#ifndef AMDAHL_SRC_DOMAIN_SOLVER_EXACT_TWO_INDEX_FORMULATION_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_EXACT_TWO_INDEX_FORMULATION_H_

#include "domain/model/VrpProblem.h"
#include "domain/model/VrpSolution.h"

struct PricingSubProblemData {
    std::unordered_map<int, double> dualValueForPickupNodes;
    int pickupNodeNumLimitInRoute{0};
};

struct PricingSubProblemSolution {
    std::vector<int> nodesInRoute;
    double reducedCost{0.0};
};

void twoIndexFormulationSolve(const VrpConfig& vrpConfig, const VrpProblem* vrpProblem, const VrpSolution* initSolution,
                              bool freeNodes, int freeNodesNum, const std::vector<int>& freeRouteIndices,
                              double timeLimit, bool pricingSubProblem,
                              const PricingSubProblemData& pricingSubProblemData,
                              PricingSubProblemSolution& pricingSubProblemSolution, VrpSolution &vrpSolution,
                              const std::string& resultFolder);

#endif