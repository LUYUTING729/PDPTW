#ifndef AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_MEMETIC_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_MEMETIC_H_

#include "domain/model/VrpSolution.h"
#include "domain/model/entity/VrpConfig.h"

constexpr int kPopulationSize = 100;
constexpr int kMaxGeneration = 1000000;
constexpr int kThreadsNum = 12;
constexpr double kPenaltyCoeffUpper = 20000.0;
constexpr double kPenaltyCoeffLower = 1.0;
constexpr double kSrexProb = 0.1;

void buildInitialSolution(int routeNum, VrpSolution& vrpSolution);

void memetic(VrpConfig& vrpConfig, int routeNum, double timeLimit, const VrpSolution& bks,
             const std::vector<VrpSolution>& initialSols, VrpProblem& vrpProblem, VrpSolution& bestSolution,
             const std::string& resultFolder);

#endif