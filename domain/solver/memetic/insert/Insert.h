#ifndef AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_INSERT_INSERT_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_MEMETIC_INSERT_INSERT_H_

#include "domain/model/VrpSolution.h"
#include "domain/model/entity/VrpConfig.h"

constexpr double kSortBasedOnRandomProbability = 0.4;
constexpr double kSortBasedOnDemandProbability = 0.3;
constexpr double kSortBasedOnFarDistanceProbability = 0.2;
constexpr double kSortBasedOnCloseDistanceProbability = 0.1;
constexpr double kBlinkRate = 0.01;

struct InsertionData {
    int routeId{-1};
    RouteNode* pickupPrevNode{nullptr};
    RouteNode* deliveryPrevNode{nullptr};
    double deltaTwPenalty{0.0};
    double deltaCapacityViolations{0.0};
    double deltaDistance{0.0};
    bool newVehicle{false};
};

void calInsertionData(const VrpSolution& vrpSolution, RouteNode* pickupNode, RouteNode* deliveryNode, int routeId,
                      std::vector<InsertionData>& insertions, bool allowInfeasible);

void bestInsert(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, bool allowInfeasible);

void greedyInsertWithBlinks(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, bool allowInfeasible);
#endif