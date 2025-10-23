#ifndef AMDAHL_SRC_DOMAIN_MODEL_VRP_PROBLEM_H_
#define AMDAHL_SRC_DOMAIN_MODEL_VRP_PROBLEM_H_

#include <unordered_map>

#include "domain/model/entity/VrpData.h"

class VrpProblem {
   public:
    VrpData vrpData;
    std::vector<std::vector<int>> adjacentList;
    std::unordered_map<int, std::unordered_map<int, double>> requestDistanceMap;
    std::unordered_map<int, std::unordered_map<int, double>> requestDemandDiffMap;
};
#endif