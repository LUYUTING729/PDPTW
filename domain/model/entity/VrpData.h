#ifndef AMDAHL_SRC_DOMAIN_MODEL_ENTITY_VRP_DATA_H_
#define AMDAHL_SRC_DOMAIN_MODEL_ENTITY_VRP_DATA_H_

#include <string>
#include <vector>

#include "domain/model/entity/VrpNode.h"

class VrpData {
   public:
    int nodeNum;
    int vehicleUpLimit;
    double vehicleCapacity;
    std::vector<VrpNode> nodeList;
    std::vector<std::vector<double>> disMatrix;
    std::vector<int> pickNodeIds;
    std::string name;
};

#endif