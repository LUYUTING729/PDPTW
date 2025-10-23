#ifndef AMDAHL_SRC_DOMAIN_MODEL_ENTITY_VRP_CONFIG_H_
#define AMDAHL_SRC_DOMAIN_MODEL_ENTITY_VRP_CONFIG_H_

class VrpConfig {
   public:
    double vehicleCost{1E5};
    double twPenaltyCoeff{1E5};
    double capacityPenaltyCoeff{1E5};
};

#endif