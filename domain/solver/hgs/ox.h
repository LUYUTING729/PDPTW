#ifndef AMDAHL_SRC_DOMAIN_SOLVER_OX_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_OX_H_

#include <vector>
#include <unordered_set>
#include <queue>
#include <utility>
#include <unordered_map>
#include <iostream>


#include "domain/model/VrpProblem.h"
#include "domain/model/VrpSolution.h"
#include "domain/model/entity/VrpConfig.h"
#include "domain/model/entity/VrpRoute.h"
#include "domain/utils/Utils.h"
#include "util/distance_utils.h"

#include<cmath>

class orderXover {
   public:
    int fixRange;
    int OUTPUT=1;
    std::unordered_map<int,int> pickDeliPair;

    VrpProblem * vp;

    // Constructor
    orderXover(int range, VrpData * vd, VrpProblem * vp_ori){
        fixRange=range;
        for (int i=0; i<vd->pickNodeIds.size();++i){
            int pid = vd->pickNodeIds[i];
            int did = vd->nodeList[pid].corresNodeId;
            pickDeliPair.emplace(pid,did);
        }
        vp = vp_ori;
    };

    // Crossover func
    // 1. Find part in sol1 to fix
    // 2. Exclude nodes in fixed part, then add nodes as ordered in sol2
    std::vector<int> oxCross(const VrpSolution* sol1, const VrpSolution* sol2);

    // Crossover Utilities

    // getFixedPart takes the second solution in crossOver, fixed nodes 
    //      and return the indicies of nodes
    void getOrderedPart(const VrpSolution* sol2, const std::unordered_set<int> fixedNodes, std::vector<int> &res, int starting);

    // retInt takes in an arbitrary VrpSolution and return vector<int> of routeNodes
    std::vector<int> retInt(const VrpSolution* sol);


    //Shortest path decomposition method
    void decompose(const std::vector<int>& order, const VrpSolution* sol,  double cPen, double tPen);

    //Shortest Path algortihm (Shortest Path Decomposition ALgorithm)
    std::vector<int> spDA(const std::vector<std::vector<std::pair<float,int>>>& adj);

    void getAdj(const std::vector<int>& order, std::vector<std::vector<std::pair<float,int>>>& dMat, const VrpSolution* sol, const std::vector<VrpNode> & nodeList,  double capPen, double timePen, double capacity);

    VrpSolution constructSol(const std::vector<int> & vertices, const std::vector<int> & orderedNodes);

    void execute(const VrpSolution* sol1, const VrpSolution* sol2,  double cPen, double tPen){
        std::vector<int> reordered = oxCross(sol1,sol2);
        decompose(reordered,sol1,   cPen,  tPen);
    }
};
#endif