#include "domain/solver/exact/TwoIndChecker.h"

#include <gurobi_c++.h>

#include <cassert>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <unordered_map>

#include "domain/solver/memetic/removal/Removal.h"
#include "io/vrp_sol_writer.h"

double twoIndexFormulationSolveZ(const VrpConfig& vrpConfig, const VrpProblem* vrpProblem, const VrpSolution* initSolution,
                              bool freeNodes, int freeNodesNum, const std::vector<int>& freeRouteIndices,
                              double timeLimit, bool pricingSubProblem,
                              const PricingSubProblemData& pricingSubProblemData,
                              PricingSubProblemSolution& pricingSubProblemSolution, VrpSolution& vrpSolution,
                              const std::string& resultFolder, std::vector<std::vector<int>>& solVec) {

    if (nullptr != initSolution) {
        assert(initSolution->unassignedNodeIds.empty());
        assert(initSolution->totalCapacityViolation <= 0.0);
        assert(initSolution->totalTwViolation <= 0.0);
        vrpSolution.copySolution(*initSolution);
    }

    std::unordered_map<int, int> originalNewNodeIndexMap;
    std::unordered_map<int, int> newOriginalNodeIndexMap;
    int pickupNodeCounter = 0;
    int pickupNodeNum = (int)vrpProblem->vrpData.pickNodeIds.size();
    for (int i = 1; i < vrpProblem->vrpData.nodeNum; ++i) {
        if (Utils::VrpNodeType::kPickup == vrpProblem->vrpData.nodeList[i].nodeType) {
            pickupNodeCounter += 1;
            originalNewNodeIndexMap[i] = pickupNodeCounter;
            newOriginalNodeIndexMap[pickupNodeCounter] = i;

            int tmpDeliveryNodeIndex = vrpProblem->vrpData.nodeList[i].corresNodeId;
            originalNewNodeIndexMap[tmpDeliveryNodeIndex] = pickupNodeCounter + pickupNodeNum;
            newOriginalNodeIndexMap[pickupNodeCounter + pickupNodeNum] = tmpDeliveryNodeIndex;

            if (false && pricingSubProblem) {
                if (pricingSubProblemData.dualValueForPickupNodes.find(i) ==
                    pricingSubProblemData.dualValueForPickupNodes.end()) {
                    std::cerr << "Dual value for pickup node " << i << " not found" << std::endl;
                    exit(0);
                }
            }
        }
    }

    int totalPickupNodeNumInGraph = (int)vrpProblem->vrpData.pickNodeIds.size();
    int totalNodeNumInGraph = vrpProblem->vrpData.nodeNum + 1;

    std::vector<double> nodeSericeStartTimes(totalNodeNumInGraph,0.0);
    for (int i = 0; i < initSolution->vehicleNum; ++i) {
        RouteNode *currentNode = initSolution->routes[i].startNode;
        int j=0;
        while (currentNode != NULL) {
            nodeSericeStartTimes[currentNode->nodeId] = currentNode->latestStartServiceTime;
            currentNode = currentNode->nexNode;
            j++;
        }
    }

    std::vector<std::vector<GRBVar>> arcVariables;
    arcVariables.resize(totalNodeNumInGraph);
    std::vector<std::vector<bool>> arcValid;
    arcValid.resize(totalNodeNumInGraph);
    std::vector<GRBVar> pathVertexVariables; // z_variables
    pathVertexVariables.resize(totalNodeNumInGraph);
    for (int i = 0; i < totalNodeNumInGraph; ++i) {
        arcVariables[i].resize(totalNodeNumInGraph);
        arcValid[i].resize(totalNodeNumInGraph,0);
    }
    std::vector<GRBVar> startServiceTimeVariables;
    startServiceTimeVariables.resize(totalNodeNumInGraph);
    std::vector<GRBVar> vehicleLoadVariables;
    vehicleLoadVariables.resize(totalNodeNumInGraph);
    std::vector<GRBVar> routeIdentifierVariables;
    routeIdentifierVariables.resize(totalNodeNumInGraph);

    GRBVar* vars = 0;
    double objAdj = 0.0;

    try {
        GRBEnv env = GRBEnv(true);
        std::string logFileName = "two_index_formulation";
        if (pricingSubProblem) {
            logFileName.append("_pricing_sub_problem");
        } else {
            logFileName.append("_original");
        }
        logFileName.append(".log");
        env.set("LogFile", logFileName);
        env.set("MIPGap", "0.0");
        //env.set("MIPFocus", "1");
        env.set("TimeLimit", std::to_string(timeLimit));
        env.start();

        GRBModel model = GRBModel(env);

        for (int i = 0; i < totalNodeNumInGraph; ++i) {
            int tmpFirstOriginalNodeIndex = 0;
            if (1 <= i && i < totalNodeNumInGraph - 1) {
                tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            }

            for (int j = 0; j < totalNodeNumInGraph; ++j) {
                double tmpInfeasibleArc = false;
                if (i == j) {
                    tmpInfeasibleArc = true;
                }
                if (!tmpInfeasibleArc && 0 == j) {
                    tmpInfeasibleArc = true;
                }
                if (!tmpInfeasibleArc && totalNodeNumInGraph - 1 == i) {
                    tmpInfeasibleArc = true;
                }
                if (!tmpInfeasibleArc && 1 <= i && i <= totalPickupNodeNumInGraph && totalNodeNumInGraph - 1 == j) {
                    tmpInfeasibleArc = true;
                }
                if (!tmpInfeasibleArc && i < totalNodeNumInGraph - 1 && i > totalPickupNodeNumInGraph &&
                    i - j == totalPickupNodeNumInGraph) {
                    tmpInfeasibleArc = true;
                }
                int tmpSecondOriginalNodeIndex = 0;
                if (1 <= j && j < totalNodeNumInGraph - 1) {
                    tmpSecondOriginalNodeIndex = newOriginalNodeIndexMap[j];
                }

                if (!tmpInfeasibleArc) {
                    if (vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].candiNextNodes.find(
                            tmpSecondOriginalNodeIndex) ==
                        vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].candiNextNodes.end()) {
                        tmpInfeasibleArc = true;
                    }
                }

                double varUpperBound = 1.0;
                if (tmpInfeasibleArc) {
                    varUpperBound = 0.0;
                }
                arcValid[i][j] = !tmpInfeasibleArc;

                double objCoeff = vrpProblem->vrpData.disMatrix[tmpFirstOriginalNodeIndex][tmpSecondOriginalNodeIndex];
                if (i==0)
                    objCoeff += pricingSubProblem
                            ? - pricingSubProblemData.dualValueForPickupNodes.find(0)->second : 10000.0;
                else if (j < totalNodeNumInGraph - 1 &&
                         nodeSericeStartTimes[tmpFirstOriginalNodeIndex] > nodeSericeStartTimes[tmpSecondOriginalNodeIndex] )
                    objCoeff += 10;

                std::string varName = "x_" + std::to_string(i) + "_" + std::to_string(j);

                GRBVar newVar = model.addVar(0.0, varUpperBound, objCoeff, GRB_BINARY, varName);

                arcVariables[i][j] = newVar;
            }
            // add z_ variables
            if (pricingSubProblem) {
                double objCoeffz=0.0,bd=1.0;
                std::string varNamez = "z_" + std::to_string(i);
                if (i > 0 && i <= totalPickupNodeNumInGraph)
                    objCoeffz = -pricingSubProblemData.dualValueForPickupNodes.find(tmpFirstOriginalNodeIndex)->second;
                if (objCoeffz >= -0) bd = 0.0;
                GRBVar newVarz = model.addVar(0.0, bd, objCoeffz, GRB_BINARY, varNamez);
                pathVertexVariables[i] = newVarz;
            }
        }

        int countofbigwin=0;
        std::vector<bool> nodesWithBigWindow(totalNodeNumInGraph+1,false);
        for (int i = 0; i < totalNodeNumInGraph; ++i) {
            std::string varName = "b_" + std::to_string(i);

            int tmpFirstOriginalNodeIndex = 0;
            if (1 <= i && i < totalNodeNumInGraph - 1) {
                tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            }
            double varLower = vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].timeWindow.first;
            double varUpper = vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].timeWindow.second;
            if (varUpper-varLower > 1000) {
                std::cout << "station with big window: " << tmpFirstOriginalNodeIndex << std::endl;
                nodesWithBigWindow[i] = true;
                countofbigwin++;
            }
            double coeff = 0.0;
            if (0 == i) {
                varUpper = varLower;
            } else if (i < totalNodeNumInGraph - 1)
            {
                coeff = 1.0/10000.0;
                objAdj += varLower / 10000;
                varLower = std::max(varLower, nodeSericeStartTimes[tmpFirstOriginalNodeIndex] - 200);
                varUpper = std::min(varUpper, nodeSericeStartTimes[tmpFirstOriginalNodeIndex] + 200);
            }

            GRBVar newVar = model.addVar(varLower, varUpper, coeff, GRB_CONTINUOUS, varName);
            startServiceTimeVariables[i] = newVar;
        }
        std::cout << "big window count: " << countofbigwin << std::endl;

        for (int i = 0; i < totalNodeNumInGraph; ++i) {
            std::string varName = "q_" + std::to_string(i);

            int tmpFirstOriginalNodeIndex = 0;
            if (1 <= i && i < totalNodeNumInGraph - 1) {
                tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            }

            double varLower = std::max(0.0, vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].demand);
            double varUpper = std::min(
                vrpProblem->vrpData.vehicleCapacity,
                vrpProblem->vrpData.vehicleCapacity + vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].demand);

            GRBVar newVar = model.addVar(varLower, varUpper, 0.0, GRB_CONTINUOUS, varName);
            vehicleLoadVariables[i] = newVar;
        }

        for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
            double varLower = 1.0;
            double varUpper = totalPickupNodeNumInGraph;
            std::string varName = "v_" + std::to_string(i);

            GRBVar newVar = model.addVar(varLower, varUpper, 0.0, GRB_CONTINUOUS, varName);
            routeIdentifierVariables[i] = newVar;
        }

        if (pricingSubProblem) {
            for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
                int ii = i;
                if (i >= totalNodeNumInGraph / 2) ii = i - totalNodeNumInGraph / 2 + 1;
                GRBLinExpr linExpr = - pathVertexVariables[ii];
                for (int j = 0; j < totalNodeNumInGraph - 1; ++j) {
                    linExpr += arcVariables[j][i];
                }
                std::string constrName = "in_degree_" + std::to_string(i);
                model.addConstr(linExpr, GRB_EQUAL, 0.0, constrName);
            }

            for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
                int ii = i;
                if (i >= totalNodeNumInGraph / 2) ii = i - totalNodeNumInGraph / 2 + 1;
                GRBLinExpr linExpr = -pathVertexVariables[ii];
                for (int j = 1; j < totalNodeNumInGraph; ++j) {
                    linExpr += arcVariables[i][j];
                }
                std::string constrName = "out_degree_" + std::to_string(i);
                model.addConstr(linExpr, GRB_EQUAL, 0.0, constrName);
            }
        } else {
            for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
                GRBLinExpr linExpr;
                for (int j = 0; j < totalNodeNumInGraph - 1; ++j) {
                    linExpr += arcVariables[j][i];
                }
                std::string constrName = "in_degree_" + std::to_string(i);
                model.addConstr(linExpr, GRB_EQUAL, 1.0, constrName);
            }

            for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
                GRBLinExpr linExpr;
                for (int j = 1; j < totalNodeNumInGraph; ++j) {
                    linExpr += arcVariables[i][j];
                }
                std::string constrName = "out_degree_" + std::to_string(i);
                model.addConstr(linExpr, GRB_EQUAL, 1.0, constrName);
            }
        }

        for (int i = 0; i < totalNodeNumInGraph - 1; ++i) {
            GRBVar firstStartServiceVar = startServiceTimeVariables[i];
            int tmpFirstOriginalNodeIndex = 0;
            if (1 <= i && i < totalNodeNumInGraph - 1) {
                tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            }
            const VrpNode& tmpFirstNode = vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex];

            for (int j = 1; j < totalNodeNumInGraph; ++j) {
                if (i == j) {
                    continue;
                }

                GRBVar secondStartServiceVar = startServiceTimeVariables[j];

                int tmpSecondOriginalNodeIndex = 0;
                if (1 <= j && j < totalNodeNumInGraph - 1) {
                    tmpSecondOriginalNodeIndex = newOriginalNodeIndexMap[j];
                }

                const VrpNode& tmpSecondNode = vrpProblem->vrpData.nodeList[tmpSecondOriginalNodeIndex];

                double tmpTravelTime =
                    vrpProblem->vrpData.disMatrix[tmpFirstOriginalNodeIndex][tmpSecondOriginalNodeIndex] +
                    tmpFirstNode.serveTime;
                double tmpBigM = tmpFirstNode.timeWindow.second - tmpSecondNode.timeWindow.first + tmpTravelTime;

                GRBLinExpr linExpr;
                linExpr += firstStartServiceVar;
                linExpr -= secondStartServiceVar;
                linExpr += tmpBigM * arcVariables[i][j];
                std::string constrName = "start_service_" + std::to_string(i) + "_" + std::to_string(j);
                model.addConstr(linExpr, GRB_LESS_EQUAL, tmpBigM - tmpTravelTime, constrName);
            }
        }

        for (int i = 0; i < totalNodeNumInGraph - 1; ++i) {
            GRBVar firstVehicleLoadVar = vehicleLoadVariables[i];
            int tmpFirstOriginalNodeIndex = 0;
            if (1 <= i && i < totalNodeNumInGraph - 1) {
                tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            }
            const VrpNode& tmpFirstNode = vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex];

            for (int j = 0; j < totalNodeNumInGraph; ++j) {
                if (i == j) {
                    continue;
                }

                GRBVar secondVehicleLoadVar = vehicleLoadVariables[j];

                int tmpSecondOriginalNodeIndex = 0;
                if (1 <= j && j < totalNodeNumInGraph - 1) {
                    tmpSecondOriginalNodeIndex = newOriginalNodeIndexMap[j];
                }

                const VrpNode& tmpSecondNode = vrpProblem->vrpData.nodeList[tmpSecondOriginalNodeIndex];

                double tmpDemand = tmpSecondNode.demand;
                double tmpBigM = vrpProblem->vrpData.vehicleCapacity + tmpSecondNode.demand;

                GRBLinExpr linExpr;
                linExpr += firstVehicleLoadVar;
                linExpr -= secondVehicleLoadVar;
                linExpr += tmpBigM * arcVariables[i][j];
                std::string constrName = "vehicle_load_" + std::to_string(i) + "_" + std::to_string(j);
                model.addConstr(linExpr, GRB_LESS_EQUAL, tmpBigM - tmpDemand, constrName);
            }
        }

        for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
            GRBVar pickupStartServiceTime = startServiceTimeVariables[i];
            GRBVar deliveryStartServiceTime = startServiceTimeVariables[i + totalPickupNodeNumInGraph];

            int tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            int tmpSecondOriginalNodeIndex = newOriginalNodeIndexMap[i + totalPickupNodeNumInGraph];

            double tmpTravelTime =
                vrpProblem->vrpData.disMatrix[tmpFirstOriginalNodeIndex][tmpSecondOriginalNodeIndex] +
                vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].serveTime;
            GRBLinExpr linExpr;
            linExpr += pickupStartServiceTime;
            linExpr -= deliveryStartServiceTime;

            std::string constrName = "pd_prece_" + std::to_string(i);
            model.addConstr(linExpr, GRB_LESS_EQUAL, -tmpTravelTime, constrName);
            if (tmpTravelTime==0)
                model.addConstr(arcVariables[i][i + totalPickupNodeNumInGraph] - pathVertexVariables[i], GRB_EQUAL, 0.0,
                            "fix_" + std::to_string(i));
        }

        for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
            GRBVar pickupRouteIdentifierVar = routeIdentifierVariables[i];
            GRBVar deliveryRouteIdentifierVar = routeIdentifierVariables[i + totalPickupNodeNumInGraph];

            GRBLinExpr linExpr;
            linExpr += pickupRouteIdentifierVar;
            linExpr -= deliveryRouteIdentifierVar;

            std::string constrName = "pd_route_id_" + std::to_string(i);
            //model.addConstr(linExpr, GRB_EQUAL, 0.0, constrName);
        }

        for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
            GRBVar routeIdentifierVar = routeIdentifierVariables[i];

            GRBLinExpr linExpr;
            linExpr += routeIdentifierVar;
            linExpr -= i * arcVariables[0][i];

            std::string constrName = "route_id_first_l_" + std::to_string(i);
            //model.addConstr(linExpr, GRB_GREATER_EQUAL, 0.0, constrName);
        }

        for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
            GRBVar routeIdentifierVar = routeIdentifierVariables[i];

            GRBLinExpr linExpr;
            linExpr += routeIdentifierVar;
            linExpr += (totalPickupNodeNumInGraph - i) * arcVariables[0][i];

            std::string constrName = "route_id_first_u_" + std::to_string(i);
            //model.addConstr(linExpr, GRB_LESS_EQUAL, totalPickupNodeNumInGraph, constrName);
        }

        for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
            GRBVar firstRouteIdentifierVar = routeIdentifierVariables[i];

            for (int j = 1; j < totalNodeNumInGraph - 1; ++j) {
                if (j == i) {
                    continue;
                }

                GRBVar secondRouteIdentifierVar = routeIdentifierVariables[j];

                GRBLinExpr linExpr;
                linExpr += firstRouteIdentifierVar;
                linExpr -= secondRouteIdentifierVar;
                linExpr += totalPickupNodeNumInGraph * arcVariables[i][j];

                std::string constrName = "route_id_l_" + std::to_string(i) + "_" + std::to_string(j);
                //model.addConstr(linExpr, GRB_LESS_EQUAL, totalPickupNodeNumInGraph, constrName);
            }
        }

        for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
            GRBVar firstRouteIdentifierVar = routeIdentifierVariables[i];

            for (int j = 1; j < totalNodeNumInGraph - 1; ++j) {
                if (j == i) {
                    continue;
                }

                GRBVar secondRouteIdentifierVar = routeIdentifierVariables[j];

                GRBLinExpr linExpr;
                linExpr += firstRouteIdentifierVar;
                linExpr -= secondRouteIdentifierVar;
                linExpr -= totalPickupNodeNumInGraph * arcVariables[i][j];

                std::string constrName = "route_id_u_" + std::to_string(i) + "_" + std::to_string(j);
               // model.addConstr(linExpr, GRB_GREATER_EQUAL, -totalPickupNodeNumInGraph, constrName);
            }
        }

        if (pricingSubProblem) {
            GRBLinExpr linExpr1;
            for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
                linExpr1 += arcVariables[0][i];
            }
            std::string constrName = "single_route_from_depot";
            model.addConstr(linExpr1, GRB_EQUAL, 1.0, constrName);

            GRBLinExpr linExpr2;
            for (int i = totalPickupNodeNumInGraph + 1; i < totalNodeNumInGraph - 1; ++i) {
                linExpr2 += arcVariables[i][totalNodeNumInGraph - 1];
            }
            constrName = "single_route_to_depot";
            model.addConstr(linExpr2, GRB_EQUAL, 1.0, constrName);

            GRBLinExpr linExpr3;
            for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
                linExpr3 += pathVertexVariables[i];
            }
            constrName = "route_length_limit";
            model.addConstr(linExpr3, GRB_LESS_EQUAL, pricingSubProblemData.pickupNodeNumLimitInRoute, constrName);
        }

        for (int i = 1; i <= pickupNodeNum; ++i) {
            for (int j = i+1; j <= pickupNodeNum; ++j) {
                if (j == i) {
                    continue;
                }
                if (arcValid[i][j] || arcValid[j][i])
                    continue;
                if (arcValid[i][j + pickupNodeNum] || arcValid[j + pickupNodeNum][i])
                    continue;
                if (arcValid[i + pickupNodeNum][j] || arcValid[j][i + pickupNodeNum])
                    continue;
                if (arcValid[i + pickupNodeNum][j + pickupNodeNum] || arcValid[j + pickupNodeNum][i + pickupNodeNum])
                    continue;
                GRBLinExpr linExpr;
                linExpr = pathVertexVariables[i] + pathVertexVariables[j];
                std::string constrName = "cut" + std::to_string(i) + "_" + std::to_string(j);
                model.addConstr(linExpr, GRB_LESS_EQUAL, 1, constrName);
            }
        }

        std::vector<bool> threeNodeCut(pickupNodeNum*pickupNodeNum*pickupNodeNum,false);
        int pick1, pick2, pick3; // skipping big windows
        for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
            pick1 = i>pickupNodeNum? i - pickupNodeNum: i;
            int tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            double earlistStart1 = vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].timeWindow.first;
            double latestStart1 = vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].timeWindow.second;
            if (nodesWithBigWindow[i])
                continue;
            for (int j = 1; j < totalNodeNumInGraph - 1; ++j) {
                if (j == i || nodesWithBigWindow[j]) {
                    continue;
                }
                int tmpSecondOriginalNodeIndex = newOriginalNodeIndexMap[j];
                double earlistStart2 = vrpProblem->vrpData.nodeList[tmpSecondOriginalNodeIndex].timeWindow.first;
                double latestStart2 = vrpProblem->vrpData.nodeList[tmpSecondOriginalNodeIndex].timeWindow.second;
                pick2 = j>pickupNodeNum? j - pickupNodeNum: j;
                bool i_2_jOnly = earlistStart1 + vrpProblem->vrpData.disMatrix[tmpFirstOriginalNodeIndex][tmpSecondOriginalNodeIndex] +
                                 vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].serveTime <= latestStart2 &&
                                 earlistStart2 + vrpProblem->vrpData.disMatrix[tmpSecondOriginalNodeIndex][tmpFirstOriginalNodeIndex] +
                                 vrpProblem->vrpData.nodeList[tmpSecondOriginalNodeIndex].serveTime > latestStart1;
                if (!i_2_jOnly)
                    continue;

                for (int k = 1; k < totalNodeNumInGraph - 1; ++k) {
                    if (j == k || i == k || nodesWithBigWindow[k]) {
                        continue;
                    }
                    int tmpThirdOriginalNodeIndex = newOriginalNodeIndexMap[k];
                    double earlistStart3 = vrpProblem->vrpData.nodeList[tmpThirdOriginalNodeIndex].timeWindow.first;
                    double latestStart3 = vrpProblem->vrpData.nodeList[tmpThirdOriginalNodeIndex].timeWindow.second;
                    pick3 = k>pickupNodeNum? k - pickupNodeNum: k;
                    bool j_2_kOnly = earlistStart2 + vrpProblem->vrpData.disMatrix[tmpSecondOriginalNodeIndex][tmpThirdOriginalNodeIndex] +
                                             vrpProblem->vrpData.nodeList[tmpSecondOriginalNodeIndex].serveTime <= latestStart3 &&
                                     earlistStart3 + vrpProblem->vrpData.disMatrix[tmpThirdOriginalNodeIndex][tmpSecondOriginalNodeIndex] +
                                             vrpProblem->vrpData.nodeList[tmpThirdOriginalNodeIndex].serveTime > latestStart2;
                    if (!j_2_kOnly)
                        continue;

                    double tmpTravelTime =
                        vrpProblem->vrpData.disMatrix[tmpFirstOriginalNodeIndex][tmpSecondOriginalNodeIndex] +
                        vrpProblem->vrpData.disMatrix[tmpSecondOriginalNodeIndex][tmpThirdOriginalNodeIndex] +
                        vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].serveTime +
                        vrpProblem->vrpData.nodeList[tmpSecondOriginalNodeIndex].serveTime;

                    if (earlistStart1 + tmpTravelTime > latestStart3) // if there is a time violation
                    {
                        // sort the pick1, pick2, pick3
                        int node1=std::min(std::min(pick1,pick2), pick3);
                        int node3=std::max(std::max(pick1,pick2), pick3);
                        int node2 = pick1 + pick2 + pick3 - node1 - node3;
                        if (node1==2 && node2 == 2 && node3 == 74)
                            std::cout << "strange" << std::endl;
                        //if (pick1 == pick3) {
                        //    arcVariables[i][j].set(GRB_DoubleAttr_UB, 0.0);
                        //    arcVariables[j][k].set(GRB_DoubleAttr_UB, 0.0);
                        //}
                        threeNodeCut[(node1-1)*totalPickupNodeNumInGraph*totalPickupNodeNumInGraph + (node2-1)*totalPickupNodeNumInGraph + node3-1] = true;
                    }
                }
            }
        }

        int countThreeNodes = 0;
        for (int i = 0; i< threeNodeCut.size(); i++)
            if (threeNodeCut[i])
            {
                int node1, node2, node3;
                node3=i % pickupNodeNum + 1;
                node2=(i-node3+1) / pickupNodeNum % pickupNodeNum + 1;
                node1=(int) (i / (pickupNodeNum*pickupNodeNum)) + 1;
                GRBLinExpr linExpr;
                linExpr = pathVertexVariables[node1] + pathVertexVariables[node2] + pathVertexVariables[node3];
                std::string constrName = "cut" + std::to_string(node1) + "_" + std::to_string(node2) + "_" + std::to_string(node3);
                model.addConstr(linExpr, GRB_LESS_EQUAL, 2, constrName);
                countThreeNodes++;
            }
        std::cout << "added " << countThreeNodes << " three node cuts" << std::endl;

        if (false && nullptr != initSolution) { //turn off initial solution input
            if (freeNodes) {
                VrpSolution tmpSol(initSolution->vrpProblem);
                tmpSol.copySolution(*initSolution);

                while (tmpSol.unassignedNodeIds.size() * 2 < freeNodesNum) {
                    stringRemoval(vrpConfig, tmpSol);
                }
                std::vector<uint8_t> nodeFreeFlags;
                nodeFreeFlags.resize(totalNodeNumInGraph, 0);

                for (int i = 0; i < freeNodesNum / 2; ++i) {
                    int pickupNodeId = tmpSol.unassignedNodeIds[i].first;
                    int deliveryNodeId = tmpSol.unassignedNodeIds[i].second;

                    int newPickupNodeId = originalNewNodeIndexMap[pickupNodeId];
                    int newDeliveryNodeId = originalNewNodeIndexMap[deliveryNodeId];

                    nodeFreeFlags[newPickupNodeId] = 1;
                    nodeFreeFlags[newDeliveryNodeId] = 1;
                }

                for (int i = 0; i < totalNodeNumInGraph; ++i) {
                    for (int j = 0; j < totalNodeNumInGraph; ++j) {
                        arcVariables[i][j].set(GRB_DoubleAttr_Start, 0.0);
                    }
                }
                startServiceTimeVariables[0].set(GRB_DoubleAttr_Start,
                                                 vrpProblem->vrpData.nodeList[0].timeWindow.first);
                vehicleLoadVariables[0].set(GRB_DoubleAttr_Start, 0.0);

                double maxFinalDepotStartServiceTime = 0.0;
                for (const VrpRoute& route : initSolution->routes) {
                    int prevNodeId = 0;
                    int prevNewNodeId = 0;

                    const RouteNode* currentNode = route.startNode;
                    double startServiceTime = vrpProblem->vrpData.nodeList[0].timeWindow.first;
                    double vehicleLoad = 0.0;

                    int firstNodeId = currentNode->nodeId;
                    int firstNewNodeId = originalNewNodeIndexMap[firstNodeId];

                    while (nullptr != currentNode) {
                        int currentNodeId = currentNode->nodeId;
                        int tmpCurrentNewNodeId = originalNewNodeIndexMap[currentNodeId];

                        arcVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, 1.0);
                        startServiceTime += vrpProblem->vrpData.nodeList[prevNodeId].serveTime;
                        startServiceTime += vrpProblem->vrpData.disMatrix[prevNodeId][currentNodeId];
                        startServiceTime =
                            std::max(startServiceTime, vrpProblem->vrpData.nodeList[currentNodeId].timeWindow.first);
                        startServiceTimeVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, startServiceTime);

                        vehicleLoad += vrpProblem->vrpData.nodeList[currentNodeId].demand;
                        vehicleLoadVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, vehicleLoad);

                        routeIdentifierVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, firstNewNodeId);

                        if (!nodeFreeFlags[prevNewNodeId] && !nodeFreeFlags[tmpCurrentNewNodeId]) {
                            arcVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, 1.0);
                        }

                        currentNode = currentNode->nexNode;
                        prevNodeId = currentNodeId;
                        prevNewNodeId = tmpCurrentNewNodeId;
                    }

                    arcVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, 1.0);
                    if (!nodeFreeFlags[prevNewNodeId]) {
                        arcVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_LB, 1.0);
                    }
                    startServiceTime += vrpProblem->vrpData.nodeList[prevNodeId].serveTime;
                    startServiceTime += vrpProblem->vrpData.disMatrix[prevNodeId][0];
                    maxFinalDepotStartServiceTime = std::max(maxFinalDepotStartServiceTime, startServiceTime);
                }

                startServiceTimeVariables[totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start,
                                                                       maxFinalDepotStartServiceTime);
                vehicleLoadVariables[totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, 0.0);
            } else {
                std::vector<uint8_t> routeFreeFlags;
                routeFreeFlags.resize(initSolution->routes.size(), 0);
                for (int routeIndex : freeRouteIndices) {
                    routeFreeFlags[routeIndex] = 1;
                    std::cout << "Free Route " << routeIndex
                              << "; Node num: " << initSolution->routes[routeIndex].nodeNum << std::endl;
                }
                for (int i = 0; i < totalNodeNumInGraph; ++i) {
                    for (int j = 0; j < totalNodeNumInGraph; ++j) {
                        arcVariables[i][j].set(GRB_DoubleAttr_Start, 0.0);
                    }
                }
                startServiceTimeVariables[0].set(GRB_DoubleAttr_Start,
                                                 vrpProblem->vrpData.nodeList[0].timeWindow.first);
                vehicleLoadVariables[0].set(GRB_DoubleAttr_Start, 0.0);

                double maxFinalDepotStartServiceTime = 0.0;
                for (const VrpRoute& route : initSolution->routes) {
                    uint8_t routeFree = routeFreeFlags[route.routeId];
                    int prevNodeId = 0;
                    int prevNewNodeId = 0;

                    const RouteNode* currentNode = route.startNode;
                    double startServiceTime = vrpProblem->vrpData.nodeList[0].timeWindow.first;
                    double vehicleLoad = 0.0;

                    int firstNodeId = currentNode->nodeId;
                    int firstNewNodeId = originalNewNodeIndexMap[firstNodeId];

                    while (nullptr != currentNode) {
                        int currentNodeId = currentNode->nodeId;
                        int tmpCurrentNewNodeId = originalNewNodeIndexMap[currentNodeId];

                        arcVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, 1.0);
                        startServiceTime += vrpProblem->vrpData.nodeList[prevNodeId].serveTime;
                        startServiceTime += vrpProblem->vrpData.disMatrix[prevNodeId][currentNodeId];
                        startServiceTime =
                            std::max(startServiceTime, vrpProblem->vrpData.nodeList[currentNodeId].timeWindow.first);
                        startServiceTimeVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, startServiceTime);

                        vehicleLoad += vrpProblem->vrpData.nodeList[currentNodeId].demand;
                        vehicleLoadVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, vehicleLoad);

                        routeIdentifierVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, firstNewNodeId);

                        if (!routeFree) {
                            arcVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, 1.0);
                            startServiceTimeVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, startServiceTime);
                            startServiceTimeVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_UB, startServiceTime);
                            vehicleLoadVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, vehicleLoad);
                            vehicleLoadVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_UB, vehicleLoad);
                            routeIdentifierVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, firstNewNodeId);
                            routeIdentifierVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_UB, firstNewNodeId);
                        }

                        currentNode = currentNode->nexNode;
                        prevNodeId = currentNodeId;
                        prevNewNodeId = tmpCurrentNewNodeId;
                    }

                    arcVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, 1.0);
                    if (!routeFree) {
                        arcVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_LB, 1.0);
                    }
                    startServiceTime += vrpProblem->vrpData.nodeList[prevNodeId].serveTime;
                    startServiceTime += vrpProblem->vrpData.disMatrix[prevNodeId][0];
                    maxFinalDepotStartServiceTime = std::max(maxFinalDepotStartServiceTime, startServiceTime);
                }

                startServiceTimeVariables[totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start,
                                                                       maxFinalDepotStartServiceTime);
                vehicleLoadVariables[totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, 0.0);
            }
        }
        static int sp_num=0;
        sp_num++;
        std::string spname = std::string("sp") + std::to_string(sp_num) + std::string(".lp");
        std::string spsolname_old = std::string("sp") + std::to_string(sp_num-1) + std::string(".sol");
        std::string spsolname = std::string("sp") + std::to_string(sp_num) + std::string(".sol");
        model.write(spname);
        FILE* file = fopen(spsolname_old.c_str(), "r");
        if (file != NULL) {
            fclose(file);
            model.read(spsolname_old);
        }
        model.optimize();
        model.write(spsolname.c_str());
        // get solution and find route
        // std::cout<<"--------------------------------\n";
        int numSolutions = model.get(GRB_IntAttr_SolCount );
        for (int solIterator= numSolutions-1; solIterator>=0; solIterator --) {
            std::unordered_map<int, int> new_route;
            std::vector<int> sol;
            model.set(GRB_IntParam_SolutionNumber, solIterator);
            for (int i = 0; i < totalNodeNumInGraph; ++i) {
                for (int j = 0; j < totalNodeNumInGraph; ++j) {
                    double res = arcVariables[i][j].get(GRB_DoubleAttr_Xn);
                    if (abs(res) > 0.1) {
                        // std::cout<<"x_"<<i<<"_"<<j<<" : "<<res<<"\n";
                        new_route[newOriginalNodeIndexMap[i]] = newOriginalNodeIndexMap[j];
                    }
                }
            }
            // std::cout<<"--------------------------------\n";
            int pointer = 0;
            std::cout << " --> " << pointer;
            while (new_route.find(pointer) != new_route.end()) {
                if (new_route[pointer] == 0) {
                    break;
                }
                sol.emplace_back(new_route[pointer]);
                std::cout << " --> " << new_route[pointer];
                pointer = new_route[pointer];
            }
            std::cout<<"   "<<totalNodeNumInGraph << ", with objective: " << model.get(GRB_DoubleAttr_PoolObjVal ) << "\n";
            solVec.push_back(sol);
        }
        std::cout<<"--------------------------------\n";
        return model.get(GRB_DoubleAttr_ObjVal ) - objAdj;
    } catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    } catch (...) {
        std::cout << "Error during optimization" << std::endl;
    }

    delete[] vars;
        return 0;
}
