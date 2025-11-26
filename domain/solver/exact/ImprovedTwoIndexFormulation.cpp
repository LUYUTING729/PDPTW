#include "domain/solver/exact/TwoIndexFormulation.h"

#include <gurobi_c++.h>

#include <cassert>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <unordered_map>

#include "domain/solver/memetic/removal/Removal.h"
#include "io/vrp_sol_writer.h"
#include <sstream>

class CallBack : public GRBCallback {
   public:
    int totalNodeNumInGraph;
    int totalPickupNodeNumInGraph;
    std::unordered_map<int, int> newOriginalNodeIndexMap;
    const std::vector<std::vector<GRBVar>>& arcVariables;
    VrpSolution& newSol;
    const std::string& resultFolder;
    CallBack(int totalNodeNumInGraph_, int totalPickupNodeNumInGraph_,
             std::unordered_map<int, int> newOriginalNodeIndexMap_,
             const std::vector<std::vector<GRBVar>>& arcVariables_, VrpSolution& newSol_,
             const std::string& resultFolder_)
        : totalNodeNumInGraph(totalNodeNumInGraph_),
          totalPickupNodeNumInGraph(totalPickupNodeNumInGraph_),
          newOriginalNodeIndexMap(newOriginalNodeIndexMap_),
          arcVariables(arcVariables_),
          newSol(newSol_),
          resultFolder(resultFolder_) {}

   protected:
    void callback() {
        try {
            if (where == GRB_CB_MIPSOL) {
                std::cout<<"In callback\n";
                std::vector<int> routeStartNodeIds;
                std::vector<int> nodeNextNodeIds;
                nodeNextNodeIds.resize(totalNodeNumInGraph, -1);
                
                // 添加边界检查
                if (arcVariables.empty() || arcVariables[0].empty()) {
                    std::cerr << "Error: arcVariables is empty" << std::endl;
                    return;
                }
                
                double* x = getSolution(&arcVariables[0][0], totalNodeNumInGraph);
                if (x == nullptr) {
                    std::cerr << "Error: getSolution returned nullptr" << std::endl;
                    return;
                }

                for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
                    if (i < totalNodeNumInGraph) {  // 边界检查
                        double arcVarValue = x[i];
                        if (arcVarValue > 0.1) {
                            routeStartNodeIds.push_back(i);
                        }
                    }
                }
                delete[] x;
                x = nullptr;

                for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
                    if (i < (int)arcVariables.size()) {  // 边界检查
                        x = getSolution(&arcVariables[i][0], totalNodeNumInGraph);
                        if (x == nullptr) {
                            std::cerr << "Error: getSolution returned nullptr for i=" << i << std::endl;
                            continue;
                        }
                        for (int j = 1; j < totalNodeNumInGraph; ++j) {
                            double arcVarValue = x[j];
                            if (arcVarValue > 0.1) {
                                nodeNextNodeIds[i] = j;
                            }
                        }
                        delete[] x;
                        x = nullptr;
                    }
                }
                newSol.totalValueValid = false;
                int routeIndex = 0;
                for (int startNodeId : routeStartNodeIds) {
                    if (routeIndex >= (int)newSol.routes.size()) {
                        std::cerr << "Error: routeIndex " << routeIndex << " exceeds routes size " << newSol.routes.size() << std::endl;
                        break;
                    }
                    
                    newSol.routes[routeIndex].clearData();

                    // 添加map查找的安全检查
                    auto it = newOriginalNodeIndexMap.find(startNodeId);
                    if (it == newOriginalNodeIndexMap.end()) {
                        std::cerr << "Error: startNodeId " << startNodeId << " not found in newOriginalNodeIndexMap" << std::endl;
                        continue;
                    }
                    int originalStartNodeId = it->second;
                    
                    if (originalStartNodeId >= (int)newSol.routeNodes.size()) {
                        std::cerr << "Error: originalStartNodeId " << originalStartNodeId << " exceeds routeNodes size " << newSol.routeNodes.size() << std::endl;
                        continue;
                    }
                    
                    newSol.routes[routeIndex].startNode = &newSol.routeNodes[originalStartNodeId];
                    newSol.routes[routeIndex].startNode->preNode = nullptr;

                    RouteNode* preNode = newSol.routes[routeIndex].startNode;
                    int routeNodeNum = 1;

                    if (startNodeId < (int)nodeNextNodeIds.size()) {
                        int currentNodeId = nodeNextNodeIds[startNodeId];
                        while (totalNodeNumInGraph - 1 != currentNodeId) {
                            auto currentIt = newOriginalNodeIndexMap.find(currentNodeId);
                            if (currentIt == newOriginalNodeIndexMap.end()) {
                                std::cerr << "Error: currentNodeId " << currentNodeId << " not found in newOriginalNodeIndexMap" << std::endl;
                                break;
                            }
                            int originalCurrentNodeId = currentIt->second;
                            
                            if (originalCurrentNodeId >= (int)newSol.routeNodes.size()) {
                                std::cerr << "Error: originalCurrentNodeId " << originalCurrentNodeId << " exceeds routeNodes size " << newSol.routeNodes.size() << std::endl;
                                break;
                            }
                            
                            RouteNode* currentNode = &newSol.routeNodes[originalCurrentNodeId];

                            preNode->nexNode = currentNode;
                            currentNode->preNode = preNode;

                            preNode = currentNode;

                            if (currentNodeId >= (int)nodeNextNodeIds.size()) {
                                std::cerr << "Error: currentNodeId " << currentNodeId << " exceeds nodeNextNodeIds size" << std::endl;
                                break;
                            }
                            currentNodeId = nodeNextNodeIds[currentNodeId];
                            routeNodeNum += 1;
                        }
                    }

                    preNode->nexNode = nullptr;
                    newSol.routes[routeIndex].endNode = preNode;
                    newSol.routes[routeIndex].nodeNum = routeNodeNum;
                    newSol.updateRouteData(routeIndex, newSol.routes[routeIndex].startNode,
                                           newSol.routes[routeIndex].endNode);

                    routeIndex += 1;
                }
                std::cout<<"Writing sols\n";
                std::filesystem::path resultFolderPath = resultFolder;
                std::stringstream stream;
                stream << std::fixed << std::setprecision(2) << newSol.getTotalDistance() << "_"
                       << newSol.getTotalTwViolation() << "_" << newSol.getTotalCapacityViolation();
                std::string solQualityStr = stream.str();
                std::string solFileName = newSol.vrpProblem->vrpData.name + "." + std::to_string(newSol.routes.size()) +
                                          "_" + solQualityStr + ".txt";

                resultFolderPath /= solFileName;

                std::string solFilePath = resultFolderPath.string();
                WriteSol(newSol, solFilePath);
                std::cout<<"Finished Writing sols\n";
                std::cout<<"End callback\n";
            }
        } catch (GRBException e) {
            std::cout << "Error number: " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;
        } catch (...) {
            std::cout << "Error during callback" << std::endl;
        }
    }
};


void improvedTwoIndexFormulationSolve(const VrpConfig& vrpConfig, const VrpProblem* vrpProblem, const VrpSolution* initSolution,
                                      bool freeNodes, int freeNodesNum, const std::vector<int>& freeRouteIndices,
                                      double timeLimit, bool pricingSubProblem,
                                      const PricingSubProblemData& pricingSubProblemData,
                                      PricingSubProblemSolution& pricingSubProblemSolution, VrpSolution& vrpSolution,
                                      const std::string& resultFolder) {
    if (nullptr != initSolution) {
        assert(initSolution->unassignedNodeIds.empty());
        assert(initSolution->totalCapacityViolation <= 0.0);
        assert(initSolution->totalTwViolation <= 0.0);
        vrpSolution.copySolution(*initSolution);
    }

    // 建立原始节点索引和新节点索引之间的映射关系
    std::unordered_map<int, int> originalNewNodeIndexMap;  // 原始索引到新索引的映射
    std::unordered_map<int, int> newOriginalNodeIndexMap;  // 新索引到原始索引的映射
    int pickupNodeCounter = 0;  // 取货节点计数器
    int pickupNodeNum = (int)vrpProblem->vrpData.pickNodeIds.size();  // 取货节点总数

    // 遍历所有节点,建立映射关系
    for (int i = 1; i < vrpProblem->vrpData.nodeNum; ++i) {
        // 添加边界检查以防止数组越界
        if (i >= (int)vrpProblem->vrpData.nodeList.size()) {
            std::cerr << "Error: Node index " << i << " exceeds nodeList size "
                      << vrpProblem->vrpData.nodeList.size() << std::endl;
            break;
        }

        if (Utils::VrpNodeType::kPickup == vrpProblem->vrpData.nodeList[i].nodeType) {
            pickupNodeCounter += 1;
            originalNewNodeIndexMap[i] = pickupNodeCounter;
            newOriginalNodeIndexMap[pickupNodeCounter] = i;

            int tmpDeliveryNodeIndex = vrpProblem->vrpData.nodeList[i].corresNodeId;

            // 添加对 corresNodeId 的边界检查
            if (tmpDeliveryNodeIndex < 0 || tmpDeliveryNodeIndex >= (int)vrpProblem->vrpData.nodeList.size()) {
                std::cerr << "Error: Corresponding delivery node index " << tmpDeliveryNodeIndex
                          << " is out of bounds for pickup node " << i << std::endl;
                continue;
            }

            originalNewNodeIndexMap[tmpDeliveryNodeIndex] = pickupNodeCounter + pickupNodeNum;
            newOriginalNodeIndexMap[pickupNodeCounter + pickupNodeNum] = tmpDeliveryNodeIndex;

            pricingSubProblem = false;

            if (pricingSubProblem) {
                if (pricingSubProblemData.dualValueForPickupNodes.find(i) ==
                    pricingSubProblemData.dualValueForPickupNodes.end()) {
                    std::cerr << "Dual value for pickup node " << i << " not found" << std::endl;
                    exit(0);
                }
            }
        }
    }

    // 重置定价子问题标志
    pricingSubProblem = false;
    if (pricingSubProblem) {
        std::string filename("E:\\Xiaodong\\pdVRPTW\\pdptw1000\\pdp_1000_223\\python\\dual23.txt");
        FILE* file = fopen(filename.c_str(), "r");
        std::vector<double> dual_info(vrpProblem->vrpData.nodeNum, 0.0);
        int loc;
        double dual;
        while (fscanf(file, "%d%lf", &loc, &dual) != EOF)
            dual_info[std::max(loc, 0)] = .45 * dual;

        if (file == nullptr) {
            printf("something wrong!\n");
            return;
        }
        fclose(file);

        for (int i = 0; i < vrpProblem->vrpData.nodeNum; ++i) {
            ((PricingSubProblemData*)(&pricingSubProblemData))
                ->dualValueForPickupNodes.insert(std::make_pair(i, dual_info[i]));
        }
    }

    // 计算图中节点总数
    int totalPickupNodeNumInGraph = (int)vrpProblem->vrpData.pickNodeIds.size();  // 取货点数量
    int totalNodeNumInGraph = vrpProblem->vrpData.nodeNum + 1;                    // 总节点数(包括depot)

    // 初始化Gurobi变量
    std::vector<std::vector<GRBVar>> arcVariables;  // 弧变量 x[i][j]表示是否使用从i到j的弧
    arcVariables.resize(totalNodeNumInGraph);
    std::vector<GRBVar> pathVertexVariables;  // 路径顶点变量(定价子问题用)
    pathVertexVariables.resize(totalNodeNumInGraph);

    std::vector<GRBVar> routeIdentifierVariables;  // 路径标识符变量 v_j
    routeIdentifierVariables.resize(totalNodeNumInGraph);

    // 弧时间变量 y_ij：沿弧(i,j)到达 j 的时间
    std::vector<std::vector<GRBVar>> arcTimeVariables;
    arcTimeVariables.resize(totalNodeNumInGraph);
    // 弧载重变量 z_ij：沿弧(i,j)到达 j 的载重
    std::vector<std::vector<GRBVar>> arcLoadVariables;
    arcLoadVariables.resize(totalNodeNumInGraph);

    GRBVar* vars = 0;
    CallBack* cb_ptr = nullptr;  // 在try块外声明，使其在catch块中也可访问

    try {
        std::cout << "[DEBUG] 0: twoIndexFormulationSolve main enter\n" << std::flush;

        // 初始化Gurobi环境
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
        env.set("Threads", "1"); 
        env.set("TimeLimit", std::to_string(timeLimit));
        env.start();
        std::cout << "[DEBUG] 1: GRBEnv started successfully\n" << std::flush;

        // 创建Gurobi模型
        GRBModel model = GRBModel(env);
        std::cout << "[DEBUG] 2: GRBModel created\n" << std::flush;

        // 预计算全局时间上界(用于 depot 的 y_ij 上界)
        double globalTimeHorizon = 0.0;
        for (const auto& node : vrpProblem->vrpData.nodeList) {
            globalTimeHorizon = std::max(globalTimeHorizon, node.timeWindow.second);
        }

        // 为 x_ij, y_ij, z_ij 三个二维数组分配空间
        for (int i = 0; i < totalNodeNumInGraph; ++i) {
            arcVariables[i].resize(totalNodeNumInGraph);
            arcTimeVariables[i].resize(totalNodeNumInGraph);
            arcLoadVariables[i].resize(totalNodeNumInGraph);
        }

        // ============================
        // 添加决策变量：x_ij, y_ij, z_ij
        // ============================
        std::cout << "[DEBUG] 2-vars-start: Begin adding variables, totalNodeNumInGraph=" << totalNodeNumInGraph << "\n" << std::flush;
        
        for (int i = 0; i < totalNodeNumInGraph; ++i) {
            int tmpFirstOriginalNodeIndex = 0;
            if (1 <= i && i < totalNodeNumInGraph - 1) {
                tmpFirstOriginalNodeIndex = newOriginalNodeIndexMap[i];
            }

            // 内层循环: 为每对节点(i,j)添加弧相关变量
            for (int j = 0; j < totalNodeNumInGraph; ++j) {
                bool tmpInfeasibleArc = false;

                // 1. 禁止自环
                if (i == j) {
                    tmpInfeasibleArc = true;
                }
                // 2. 禁止到达起始 depot(0) 的弧
                if (!tmpInfeasibleArc && 0 == j) {
                    tmpInfeasibleArc = true;
                }
                // 3. 禁止从终点 depot 出发的弧
                if (!tmpInfeasibleArc && totalNodeNumInGraph - 1 == i) {
                    tmpInfeasibleArc = true;
                }
                // 4. 禁止取货点直接到终点 depot 的弧
                if (!tmpInfeasibleArc && 1 <= i && i <= totalPickupNodeNumInGraph &&
                    totalNodeNumInGraph - 1 == j) {
                    tmpInfeasibleArc = true;
                }
                // 5. 配送点相关限制
                if (!tmpInfeasibleArc && i < totalNodeNumInGraph - 1 && i > totalPickupNodeNumInGraph &&
                    i - j == totalPickupNodeNumInGraph) {
                    tmpInfeasibleArc = true;
                }

                // 获取节点j的原始索引
                int tmpSecondOriginalNodeIndex = 0;
                if (1 <= j && j < totalNodeNumInGraph - 1) {
                    tmpSecondOriginalNodeIndex = newOriginalNodeIndexMap[j];
                }

                // 检查是否在候选下一节点集合中
                if (!tmpInfeasibleArc) {
                    if (vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].candiNextNodes.find(
                            tmpSecondOriginalNodeIndex) ==
                        vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].candiNextNodes.end()) {
                        tmpInfeasibleArc = true;
                    }
                }

                // 设置 x_ij 变量上界
                double xUpperBound = 1.0;
                if (tmpInfeasibleArc) {
                    xUpperBound = 0.0;
                }

                // 计算目标函数系数（保持原逻辑）
                double objCoeff =
                    vrpProblem->vrpData.disMatrix[tmpFirstOriginalNodeIndex][tmpSecondOriginalNodeIndex];
                if (pricingSubProblem) {
                    if (Utils::VrpNodeType::kPickup ==
                        vrpProblem->vrpData.nodeList[tmpFirstOriginalNodeIndex].nodeType) {
                        if (pricingSubProblemData.dualValueForPickupNodes.find(tmpFirstOriginalNodeIndex) !=
                            pricingSubProblemData.dualValueForPickupNodes.end()) {
                            objCoeff -=
                                pricingSubProblemData.dualValueForPickupNodes.find(tmpFirstOriginalNodeIndex)->second;
                        } else {
                            std::cerr << "Dual value for pickup node " << i << " not found" << std::endl;
                            exit(0);
                        }
                    }
                } else if (0 == i) {  // 从起始 depot 出发的弧,添加较大惩罚(相当于车辆固定成本)
                    objCoeff += 10000.0;
                }

                // ------------ x_ij 变量 ------------
                {
                    std::string varName = "x_" + std::to_string(i) + "_" + std::to_string(j);
                    GRBVar xVar = model.addVar(0.0, xUpperBound, objCoeff, GRB_BINARY, varName);
                    arcVariables[i][j] = xVar;
                }

                // ------------ y_ij 变量（弧时间）------------
                {
                    // 对应节点 j 的时间窗 [e_j, l_j]
                    double e_j = 0.0;
                    double l_j = 0.0;
                    if (1 <= j && j < totalNodeNumInGraph - 1) {
                        const VrpNode& nodeJ = vrpProblem->vrpData.nodeList[tmpSecondOriginalNodeIndex];
                        e_j = nodeJ.timeWindow.first;
                        l_j = nodeJ.timeWindow.second;
                    } else {
                        // depot/终点：使用全局时间上界
                        e_j = 0.0;
                        l_j = globalTimeHorizon;
                    }

                    double yLower = 0.0;
                    double yUpper = tmpInfeasibleArc ? 0.0 : l_j;

                    std::string yName = "y_" + std::to_string(i) + "_" + std::to_string(j);
                    GRBVar yVar = model.addVar(yLower, yUpper, 0.0, GRB_CONTINUOUS, yName);
                    arcTimeVariables[i][j] = yVar;

                    // 链接约束：y_ij <= l_j * x_ij  (x_ij=0 => y_ij=0)
                    if (!tmpInfeasibleArc && l_j > 0.0) {
                        GRBLinExpr linkExpr;
                        linkExpr += yVar;
                        linkExpr -= l_j * arcVariables[i][j];
                        std::string cname = "link_y_x_" + std::to_string(i) + "_" + std::to_string(j);
                        model.addConstr(linkExpr, GRB_LESS_EQUAL, 0.0, cname);
                    }
                }

                // ------------ z_ij 变量（弧载重）------------
                {
                    double capacity = vrpProblem->vrpData.vehicleCapacity;
                    double zLower = 0.0;
                    double zUpper = tmpInfeasibleArc ? 0.0 : capacity;

                    std::string zName = "z_" + std::to_string(i) + "_" + std::to_string(j);
                    GRBVar zVar = model.addVar(zLower, zUpper, 0.0, GRB_CONTINUOUS, zName);
                    arcLoadVariables[i][j] = zVar;

                    // 链接约束：z_ij <= Cap * x_ij  (x_ij=0 => z_ij=0)
                    if (!tmpInfeasibleArc && capacity > 0.0) {
                        GRBLinExpr linkExpr;
                        linkExpr += zVar;
                        linkExpr -= capacity * arcVariables[i][j];
                        std::string cname = "link_z_x_" + std::to_string(i) + "_" + std::to_string(j);
                        model.addConstr(linkExpr, GRB_LESS_EQUAL, 0.0, cname);
                    }
                }
            }

            // 路径顶点变量(仅在定价子问题中) —— 保持原逻辑
            if (pricingSubProblem) {
                double objCoeffz = 0.0, bd = 1.0;
                std::string varNamez = "z_" + std::to_string(i);
                if (i > 0 && i <= totalPickupNodeNumInGraph) {
                    objCoeffz =
                        -pricingSubProblemData.dualValueForPickupNodes.find(tmpFirstOriginalNodeIndex)->second;
                }
                if (objCoeffz >= -65) bd = 0.0;
                GRBVar newVarz = model.addVar(0.0, bd, objCoeffz, GRB_BINARY, varNamez);
                pathVertexVariables[i] = newVarz;
            }
        }

        // 路径标识符变量 v_j
        for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
            double varLower = 1.0;
            double varUpper = totalPickupNodeNumInGraph;
            std::string varName = "v_" + std::to_string(i);

            GRBVar newVar = model.addVar(varLower, varUpper, 0.0, GRB_CONTINUOUS, varName);
            routeIdentifierVariables[i] = newVar;
        }
        
        std::cout << "[DEBUG] 2-vars-end: All variables added\n" << std::flush;

        // =========================
        // 流量 / 出入度约束
        // =========================
        std::cout << "[DEBUG] 2-constr-start: Begin adding constraints\n" << std::flush;
        
        if (pricingSubProblem) {
            for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
                GRBLinExpr linExpr;
                for (int j = 0; j < totalNodeNumInGraph - 1; ++j) {
                    linExpr += arcVariables[j][i];  // Σ x_{ji}
                }
                for (int j = 1; j < totalNodeNumInGraph; ++j) {
                    linExpr -= arcVariables[i][j];  // - Σ x_{ij}
                }
                std::string constrName = "flow_balance_" + std::to_string(i);
                model.addConstr(linExpr, GRB_EQUAL, 0.0, constrName);
            }
        } else {
            // 入度：每个非仓库节点恰好被访问一次
            for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
                GRBLinExpr linExpr;
                for (int j = 0; j < totalNodeNumInGraph - 1; ++j) {
                    linExpr += arcVariables[j][i];
                }
                std::string constrName = "in_degree_" + std::to_string(i);
                model.addConstr(linExpr, GRB_EQUAL, 1.0, constrName);
            }
            // 出度：每个非仓库节点恰好离开一次
            for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
                GRBLinExpr linExpr;
                for (int j = 1; j < totalNodeNumInGraph; ++j) {
                    linExpr += arcVariables[i][j];
                }
                std::string constrName = "out_degree_" + std::to_string(i);
                model.addConstr(linExpr, GRB_EQUAL, 1.0, constrName);
            }
        }

        // =========================
        // 基于 y_ij 的时间相关约束
        // =========================

        // (3) 时间传播：Σ_k y_{jk} >= Σ_i ( y_{ij} + t_ij x_ij ), ∀ j∈P∪D
        for (int j = 1; j < totalNodeNumInGraph - 1; ++j) {
            int oriJ = newOriginalNodeIndexMap[j];
            const VrpNode& nodeJ = vrpProblem->vrpData.nodeList[oriJ];

            if (!(nodeJ.nodeType == Utils::VrpNodeType::kPickup ||
                  nodeJ.nodeType == Utils::VrpNodeType::kDelivery)) {
                continue;
            }

            GRBLinExpr lhs;
            for (int k = 1; k < totalNodeNumInGraph; ++k) {
                lhs += arcTimeVariables[j][k];
            }

            GRBLinExpr rhs;
            for (int i = 0; i < totalNodeNumInGraph - 1; ++i) {
                int oriI = 0;
                if (1 <= i && i < totalNodeNumInGraph - 1) {
                    oriI = newOriginalNodeIndexMap[i];
                }
                const VrpNode& nodeI = vrpProblem->vrpData.nodeList[oriI];

                double travelTime =
                    vrpProblem->vrpData.disMatrix[oriI][oriJ] + nodeI.serveTime;

                rhs += arcTimeVariables[i][j];
                rhs += travelTime * arcVariables[i][j];
            }

            std::string cname = "time_propagation_arc_" + std::to_string(j);
            model.addConstr(lhs, GRB_GREATER_EQUAL, rhs, cname);
        }

        // (5) 时间窗：e_j Σ_i x_ij <= Σ_k y_jk <= l_j Σ_i x_ij, ∀ j∈P∪D
        for (int j = 1; j < totalNodeNumInGraph - 1; ++j) {
            int oriJ = newOriginalNodeIndexMap[j];
            const VrpNode& nodeJ = vrpProblem->vrpData.nodeList[oriJ];

            if (!(nodeJ.nodeType == Utils::VrpNodeType::kPickup ||
                  nodeJ.nodeType == Utils::VrpNodeType::kDelivery)) {
                continue;
            }

            double e_j = nodeJ.timeWindow.first;
            double l_j = nodeJ.timeWindow.second;

            GRBLinExpr inFlow;
            for (int i = 0; i < totalNodeNumInGraph - 1; ++i) {
                inFlow += arcVariables[i][j];
            }

            GRBLinExpr sumYout;
            for (int k = 1; k < totalNodeNumInGraph; ++k) {
                sumYout += arcTimeVariables[j][k];
            }

            std::string cnameL = "time_window_arc_L_" + std::to_string(j);
            model.addConstr(e_j * inFlow, GRB_LESS_EQUAL, sumYout, cnameL);

            std::string cnameU = "time_window_arc_U_" + std::to_string(j);
            model.addConstr(sumYout, GRB_LESS_EQUAL, l_j * inFlow, cnameU);
        }

        // (4) 取送配对时间先后：Σ_k y_{j,k} <= Σ_k y_{n+j,k}, ∀ j∈P
        for (int j = 1; j <= totalPickupNodeNumInGraph; ++j) {
            int delIdx = j + totalPickupNodeNumInGraph;

            GRBLinExpr lhs;
            GRBLinExpr rhs;
            for (int k = 1; k < totalNodeNumInGraph; ++k) {
                lhs += arcTimeVariables[j][k];
                rhs += arcTimeVariables[delIdx][k];
            }

            std::string cname = "time_pd_order_arc_" + std::to_string(j);
            model.addConstr(lhs, GRB_LESS_EQUAL, rhs, cname);
        }

        // =========================
        // 基于 z_ij 的载重相关约束
        // =========================
        double capacity = vrpProblem->vrpData.vehicleCapacity;

        // (6)(7) 取货节点载重递推与容量：
        // Σ_k z_jk >= Σ_i (z_ij + q_j x_ij)
        // q_j Σ_i x_ij <= Σ_k z_jk <= Cap Σ_i x_ij
        for (int j = 1; j <= totalPickupNodeNumInGraph; ++j) {
            int oriJ = newOriginalNodeIndexMap[j];
            const VrpNode& nodeJ = vrpProblem->vrpData.nodeList[oriJ];
            double q_j = nodeJ.demand;  // pickup 的装载量(>0)

            GRBLinExpr sumZout;
            for (int k = 1; k < totalNodeNumInGraph; ++k) {
                sumZout += arcLoadVariables[j][k];
            }

            GRBLinExpr rhs;
            for (int i = 0; i < totalNodeNumInGraph - 1; ++i) {
                rhs += arcLoadVariables[i][j];
                rhs += q_j * arcVariables[i][j];
            }

            std::string cname6 = "load_pickup_flow_" + std::to_string(j);
            model.addConstr(sumZout, GRB_GREATER_EQUAL, rhs, cname6);

            GRBLinExpr inFlow;
            for (int i = 0; i < totalNodeNumInGraph - 1; ++i) {
                inFlow += arcVariables[i][j];
            }

            std::string cname7L = "load_pickup_capL_" + std::to_string(j);
            model.addConstr(q_j * inFlow, GRB_LESS_EQUAL, sumZout, cname7L);

            std::string cname7U = "load_pickup_capU_" + std::to_string(j);
            model.addConstr(sumZout, GRB_LESS_EQUAL, capacity * inFlow, cname7U);
        }

        // 送货节点减载约束：
        // 对 delivery j（对应 pickup j-n）：
        // Σ_k z_jk <= Σ_i (z_ij - q_pick x_ij)
        for (int j = totalPickupNodeNumInGraph + 1;
             j <= 2 * totalPickupNodeNumInGraph; ++j) {

            int pickupIdx = j - totalPickupNodeNumInGraph;
            int oriPickup = newOriginalNodeIndexMap[pickupIdx];
            const VrpNode& pickupNode = vrpProblem->vrpData.nodeList[oriPickup];
            double q_pick = pickupNode.demand;

            GRBLinExpr sumZout;
            for (int k = 1; k < totalNodeNumInGraph; ++k) {
                sumZout += arcLoadVariables[j][k];
            }

            GRBLinExpr rhs;
            for (int i = 0; i < totalNodeNumInGraph - 1; ++i) {
                rhs += arcLoadVariables[i][j];
                rhs -= q_pick * arcVariables[i][j];
            }

            std::string cname = "load_delivery_dec_" + std::to_string(j);
            model.addConstr(sumZout, GRB_LESS_EQUAL, rhs, cname);

            // 容量上界：Σ_k z_jk <= Cap Σ_i x_ij
            GRBLinExpr inFlow;
            for (int i = 0; i < totalNodeNumInGraph - 1; ++i) {
                inFlow += arcVariables[i][j];
            }
            std::string cnameCap = "load_delivery_capU_" + std::to_string(j);
            model.addConstr(sumZout, GRB_LESS_EQUAL, capacity * inFlow, cnameCap);
        }

        // =========================
        // 路径标识符 v_j 相关约束
        // =========================

        // 取送配对同路径：v_{n+i} = v_i
        for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
            GRBVar pickupRouteIdentifierVar = routeIdentifierVariables[i];
            GRBVar deliveryRouteIdentifierVar = routeIdentifierVariables[i + totalPickupNodeNumInGraph];

            GRBLinExpr linExpr;
            linExpr += pickupRouteIdentifierVar;
            linExpr -= deliveryRouteIdentifierVar;

            std::string constrName = "pd_route_id_" + std::to_string(i);
            model.addConstr(linExpr, GRB_EQUAL, 0.0, constrName);
        }

        // 起点路径标识符下界：v_i >= i * x_{0,i}
        for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
            GRBVar routeIdentifierVar = routeIdentifierVariables[i];

            GRBLinExpr linExpr;
            linExpr += routeIdentifierVar;
            linExpr -= i * arcVariables[0][i];

            std::string constrName = "route_id_first_l_" + std::to_string(i);
            model.addConstr(linExpr, GRB_GREATER_EQUAL, 0.0, constrName);
        }

        // 起点路径标识符上界：v_i <= n - (n-i) * x_{0,i}
        for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
            GRBVar routeIdentifierVar = routeIdentifierVariables[i];

            GRBLinExpr linExpr;
            linExpr += routeIdentifierVar;
            linExpr += (totalPickupNodeNumInGraph - i) * arcVariables[0][i];

            std::string constrName = "route_id_first_u_" + std::to_string(i);
            model.addConstr(linExpr, GRB_LESS_EQUAL, totalPickupNodeNumInGraph, constrName);
        }

        // 内部连续性：v_j - v_i 在 [-M, M] ∩ {0} 上受 x_ij 控制
        for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
            GRBVar firstRouteIdentifierVar = routeIdentifierVariables[i];

            for (int j = 1; j < totalNodeNumInGraph - 1; ++j) {
                if (j == i) continue;

                GRBVar secondRouteIdentifierVar = routeIdentifierVariables[j];

                GRBLinExpr linExpr;
                linExpr += firstRouteIdentifierVar;
                linExpr -= secondRouteIdentifierVar;
                linExpr += totalPickupNodeNumInGraph * arcVariables[i][j];

                std::string constrName = "route_id_l_" + std::to_string(i) + "_" + std::to_string(j);
                model.addConstr(linExpr, GRB_LESS_EQUAL, totalPickupNodeNumInGraph, constrName);
            }
        }

        for (int i = 1; i < totalNodeNumInGraph - 1; ++i) {
            GRBVar firstRouteIdentifierVar = routeIdentifierVariables[i];

            for (int j = 1; j < totalNodeNumInGraph - 1; ++j) {
                if (j == i) continue;

                GRBVar secondRouteIdentifierVar = routeIdentifierVariables[j];

                GRBLinExpr linExpr;
                linExpr += firstRouteIdentifierVar;
                linExpr -= secondRouteIdentifierVar;
                linExpr -= totalPickupNodeNumInGraph * arcVariables[i][j];

                std::string constrName = "route_id_u_" + std::to_string(i) + "_" + std::to_string(j);
                model.addConstr(linExpr, GRB_GREATER_EQUAL, -totalPickupNodeNumInGraph, constrName);
            }
        }

        // =========================
        // 定价子问题附加约束
        // =========================
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
            // constrName = "single_route_to_depot";
            // model.addConstr(linExpr2, GRB_EQUAL, 1.0, constrName);

            GRBLinExpr linExpr3;
            for (int i = 1; i <= totalPickupNodeNumInGraph; ++i) {
                for (int j = 1; j < totalNodeNumInGraph; ++j) {
                    linExpr3 += arcVariables[i][j];
                }
            }
            constrName = "route_length_limit";
            model.addConstr(linExpr3, GRB_LESS_EQUAL, pricingSubProblemData.pickupNodeNumLimitInRoute, constrName);
        }
        
        std::cout << "[DEBUG] 2-constr-end: All constraints added\n" << std::flush;

        // =========================
        // 初始解 warm-start（只对 x / v 做初始化）
        // =========================
        std::cout << "[DEBUG] 2-warmstart-start: Begin warm-start setup\n" << std::flush;
        
        if (nullptr != initSolution) {
            if (freeNodes) {
                // 随机释放节点
                VrpSolution tmpSol(initSolution->vrpProblem);
                tmpSol.copySolution(*initSolution);

                while (tmpSol.unassignedNodeIds.size() * 2 < freeNodesNum) {
                    stringRemoval(vrpConfig, tmpSol);
                }

                std::vector<uint8_t> nodeFreeFlags(totalNodeNumInGraph, 0);

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

                for (const VrpRoute& route : initSolution->routes) {
                    int prevNodeId = 0;
                    int prevNewNodeId = 0;

                    const RouteNode* currentNode = route.startNode;
                    double arrivalTime = vrpProblem->vrpData.nodeList[0].timeWindow.first;  // 从depot出发
                    double vehicleLoad = 0.0;

                    int firstNodeId = currentNode->nodeId;
                    int firstNewNodeId = originalNewNodeIndexMap[firstNodeId];

                    while (nullptr != currentNode) {
                        int currentNodeId = currentNode->nodeId;
                        int tmpCurrentNewNodeId = originalNewNodeIndexMap[currentNodeId];

                        // ===== x_ij warm-start =====
                        arcVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, 1.0);

                        // ===== y_ij warm-start：计算到达当前节点的时间 =====
                        arrivalTime += vrpProblem->vrpData.nodeList[prevNodeId].serveTime;
                        arrivalTime += vrpProblem->vrpData.disMatrix[prevNodeId][currentNodeId];
                        arrivalTime =
                            std::max(arrivalTime, vrpProblem->vrpData.nodeList[currentNodeId].timeWindow.first);
                        
                        // 为y_ij设置初值
                        arcTimeVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, arrivalTime);

                        // ===== z_ij warm-start：计算到达当前节点的累计载重 =====
                        vehicleLoad += vrpProblem->vrpData.nodeList[currentNodeId].demand;
                        
                        // 为z_ij设置初值
                        arcLoadVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, vehicleLoad);

                        // v_j warm-start
                        routeIdentifierVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, firstNewNodeId);

                        if (!nodeFreeFlags[prevNewNodeId] && !nodeFreeFlags[tmpCurrentNewNodeId]) {
                            arcVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, 1.0);
                        }

                        currentNode = currentNode->nexNode;
                        prevNodeId = currentNodeId;
                        prevNewNodeId = tmpCurrentNewNodeId;
                    }

                    // ===== 终点 depot 的 warm-start =====
                    arcVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, 1.0);
                    
                    // y_ij：到达终点depot的时间
                    arrivalTime += vrpProblem->vrpData.nodeList[prevNodeId].serveTime;
                    arrivalTime += vrpProblem->vrpData.disMatrix[prevNodeId][0];
                    arcTimeVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, arrivalTime);
                    
                    // z_ij：到达终点时载重应为0（全部卸货）
                    arcLoadVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, 0.0);
                    
                    if (!nodeFreeFlags[prevNewNodeId]) {
                        arcVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_LB, 1.0);
                    }
                }
            } else {
                // 只释放部分路径
                std::vector<uint8_t> routeFreeFlags(initSolution->routes.size(), 0);

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

                for (const VrpRoute& route : initSolution->routes) {
                    uint8_t routeFree = routeFreeFlags[route.routeId];
                    int prevNodeId = 0;
                    int prevNewNodeId = 0;

                    const RouteNode* currentNode = route.startNode;
                    double arrivalTime = vrpProblem->vrpData.nodeList[0].timeWindow.first;  // 从depot出发
                    double vehicleLoad = 0.0;

                    int firstNodeId = currentNode->nodeId;
                    int firstNewNodeId = originalNewNodeIndexMap[firstNodeId];

                    while (nullptr != currentNode) {
                        int currentNodeId = currentNode->nodeId;
                        int tmpCurrentNewNodeId = originalNewNodeIndexMap[currentNodeId];

                        // ===== x_ij warm-start =====
                        arcVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, 1.0);

                        // ===== y_ij warm-start：计算到达当前节点的时间 =====
                        arrivalTime += vrpProblem->vrpData.nodeList[prevNodeId].serveTime;
                        arrivalTime += vrpProblem->vrpData.disMatrix[prevNodeId][currentNodeId];
                        arrivalTime =
                            std::max(arrivalTime, vrpProblem->vrpData.nodeList[currentNodeId].timeWindow.first);
                        
                        // 为y_ij设置初值
                        arcTimeVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, arrivalTime);

                        // ===== z_ij warm-start：计算到达当前节点的累计载重 =====
                        vehicleLoad += vrpProblem->vrpData.nodeList[currentNodeId].demand;
                        
                        // 为z_ij设置初值
                        arcLoadVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, vehicleLoad);

                        // v_j warm-start
                        routeIdentifierVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_Start, firstNewNodeId);

                        if (!routeFree) {
                            arcVariables[prevNewNodeId][tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, 1.0);
                            routeIdentifierVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_LB, firstNewNodeId);
                            routeIdentifierVariables[tmpCurrentNewNodeId].set(GRB_DoubleAttr_UB, firstNewNodeId);
                        }

                        currentNode = currentNode->nexNode;
                        prevNodeId = currentNodeId;
                        prevNewNodeId = tmpCurrentNewNodeId;
                    }

                    // ===== 终点 depot 的 warm-start =====
                    arcVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, 1.0);
                    
                    // y_ij：到达终点depot的时间
                    arrivalTime += vrpProblem->vrpData.nodeList[prevNodeId].serveTime;
                    arrivalTime += vrpProblem->vrpData.disMatrix[prevNodeId][0];
                    arcTimeVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, arrivalTime);
                    
                    // z_ij：到达终点时载重应为0（全部卸货）
                    arcLoadVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_Start, 0.0);
                    
                    if (!routeFree) {
                        arcVariables[prevNewNodeId][totalNodeNumInGraph - 1].set(GRB_DoubleAttr_LB, 1.0);
                    }
                }
            }
        }

        std::cout << "[DEBUG] 2b: About to set callback (if needed)\n" << std::flush;
        
        if (!pricingSubProblem) {
            model.update();
            std::cout << "[DEBUG] 2c: model.update() called\n" << std::flush;
            // 动态分配Callback对象，使其生命周期延伸到optimize()之后
            cb_ptr = new CallBack(totalNodeNumInGraph, totalPickupNodeNumInGraph, newOriginalNodeIndexMap,
                                  arcVariables, vrpSolution, resultFolder);
            model.setCallback(cb_ptr);
            std::cout << "[DEBUG] 2d: Callback set\n" << std::flush;
        }

        std::cout << "[DEBUG] 3: Model built successfully, vars="
                  << model.get(GRB_IntAttr_NumVars)
                  << ", rows=" << model.get(GRB_IntAttr_NumConstrs)
                  << std::endl << std::flush;

        // 输出.lp模型文件
        try {
            std::string lpFileName = resultFolder;
            if (!lpFileName.empty() && lpFileName.back() != '/') {
                lpFileName += "/";
            }
            lpFileName += "two_index_formulation";
            if (pricingSubProblem) {
                lpFileName += "_pricing_sub_problem";
            } else {
                lpFileName += "_original";
            }
            lpFileName += ".lp";

            std::filesystem::path lpFilePath(lpFileName);
            std::filesystem::create_directories(lpFilePath.parent_path());

            model.write(lpFileName);
            std::cout << "LP model file written to: " << lpFileName << std::endl;
            std::cout << "[DEBUG] 4: LP file written, about to call optimize()\n" << std::flush;
        } catch (const std::exception& e) {
            std::cerr << "Warning: Failed to write LP model file: " << e.what() << std::endl;
            std::cout << "[DEBUG] 4-ERROR: Exception during LP write, but continuing: " << e.what() << "\n" << std::flush;
        }

        // ========== 关键步骤：启用详细日志并调用求解 ==========
        model.getEnv().set(GRB_IntParam_OutputFlag, 1);
        model.getEnv().set(GRB_IntParam_LogToConsole, 1);
        
        std::cout << "[DEBUG] 4b: Before model.optimize(), enabling log output\n" << std::flush;

        // 求解模型
        model.optimize();
        
        std::cout << "[DEBUG] 5: model.optimize() returned, status="
                  << model.get(GRB_IntAttr_Status)
                  << std::endl << std::flush;

        // 输出求解结果
        if (model.get(GRB_IntAttr_SolCount) > 0) {
            std::cout << "[DEBUG] 6: Solution found, objective=" << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
        } else {
            std::cout << "[DEBUG] 6: No solution found\n" << std::flush;
        }


        
    } catch (GRBException e) {
        // 处理Gurobi异常
        std::cout << "[GRBException] Error code = " << e.getErrorCode() << std::endl;
        std::cout << "[GRBException] Message: " << e.getMessage() << std::endl;
        std::cerr << "[GRBException] Error code = " << e.getErrorCode() << std::endl;
        std::cerr << "[GRBException] Message: " << e.getMessage() << std::endl;
    } catch (std::exception& e) {
        // 处理标准异常
        std::cout << "[std::exception] " << e.what() << std::endl;
        std::cerr << "[std::exception] " << e.what() << std::endl;
    } catch (...) {
        // 处理其他异常
        std::cout << "[unknown exception] Error during optimization" << std::endl;
        std::cerr << "[unknown exception] Error during optimization" << std::endl;
    }
    
    // 清理动态分配的Callback对象
    if (cb_ptr != nullptr) {
        delete cb_ptr;
        cb_ptr = nullptr;
        std::cout << "[DEBUG] Callback object deleted\n" << std::flush;
    }

    // 清理内存
    delete[] vars;
}
