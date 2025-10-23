#include "domain/solver/memetic/Memetic.h"

#include <omp.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "domain/solver/memetic/LargeNeighborhoodSearch.h"
#include "domain/solver/memetic/LocalSearch.h"
#include "domain/solver/memetic/crossover/CrossoverStrategy.h"
#include "domain/solver/memetic/crossover/EdgeAssemblyCrossover.h"
#include "domain/solver/memetic/crossover/SrexCrossover.h"
#include "domain/solver/memetic/insert/Insert.h"
#include "domain/solver/memetic/removal/Removal.h"
#include "io/vrp_sol_writer.h"
#include "util/random_utils.h"

void buildInitialSolution(int routeNum, VrpSolution& vrpSolution) {
    vrpSolution.routes.clear();
    vrpSolution.vehicleNum = routeNum;
    int avgRouteNodeNum = (int)vrpSolution.vrpProblem->vrpData.pickNodeIds.size() / vrpSolution.vehicleNum;

    std::vector<int> pickupNodeIds = vrpSolution.vrpProblem->vrpData.pickNodeIds;
    std::shuffle(pickupNodeIds.begin(), pickupNodeIds.end(), GetRandomGenerator());

    for (int i = 0; i < vrpSolution.vehicleNum; ++i) {
        vrpSolution.routes.emplace_back(i);
        RouteNode* prevNode = nullptr;
        int nodeNumInRoute = avgRouteNodeNum;
        if (i == vrpSolution.vehicleNum - 1) {
            nodeNumInRoute = (int)pickupNodeIds.size() - i * avgRouteNodeNum;
        }
        for (int j = i * avgRouteNodeNum; j < i * avgRouteNodeNum + nodeNumInRoute; ++j) {
            int pickupNodeId = pickupNodeIds[j];

            RouteNode* currentNode = &vrpSolution.routeNodes[pickupNodeId];
            RouteNode* currentCorresRouteNode = currentNode->corresRouteNode;

            vrpSolution.insertNode(i, pickupNodeId, currentCorresRouteNode->nodeId, prevNode, prevNode, false);

            prevNode = currentCorresRouteNode;
        }

        vrpSolution.updateRouteData(i, vrpSolution.routes[i].startNode, vrpSolution.routes[i].endNode);
    }
}

void writeBestSol(VrpSolution& bestSolution, const std::string& resultFolder) {
    std::filesystem::path resultFolderPath = resultFolder;
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << bestSolution.getTotalDistance() << "_"
           << bestSolution.getTotalTwViolation() << "_" << bestSolution.getTotalCapacityViolation();
    std::string solQualityStr = stream.str();
    std::string solFileName = bestSolution.vrpProblem->vrpData.name + "." + std::to_string(bestSolution.routes.size()) +
                              "_" + solQualityStr + ".txt";

    resultFolderPath /= solFileName;

    std::string solFilePath = resultFolderPath.string();
    WriteSol(bestSolution, solFilePath);
}

void memetic(VrpConfig& vrpConfig, int routeNum, double timeLimit, const VrpSolution& bks,
             const std::vector<VrpSolution>& initialSols, VrpProblem& vrpProblem, VrpSolution& bestSolution,
             const std::string& resultFolder) {
    std::cout << std::left << std::setw(12) << "Generation" << std::setw(12) << "Distance" << std::setw(20)
              << "TW Violation" << std::setw(20) << "Capacity Violation" << std::setw(20) << "Time" << std::endl;

    omp_set_num_threads(kThreadsNum);

    std::cout.precision(2);
    std::cout.flags(std::ostream::fixed);

    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point currentTime;

    std::vector<VrpSolution> population;
    population.reserve(kPopulationSize);
    for (int i = 0; i < kPopulationSize; ++i) {
        population.emplace_back(&vrpProblem);
    }

    for (int i = 0; i < kPopulationSize; ++i) {
        if (i < initialSols.size()) {
            population[i].copySolution(initialSols[i]);
        } else {
            buildInitialSolution(routeNum, population[i]);
        }
    }

#pragma omp parallel for shared(vrpConfig, population) default(none)
    for (int i = 1; i < kPopulationSize; ++i) {
        localSearch(vrpConfig, population[i]);
    }

    for (int i = 0; i < kPopulationSize; ++i) {
        if (0 == i) {
            bestSolution.copySolution(population[i]);
        } else {
            if (bestSolution.getTotalObjectiveValue(vrpConfig) > population[i].getTotalObjectiveValue(vrpConfig)) {
                bestSolution.copySolution(population[i]);
            }
        }

        if (population[i].getTotalTwViolation() <= 0.0 && population[i].getTotalCapacityViolation() <= 0.0) {
            writeBestSol(population[i], resultFolder);
        }
    }

    writeBestSol(bestSolution, resultFolder);

    currentTime = std::chrono::steady_clock::now();
    double duration = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();

    std::cout << std::left << std::setw(12) << 0 << std::setw(12) << bestSolution.getTotalDistance() << std::setw(20)
              << bestSolution.getTotalTwViolation() << std::setw(20) << bestSolution.getTotalCapacityViolation()
              << std::setw(20) << duration << std::endl;

    CrossoverStrategy crossoverStrategy = CrossoverStrategy::kEax;

    std::vector<int> solIndices(kPopulationSize, 0);
    std::iota(solIndices.begin(), solIndices.end(), 0);

    std::vector<VrpSolution> children;
    children.reserve(kPopulationSize);
    for (int i = 0; i < kPopulationSize; ++i) {
        children.emplace_back(&vrpProblem);
    }

    vrpConfig.twPenaltyCoeff = 1.0;
    vrpConfig.capacityPenaltyCoeff = 0.0;
    for (int i = 1; i < kMaxGeneration; ++i) {
        std::shuffle(solIndices.begin(), solIndices.end(), GetRandomGenerator());

#pragma omp parallel for shared(vrpConfig, population, children, solIndices, startTime, timeLimit) default(none)
        for (int j = 0; j < kPopulationSize; ++j) {
            int firstSolIndex = solIndices[j];
            int secondSolIndex = solIndices[(j + 1) % kPopulationSize];

            double tmpRandValue = GetRandDouble(0.0, 1.0);
            if (tmpRandValue < kSrexProb) {
                srexCrossover(vrpConfig, population[firstSolIndex], population[secondSolIndex], startTime, timeLimit,
                              children[j]);
            } else {
                eax(vrpConfig, population[firstSolIndex], population[secondSolIndex], children[j]);
            }

            bool emptyVehicle = std::any_of(children[j].routes.begin(), children[j].routes.end(),
                                            [](const VrpRoute& route) { return route.nodeNum == 0; });
            if (emptyVehicle &&
                (children[j].getTotalTwViolation() > 0.0 || children[j].getTotalCapacityViolation() > 0.0)) {
                children[j].copySolution(population[firstSolIndex]);
            }
        }

        for (int j = 0; j < kPopulationSize; ++j) {
            if (children[j].getTotalObjectiveValue(vrpConfig) <
                population[solIndices[j]].getTotalObjectiveValue(vrpConfig)) {
                population[solIndices[j]].copySolution(children[j]);

                if (children[j].getTotalObjectiveValue(vrpConfig) < bestSolution.getTotalObjectiveValue(vrpConfig)) {
                    bestSolution.copySolution(children[j]);
                    writeBestSol(bestSolution, resultFolder);
                }
            }

            if (children[j].getTotalTwViolation() <= 0.0 && children[j].getTotalCapacityViolation() <= 0.0) {
                writeBestSol(children[j], resultFolder);
            }
        }

        currentTime = std::chrono::steady_clock::now();
        duration = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();
        std::cout << std::left << std::setw(12) << i << std::setw(12) << bestSolution.getTotalDistance()
                  << std::setw(20) << bestSolution.getTotalTwViolation() << std::setw(20)
                  << bestSolution.getTotalCapacityViolation() << std::setw(20) << duration << std::endl;

        if (bestSolution.getTotalCapacityViolation() <= 0.0) {
            vrpConfig.capacityPenaltyCoeff *= 0.9;
            vrpConfig.capacityPenaltyCoeff = std::max(vrpConfig.capacityPenaltyCoeff, kPenaltyCoeffLower);
        } else {
            vrpConfig.capacityPenaltyCoeff *= 1.1;
            vrpConfig.capacityPenaltyCoeff = std::min(vrpConfig.capacityPenaltyCoeff, kPenaltyCoeffUpper);
        }

        if (bestSolution.getTotalTwViolation() <= 0.0) {
            vrpConfig.twPenaltyCoeff *= 0.9;
            vrpConfig.twPenaltyCoeff = std::max(vrpConfig.twPenaltyCoeff, kPenaltyCoeffLower);
        } else {
            vrpConfig.twPenaltyCoeff *= 1.1;
            vrpConfig.twPenaltyCoeff = std::min(vrpConfig.twPenaltyCoeff, kPenaltyCoeffUpper);
        }

        if (duration > timeLimit) {
            break;
        }
    }
}