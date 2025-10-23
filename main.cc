#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "config.h"

#ifdef USE_GUROBI
#include "domain/solver/exact/TwoIndexFormulation.h"
#endif

#include "domain/solver/memetic/LargeNeighborhoodSearch.h"
#include "domain/solver/memetic/Memetic.h"
#include "domain/solver/memetic/insert/Insert.h"
#include "domain/solver/memetic/removal/Removal.h"
#include "io/pdptw_reader.h"
#include "io/sol_reader.h"
#include "model/pdptw.h"
#include "model/solution.h"
#include "util/convert_utils.h"
#include "util/string_utils.h"

void sisr(VrpProblem& vrpProblem, int vehicleNum) {
    VrpSolution vrpSolution(&vrpProblem);
    buildInitialSolution(vehicleNum, vrpSolution);

    VrpConfig vrpConfig;
    double initialTemperature = 100.0;
    double finalTemperature = 1.0;
    int iterationNum = 100000;
    double coolingRate = std::pow(finalTemperature / initialTemperature, 1.0 / iterationNum);
    VrpSolution currentSol(vrpSolution.vrpProblem);
    currentSol.copySolution(vrpSolution);
    VrpSolution newSol(vrpSolution.vrpProblem);
    double currentTemperature = initialTemperature;

    std::cout << std::left << std::setw(12) << "Iteration" << std::setw(12) << "Distance" << std::setw(20)
              << "TW Violation" << std::setw(20) << "Capacity Violation" << std::setw(20) << "Time" << std::endl;

    std::cout.precision(2);
    std::cout.flags(std::ostream::fixed);

    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point currentTime;
    double duration;
    for (int i = 0; i < iterationNum; ++i) {
        newSol.copySolution(currentSol);

        currentTemperature *= coolingRate;

        stringRemoval(vrpConfig, newSol);
        greedyInsertWithBlinks(vrpConfig, newSol, true);
        insertEmptyRoute(newSol);

        double currentObj = currentSol.getTotalObjectiveValue(vrpConfig);
        double newObj = newSol.getTotalObjectiveValue(vrpConfig);

        if (newObj < currentObj) {
            currentSol.copySolution(newSol);
            if (newObj < vrpSolution.getTotalObjectiveValue(vrpConfig)) {
                vrpSolution.copySolution(newSol);
            }
        } else {
            double randNum = GetRandDouble(0.0, 1.0);
            double delta = newObj - currentObj;
            double prob = std::exp(-delta / currentTemperature);
            if (randNum < prob) {
                currentSol.copySolution(newSol);
            }
        }

        if (0 == i % 1000 || i == iterationNum - 1) {
            currentTime = std::chrono::steady_clock::now();
            duration = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();

            std::cout << std::left << std::setw(12) << i << std::setw(12) << vrpSolution.getTotalDistance()
                      << std::setw(20) << vrpSolution.getTotalTwViolation() << std::setw(20)
                      << vrpSolution.getTotalCapacityViolation() << std::setw(20) << duration << std::endl;
        }
    }
}

void writeFeasibleArcs(const VrpSolution& vrpSolution) {
    std::ofstream ofs;
    ofs.open(vrpSolution.vrpProblem->vrpData.name + "_feasible_arcs.txt", std::ios::out);
    for (int i = 0; i < vrpSolution.vrpProblem->vrpData.nodeNum; ++i) {
        for (int j : vrpSolution.vrpProblem->vrpData.nodeList[i].candiNextNodes) {
            ofs << i << " " << j << std::endl;
        }
    }

    ofs.close();
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printf("Invalid argument number.\n");
        exit(0);
    }
    std::string configFileName = argv[1];
    std::ifstream ifs(configFileName);
    if (!ifs.is_open()) {
        std::cout << "Failed to open " << configFileName << std::endl;
        ifs.close();
        return 1;
    }
    nlohmann::json configJson;
    ifs >> configJson;
    ifs.close();

    std::string instanceName = configJson.at("instanceName");
    int additionalVehicleNum = configJson.at("additionalVehicleNum");
    double timeLimit = configJson.at("timeLimit");
    std::string resultFolder = configJson.at("resultFolder");
    std::vector<std::string> initialSolFilePaths;
    for (const auto& i : configJson.at("initialSolFiles")) {
        initialSolFilePaths.push_back(i);
    }

    std::cout << std::left << std::setw(20) << "Instance Name: " << instanceName << std::endl;
    std::cout << std::left << std::setw(20) << "Time Limit: " << timeLimit << std::endl;
    std::cout << std::left << std::setw(20) << "Result Folder: " << resultFolder << std::endl;
    std::cout << std::left << std::setw(20) << "Initial Sol Files: " << std::endl;
    for (int i = 0; i < initialSolFilePaths.size(); ++i) {
        std::cout << i << ": " << initialSolFilePaths[i] << std::endl;
    }

    std::filesystem::path instanceFilePath = AMDAHL_DIR;
    instanceFilePath /= "data";
    instanceFilePath /= "instance";
    instanceFilePath /= instanceName + ".txt";
    PDPTW problem;
    ReadPDPTW(instanceFilePath.string(), problem);

    instanceName = ToLower(instanceName);

    std::filesystem::path bks_folder = AMDAHL_DIR;
    bks_folder /= "data";
    bks_folder /= "bks";

    std::filesystem::path bksFilePath;
    for (const auto& file : std::filesystem::directory_iterator(bks_folder)) {
        std::string file_path = file.path().string();
        std::string file_name = file.path().stem().string();
        if (file_name.size() < instanceName.size()) {
            continue;
        }

        if (0 != file_name.compare(0, instanceName.size(), instanceName, 0, instanceName.size())) {
            continue;
        }

        if ("lr2_10_5" == instanceName) {
            std::string bksName = "lr2_10_5.13_55038.21";
            if (0 != file_name.compare(0, bksName.size(), bksName, 0, bksName.size())) {
                continue;
            }
        }

        if (file_name.size() > instanceName.size()) {
            char tmp_char = file_name[instanceName.size()];
            if ('0' <= tmp_char && '9' >= tmp_char) {
                continue;
            }
        }

        bksFilePath = file.path();
    }

    Solution solution(problem);
    ReadSol(bksFilePath.string(), solution);

    VrpProblem vrpProblem;
    ConvertVrpProblem(problem, vrpProblem);

    VrpSolution bks(&vrpProblem);
    ConvertVrpSolution(solution, bks);

    std::cout.precision(2);
    std::cout.flags(std::ostream::fixed);
    std::cout << "BKS: " << bks.getTotalDistance() << ", " << bks.getTotalTwViolation() << ", "
              << bks.getTotalCapacityViolation() << std::endl;
    std::vector<VrpSolution> initialSols;
    for (int i = 0; i < initialSolFilePaths.size(); ++i) {
        Solution initSol(problem);
        ReadSol(initialSolFilePaths[i], initSol);

        initialSols.emplace_back(&vrpProblem);
        ConvertVrpSolution(initSol, initialSols[i]);
    }
#ifdef USE_GUROBI
    if ("two_index" == configJson.at("solverType")) {
        VrpConfig vrpConfig;
        VrpSolution newSol(&vrpProblem);

        bool freeNodes = configJson.at("freeNodes");
        int freeNodeNum = configJson.at("freeNodesNum");
        double timeLimit = configJson.at("timeLimit");

        std::vector<int> freeRouteIndices;
        for (const auto& i : configJson.at("freeRouteIndices")) {
            freeRouteIndices.push_back(i);
        }
        PricingSubProblemData pricingSubProblemData;
        pricingSubProblemData.pickupNodeNumLimitInRoute = 22;
        //    configJson.at("pickupNodeNumLimitInRouteForPricingSubProblem");
        PricingSubProblemSolution pricingSubProblemSolution;
        std::unordered_map<int, double> dualValueForPickupNodes;

        if (initialSols.empty()) {
            twoIndexFormulationSolve(vrpConfig, &vrpProblem, &bks, freeNodes, freeNodeNum, freeRouteIndices, timeLimit,
                                     false, pricingSubProblemData, pricingSubProblemSolution, newSol, resultFolder);
        } else {
            twoIndexFormulationSolve(vrpConfig, &vrpProblem, &initialSols[0], freeNodes, freeNodeNum, freeRouteIndices,
                                     timeLimit, false, pricingSubProblemData, pricingSubProblemSolution, newSol,
                                     resultFolder);
        }

        return 0;
    }
#endif

    //    sisr(vrpProblem, bks.vehicleNum + additionalVehicleNum);

    VrpConfig vrpConfig;
    vrpConfig.vehicleCost = 0.0;
    vrpConfig.twPenaltyCoeff = 20000.0;
    vrpConfig.capacityPenaltyCoeff = 20000.0;
    VrpSolution memeticSol(&vrpProblem);
    memetic(vrpConfig, bks.vehicleNum + additionalVehicleNum, timeLimit, bks, initialSols, vrpProblem, memeticSol,
            resultFolder);
}
