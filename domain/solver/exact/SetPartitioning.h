#ifndef AMDAHL_SRC_DOMAIN_SOLVER_EXACT_SETPARTITIONING_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_EXACT_SETPARTITIONING_H_

#include <vector>
#include <unordered_set>
#include <queue>
#include <utility>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <third_party/nlohmann/json.hpp>
#include <string>

#include "config.h"
#include "domain/model/VrpProblem.h"
#include "domain/model/VrpSolution.h"
#include "domain/model/entity/VrpConfig.h"
#include "domain/model/entity/VrpRoute.h"
#include "domain/utils/Utils.h"
#include "util/distance_utils.h"
#include <gurobi_c++.h>
#include "domain/solver/exact/TwoIndChecker.h"
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

class SetPartitioning {
   public:
    double gloabl_lb;
    std::vector<double> pi_k;
    std::vector<double> rho_k;

    std::vector<std::vector<int>> routes;

    GRBEnv* env;
    GRBModel* model;
    std::vector<GRBVar> spVars;
    std::unordered_map<int,GRBConstr> spConstrs;

    VrpProblem * vp;
    VrpData * vd;

    int nX;

    double lambda=0.04;

    int maxRoute = -1;

    std::string cf;

    // Constructor
    SetPartitioning(VrpData * vd_ori, VrpProblem * vp_ori, std::string config_file, std::string col_file, int maxRoute1){
        setRoute(maxRoute1);
        vp = vp_ori;
        vd = vd_ori;
        nX = vd->nodeList.size();
        rho_k.resize(nX, 0);
        pi_k.resize(nX,0);
        fill(pi_k.begin(),pi_k.end(),0.0);
        env = new GRBEnv();
        model = new GRBModel(*env);

        configFileName = config_file;
        cf = col_file;
        init();
    };

    void setRoute(int mr){maxRoute = mr;};

    std::vector<double> validateRoute();

    void readStartingRoutes(std::string file_name, std::vector<std::vector<int>>& xy_relate,
                                                   std::vector<double>& net_distance_vec);
    void createSP(std::string file_name);

    double getDistance(int x, int y){
        double res = (vd->nodeList[x].lat - vd->nodeList[y].lat)*(vd->nodeList[x].lat - vd->nodeList[y].lat)+(vd->nodeList[x].lon - vd->nodeList[y].lon)*(vd->nodeList[x].lon - vd->nodeList[y].lon);
        res = std::sqrt(res);
        return res;
    };

    void solveSP(){
        createSP(cf);
        // solve and update dual value
        model->optimize(); 
        int starter = 0;
        if (maxRoute < 0){
            starter = 1;
        }
        for (int i=starter;i<nX;++i){
            double dv;
            if (vd->nodeList[i].demand >= -1e-5) {
                dv = spConstrs[i].get(GRB_DoubleAttr_Pi);
                rho_k[i] = dv;
            }
        }
        std::cout << "current SP objective: " << model->get(GRB_DoubleAttr_ObjVal) << std::endl;
    }

    double solveTwoInd(const std::vector<double> &rho_kz, std::vector<std::vector<int> > &sol_vec);

    void setInitLambda(double init_lambda){
        lambda = init_lambda;
    }

    void binSearch(int iter){
        std::vector<double> rho_current(rho_k);
        double EPS=.0003;

        std::vector<int> sol_vecZ;
        std::vector<std::vector<int> > sol_vec_vec;

        while (true){
            // std::vector<int> sol_vec;
            // sol_vecZ.resize(0);
            sol_vec_vec.clear();
            double tmp = solveTwoInd(rho_current, sol_vec_vec);
            sol_vecZ = sol_vec_vec[sol_vec_vec.size()-1];
            double dual_sum_rho_current = rho_current[0];
            double dual_sum_pi_k = pi_k[0];
            std::cout << "working dual0 = "<< dual_sum_rho_current << ", feasible dual0 = " << dual_sum_pi_k << std::endl;
            for (int i = 0; i < sol_vecZ.size(); ++i) {
                dual_sum_rho_current += rho_current[sol_vecZ[i]];
                dual_sum_pi_k += pi_k[sol_vecZ[i]];
            }
            double column_cost = tmp + dual_sum_rho_current;
            // int x; 
            // std::cout << "Wait for input::::"; // Type a number and press enter
            // std::cin >> x; // Get user input from the keyboard

            for (int newRouteIter =0; newRouteIter < sol_vec_vec.size(); newRouteIter++) {
                std::string tmp_str = "Route ";
                sol_vecZ = sol_vec_vec[newRouteIter];
                tmp_str += std::to_string(routes.size()) + " :";
                // std::cout<<"Route XXX :";
                for (int i = 0; i < sol_vecZ.size(); ++i) {
                    // std::cout<<" "<<sol_vecZ[i];
                    tmp_str += " " + std::to_string(sol_vecZ[i]);
                }
                // std::cout<<"\n";
                std::cout << tmp_str << "\n";

                std::ofstream outfile;
                outfile.open(cf, std::ios_base::app);  // append instead of overwrite
                outfile << tmp_str << std::endl;
                outfile.close();
                routes.push_back(sol_vecZ);
            }
            if (tmp <= -EPS) {
                double new_lambda = 1 + tmp / (column_cost - dual_sum_pi_k-tmp);
                lambda *= new_lambda;
                std::vector<double> test(rho_current);
                for (int i = 0; i < rho_current.size(); ++i) {
                    rho_current[i] = (1 - new_lambda) * pi_k[i] + new_lambda * rho_current[i];
                    test[i] = (1 - lambda) * pi_k[i] + lambda * rho_k[i];
                    if (abs(rho_current[i] - test[i]) > .0001)
                    {
                        std::cout << "error too big" << std::endl;
                    }
                }
                std::cout << "OBJ: " << tmp << "   Going down\n    <<New lambda: " << lambda << "\n";
            } else {
                std::cout << "best lambda: " << lambda << "\n";
                break;
            }
        }

        // int z; 
        // std::cout << "Wait for input::::"; // Type a number and press enter
        // std::cin >> z; // Get user input from the keyboard

        double dual_sum = rho_current[0] * maxRoute;
        for (int i = 1; i < pi_k.size(); i++) 
            dual_sum += rho_current[i];
        std::cout << "Finishing iteration " << iter << ", the current lower bound is:" << dual_sum << std::endl;

        // update global pi
        for (int i=0;i<pi_k.size();++i){
            pi_k[i] = rho_current[i];
        }
    }

    VrpProblem vrpProblem;
    //VrpSolution bks;
    std::vector<VrpSolution> initialSols;
    std::string configFileName;
    std::vector<double> refNodeSericeStartTimes;

    void init() {
        std::ifstream ifs(configFileName);
        if (!ifs.is_open()) {
            std::cout << "Failed to open " << configFileName << std::endl;
            ifs.close();
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

        ConvertVrpProblem(problem, vrpProblem);
        std::cout<<"ZZZZZZZZ\n";
        int total_pickup = 0;
        for (int i=0;i<vrpProblem.vrpData.nodeList.size();++i){
            if (vrpProblem.vrpData.nodeList[i].nodeType == Utils::VrpNodeType::kPickup){
                total_pickup+=1;
            }
        }
        std::cout<<total_pickup<<"\n";

        std::cout.precision(4);
        std::cout.flags(std::ostream::fixed);
        //std::vector<VrpSolution> initialSols;
        for (int i = 0; i < initialSolFilePaths.size(); ++i) {
            Solution initSol(problem);
            ReadSol(initialSolFilePaths[i], initSol);

            initialSols.emplace_back(&vrpProblem);
            ConvertVrpSolution(initSol, initialSols[i]);
            std::cout << "Initial solution: " << i << ": " << initialSols[i].getTotalDistance() << ", " <<
                initialSols[i].getTotalTwViolation() << ", " << initialSols[i].getTotalCapacityViolation() << std::endl;
        }
        initialSols.emplace_back(&vrpProblem);
        Solution solution(problem);
        ReadSol(bksFilePath.string(), solution);
        ConvertVrpSolution(solution, initialSols.back());
        maxRoute=initialSols.back().routes.size();
        std::cout << "BKS: " << initialSols.back().getTotalDistance() << ", " << initialSols.back().getTotalTwViolation() << ", "
                  << initialSols.back().getTotalCapacityViolation() << std::endl;

        VrpSolution *initSolution = &initialSols[1];
        int totalNodeNumInGraph = vrpProblem.vrpData.nodeNum + 1;
        refNodeSericeStartTimes.resize(totalNodeNumInGraph,0.0);
        for (int i = 0; i < initSolution->vehicleNum; ++i) {
            RouteNode *currentNode = initSolution->routes[i].startNode;
            int j=0;
            while (currentNode != NULL) {
                refNodeSericeStartTimes[currentNode->nodeId] = currentNode->latestStartServiceTime;
                currentNode = currentNode->nexNode;
                j++;
            }
        }
    }
};

#endif