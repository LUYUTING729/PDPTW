#include "domain/solver/exact/SetPartitioning.h"
#include <unordered_set>
#include <vector>
#include "SetPartitioning.h"

void SetPartitioning::readStartingRoutes(std::string file_name, std::vector<std::vector<int>>& xy_relate,
                                         std::vector<double>& net_distance_vec) {
    xy_relate.resize(nX);

    std::string tmp_str;
    std::ifstream col_file(file_name);
    std::string delimiter1 = ":";
    std::string delimiter2 = " ";

    while (getline(col_file, tmp_str)) {
        int current_route = routes.size();
        double net_distance = 0;
        size_t pos = 0;
        std::string token;
        std::vector<int> tmp_route;
        while ((pos = tmp_str.find(delimiter1)) != std::string::npos) {
            token = tmp_str.substr(0, pos);
            tmp_str.erase(0, pos + delimiter1.length());
        }
        int pointer = 0;
        while ((pos = tmp_str.find(delimiter2)) != std::string::npos) {
            token = tmp_str.substr(0, pos);
            if (token.size() != 0) {
                int next_pos = std::stoi(token);
                double tmp_dist = getDistance(pointer, next_pos);
                net_distance += tmp_dist;
                pointer = next_pos;
                tmp_route.push_back(pointer);
                xy_relate[pointer].push_back(current_route);
            }
            tmp_str.erase(0, pos + delimiter2.length());
        }
        // need to deal with the last entity
        int next_pos = std::stoi(tmp_str);
        double tmp_dist = getDistance(pointer, next_pos);
        net_distance += tmp_dist;
        pointer = next_pos;
        tmp_route.push_back(pointer);
        xy_relate[pointer].push_back(current_route);
        // get the distance back to depot
        tmp_dist = getDistance(pointer, 0);
        net_distance += tmp_dist;
        routes.push_back(tmp_route);
        net_distance_vec.push_back(net_distance);
    }

    col_file.close();

    std::vector<double> timePenalty = validateRoute();
    for (int i=0; i < routes.size(); i++) 
        net_distance_vec[i] += timePenalty[i];
}

void SetPartitioning::createSP(std::string file_name) {
    std::vector<std::vector<int>> xy_relate;
    std::vector<double> net_distance_vec;
    
    routes.clear();
    readStartingRoutes(file_name, xy_relate, net_distance_vec);

    if (model) {
        delete model;
        delete (env);
        env = new GRBEnv();
        model = new GRBModel(*env);
        spVars.clear();
        spConstrs.clear();
    }
    
    for (int current_route = 0; current_route < routes.size(); current_route++) {
        std::string varName = "y_" + std::to_string(current_route);
        GRBVar newVar;
        if (maxRoute >= 0){ 
            newVar = model->addVar(0, 1.0, net_distance_vec[current_route], GRB_CONTINUOUS, varName);
        }else
        {
            newVar = model->addVar(0, 1.0, net_distance_vec[current_route] + 10000, GRB_CONTINUOUS, varName);
        }
        spVars.push_back(newVar);
    }
    
    // add constraint
    for (int i = 0; i<xy_relate.size(); ++i){
        if (xy_relate[i].size() == 0){
            continue;
        }
        GRBLinExpr linExpr;
        for (int j = 0; j<xy_relate[i].size(); ++j){
            // std::cout<<i<<"  "<<xy_relate[i][j]<<"\n";
            linExpr += spVars[xy_relate[i][j]];
        }
        std::string constrName = "C" + std::to_string(i);
        if (vd->nodeList[i].demand > 1e-5 )
            spConstrs[i] = model->addConstr(linExpr, GRB_GREATER_EQUAL, 1.0, constrName);
    }

    std::cout << "maxRoute:::: " << maxRoute << "\n";
    // add route size constraint
    if (maxRoute >= 0){
        GRBLinExpr linExpr2;
        GRBVar slackVar;
        slackVar = model->addVar(0, 500, 10000, GRB_CONTINUOUS, "slackVar");
        for (int j = 0; j < spVars.size(); ++j) {
            linExpr2 += spVars[j];
        }
        std::string constrName = "C0000";
        spConstrs[0] = model->addConstr(linExpr2 - slackVar, GRB_LESS_EQUAL, maxRoute, constrName);
    }    
    model->update();
    model->write("rmp.lp");
}


double SetPartitioning::solveTwoInd(const std::vector<double> &rho_kz, std::vector<std::vector<int> >& sol_vec){

    std::ifstream ifs(configFileName);
    if (!ifs.is_open()) {
        std::cout << "Failed to open " << configFileName << std::endl;
        ifs.close();
    }
    nlohmann::json configJson;
    ifs >> configJson;
    ifs.close();

    std::string resultFolder = configJson.at("resultFolder");

    double res;
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
        try {
            pricingSubProblemData.pickupNodeNumLimitInRoute =
                configJson.at("pickupNodeNumLimitInRouteForPricingSubProblem");
        } catch(...) {
            int maxNumNodes=-1;
            for (int i =0; i<initialSols.back().routes.size();i++)
            {
                if (maxNumNodes<initialSols.back().routes[i].nodeNum/2)
                    maxNumNodes = initialSols.back().routes[i].nodeNum/2;
            }
            pricingSubProblemData.pickupNodeNumLimitInRoute = maxNumNodes;
        }
        PricingSubProblemSolution pricingSubProblemSolution;
        for (int i=0;i < vrpProblem.vrpData.nodeNum; ++i){
            pricingSubProblemData.dualValueForPickupNodes[i] = rho_kz[i];
        }
        if (initialSols.empty()) {
            // not possible, as bks has been pushed first
        } else {
            res= twoIndexFormulationSolveZ(vrpConfig, &vrpProblem, &initialSols.back(), freeNodes, freeNodeNum, freeRouteIndices,
                                     timeLimit, true, pricingSubProblemData, pricingSubProblemSolution, newSol,
                                     resultFolder, sol_vec);
        }
        std::cout<<"!!!!Current sol having " << sol_vec[sol_vec.size()-1].size() << ", " <<pricingSubProblemSolution.nodesInRoute.size()<<" nodes in route\n";
    }
    return res;
}

std::vector<double> SetPartitioning::validateRoute() {
    std::vector<double> timePenalty(routes.size(),0.0);
    double max_cap = vd->vehicleCapacity;
    // validate routes 1 by 1
    for (int i=0;i<routes.size();++i){
        int pointer = 0;
        double current_time = 0;
        double current_load = 0;
        Utils::VrpNodeType last_type = Utils::VrpNodeType::kDepot;
        std::unordered_set<int> pickups;
        int flag = 0;
        for (int j=0;j<routes[i].size();++j){
            int next_pos = routes[i][j];
            Utils::VrpNodeType currentType = vd->nodeList[next_pos].nodeType;
            double distance = getDistance(pointer,next_pos);
            current_time += distance;
            // update and check for capacity
            if (Utils::VrpNodeType::kPickup == currentType){
                current_load += vd->nodeList[next_pos].demand; 
                if (current_load >max_cap){
                    std::cout<<"Verifying route "<<i+1<<"...   "<<" invalid::load exceed --- "<<current_load<<"/"<<max_cap<<"\n";
                    flag = 1;
                    break;
                }
                pickups.insert(next_pos);
            }else if (Utils::VrpNodeType::kDelivery == currentType){
                current_load += vd->nodeList[next_pos].demand; 
                // check if already picked up
                if (pickups.find(vd->nodeList[next_pos].corresNodeId)==pickups.end()){
                    std::cout<<"Verifying route "<<i+1<<"...   "<<" invalid::delivery before pickup\n";
                    flag = 1;
                    break;
                }else{
                    pickups.erase(pickups.find(vd->nodeList[next_pos].corresNodeId));
                }
            }
            // update time and check 
            if (current_time > vd->nodeList[next_pos].timeWindow.second){
                // cannot arrive in time
                std::cout<<"Verifying route "<<i<<"...   "<<" invalid::time window does not match\n";
                std::cout<<"   current node: "<<next_pos<<"  current time: "<<current_time<<"  time window:["<<vd->nodeList[next_pos].timeWindow.first<<","<<vd->nodeList[next_pos].timeWindow.second<<"]\n";
                flag = 1;
                break;
            }
            if (current_time < vd->nodeList[next_pos].timeWindow.first){
                current_time = vd->nodeList[next_pos].timeWindow.first;
            }
            timePenalty[i] += (current_time - vd->nodeList[next_pos].timeWindow.first)/10000;
            if (j> 0 && refNodeSericeStartTimes[pointer] > refNodeSericeStartTimes[next_pos] )
                timePenalty[i] += 10;
            current_time += vd->nodeList[next_pos].serveTime;

            pointer = next_pos;
        }

        for (int k=0;k<1;++k){
            int next_pos = 0;
            Utils::VrpNodeType currentType = vd->nodeList[next_pos].nodeType;
            double distance = getDistance(pointer,next_pos);
            current_time += distance;
            // update and check for capacity
            if (Utils::VrpNodeType::kPickup == currentType){
                current_load += vd->nodeList[next_pos].demand; 
                if (current_load >max_cap){
                    std::cout<<"Verifying route "<<i<<"...   "<<" invalid::load exceed --- "<<current_load<<"/"<<max_cap<<"\n";
                    flag = 1;
                    break;
                }
                pickups.insert(next_pos);
            }else if (Utils::VrpNodeType::kDelivery == currentType){
                current_load += vd->nodeList[next_pos].demand; 
                // check if already picked up
                if (pickups.find(vd->nodeList[next_pos].corresNodeId)==pickups.end()){
                    std::cout<<"Verifying route "<<i<<"...   "<<" invalid::delivery before pickup\n";
                    flag = 1;
                    break;
                }else{
                    pickups.erase(pickups.find(vd->nodeList[next_pos].corresNodeId));
                }
            }
            // update time and check 
            if (current_time > vd->nodeList[next_pos].timeWindow.second){
                // cannot arrive in time
                std::cout<<"Verifying route "<<i<<"...   "<<" invalid::time window does not match\n";
                std::cout<<"   current node: "<<next_pos<<"  current time: "<<current_time<<"  time window:["<<vd->nodeList[next_pos].timeWindow.first<<""<<vd->nodeList[next_pos].timeWindow.second<<"]\n";
                flag = 1;
                break;
            }
            if (current_time < vd->nodeList[next_pos].timeWindow.first){
                current_time = vd->nodeList[next_pos].timeWindow.first;
            }
            current_time += vd->nodeList[next_pos].serveTime;

            pointer = next_pos;
        }

        if (flag==0){
            // check if still not delivered
            if (pickups.size()!=0){
                std::cout<<"Verifying route "<<i<<"...   "<<" invalid::not all cargos are delivered\n";
                continue;
            }
            // std::cout<<"Verifying route "<<i<<"...   "<<"valid \n";
        }
    }
    return timePenalty;
}