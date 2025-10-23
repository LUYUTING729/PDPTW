#include <filesystem>
#include <iostream>
#include <string>

#include "config.h"
#include "domain/solver/memetic/insert/Insert.h"
#include "domain/solver/memetic/removal/Removal.h"
#include "domain/utils/Utils.h"
#include "io/pdptw_reader.h"
#include "io/sol_reader.h"
#include "model/pdptw.h"
#include "model/solution.h"
#include "util/convert_utils.h"
#include "util/string_utils.h"
#include "util/distance_utils.h"
#include "domain/solver/hgs/ox.h"

int main2(int argc, char* argv[]) {

    std::cout<<"TESTING SCRIPT, DELETE WHEN FINSIHED DEBUGGING\n";

    std::cout<<"1....TESTING CREATING NODE\n";
    // int id;
    // double lat;
    // double lon;
    // Utils::VrpNodeType nodeType;
    // double demand;
    // double serveTime;
    // std::pair<double, double> timeWindow;
    // int corresNodeId;
    // std::set<int> candiPreNodes;
    // std::set<int> candiNextNodes;
    VrpNode vn0;
    vn0.id = 0;
    vn0.lat = 0.0;
    vn0.lon = 0.0;
    vn0.nodeType = Utils::VrpNodeType::kDepot;
    vn0.demand = 0.0;
    vn0.serveTime = 0.0;
    vn0.timeWindow.first = 0.0;
    vn0.timeWindow.second = 1000.0;
    vn0.corresNodeId = 0;
    VrpNode vn1;
    vn1.id = 1;
    vn1.lat = 1.0;
    vn1.lon = 1.0;
    vn1.nodeType = Utils::VrpNodeType::kPickup;
    vn1.demand = 10.0;
    vn1.serveTime = 2.0;
    vn1.timeWindow.first = 4.0;
    vn1.timeWindow.second = 8.0;
    vn1.corresNodeId = 2;
    VrpNode vn2;
    vn2.id = 2;
    vn2.lat = -1.0;
    vn2.lon = 1.0;
    vn2.nodeType = Utils::VrpNodeType::kDelivery;
    vn2.demand = 10.0;
    vn2.serveTime = 2.0;
    vn2.timeWindow.first = 7.0;
    vn2.timeWindow.second = 14.0;
    vn2.corresNodeId = 1;
    VrpNode vn3;
    vn3.id = 3;
    vn3.lat = -1.0;
    vn3.lon = -1.0;
    vn3.nodeType = Utils::VrpNodeType::kPickup;
    vn3.demand = 10.0;
    vn3.serveTime = 2.0;
    vn3.timeWindow.first = 1.0;
    vn3.timeWindow.second = 14.0;
    vn3.corresNodeId = 4;
    VrpNode vn4;
    vn4.id = 4;
    vn4.lat = 1.0;
    vn4.lon = -1.0;
    vn4.nodeType = Utils::VrpNodeType::kDelivery;
    vn4.demand = 10.0;
    vn4.serveTime = 2.0;
    vn4.timeWindow.first = 7.0;
    vn4.timeWindow.second = 14.0;
    vn4.corresNodeId = 3;


    std::cout<<"2....TESTING CREATING DATA and PROBLEM\n";
    // int nodeNum;
    // int vehicleUpLimit;
    // double vehicleCapacity;
    // std::vector<VrpNode> nodeList;
    // std::vector<std::vector<double>> disMatrix;
    // std::vector<int> pickNodeIds;
    // std::string name;
    VrpData vd;
    vd.nodeNum = 5;
    vd.vehicleUpLimit = 4;
    vd.vehicleCapacity = 120;
    vd.nodeList.push_back(vn0);
    vd.nodeList.push_back(vn1);
    vd.nodeList.push_back(vn2);
    vd.nodeList.push_back(vn3);
    vd.nodeList.push_back(vn4);
    vd.pickNodeIds.push_back(1);
    vd.pickNodeIds.push_back(3);
    // std::vector<std::vector<double>> disMatrix;
    for (int i=0; i<5; ++i){
        vd.disMatrix.emplace_back(0);
        for (int j = 0; j<5; ++j){
            if (i==j){
                continue;
            }
            double ddd = GetEuclideanDis(vd.nodeList[i].lon,vd.nodeList[j].lon,vd.nodeList[i].lat,vd.nodeList[j].lat);
            vd.disMatrix[i].emplace_back(ddd);
        }
    }
    // VrpData vrpData;
    // std::vector<std::vector<int>> adjacentList;
    VrpProblem vp;
    vp.vrpData = vd;


    std::cout<<"2....TESTING CREATING SOLUTION\n";
    VrpSolution vs(&vp);
    vs.routes.clear();
    vs.vehicleNum = 2;
    RouteNode* prevNode = nullptr;
    int i=0;
    vs.routes.emplace_back(i);
    int pickupNodeId = vp.vrpData.pickNodeIds[0];
    RouteNode* currentNode = &vs.routeNodes[pickupNodeId];
    RouteNode* currentCorresRouteNode = currentNode->corresRouteNode;
    vs.insertNode(i, pickupNodeId, currentCorresRouteNode->nodeId, prevNode, prevNode, false);
    vs.updateRouteData(i, vs.routes[i].startNode, vs.routes[i].endNode);

    i=1;
    vs.routes.emplace_back(i);
    pickupNodeId = vp.vrpData.pickNodeIds[1];
    currentNode = &vs.routeNodes[pickupNodeId];
    currentCorresRouteNode = currentNode->corresRouteNode;
    // std::cout<<pickupNodeId<<"  "<<currentNode->nodeId<<"  "<<currentCorresRouteNode->nodeId<<"  "<<(prevNode==nullptr)<<"\n";
    vs.insertNode(i, pickupNodeId, currentCorresRouteNode->nodeId, prevNode, prevNode, false);
    vs.updateRouteData(i, vs.routes[i].startNode, vs.routes[i].endNode);

    std::cout<<"Solution created with feasibility: "<<vs.feasibility<<"\n";



    VrpSolution vs2(&vp);
    vs2.routes.clear();
    vs2.vehicleNum = 1;
    prevNode = nullptr;
    i=0;
    vs2.routes.emplace_back(i);
    pickupNodeId = vp.vrpData.pickNodeIds[0];
    currentNode = &vs2.routeNodes[pickupNodeId];
    currentCorresRouteNode = currentNode->corresRouteNode;
    vs2.insertNode(i, pickupNodeId, currentCorresRouteNode->nodeId, prevNode, prevNode, false);
    pickupNodeId = vp.vrpData.pickNodeIds[1];
    currentNode = &vs2.routeNodes[pickupNodeId];
    currentCorresRouteNode = currentNode->corresRouteNode;
    vs2.insertNode(i, pickupNodeId, currentCorresRouteNode->nodeId, prevNode, prevNode, false);
    vs2.updateRouteData(i, vs2.routes[i].startNode, vs2.routes[i].endNode);
    std::cout<<"Solution created with feasibility: "<<vs2.feasibility<<"\n";


    orderXover ox(2, &vd, &vp);
    ox.execute(&vs,&vs2, 100.0,100.0);
}


void sisr(VrpSolution& vrpSolution) {
    VrpSolution newSol(vrpSolution.vrpProblem);
    newSol.copySolution(vrpSolution);

    printf("%.10f, %.10f, %.10f\n", vrpSolution.getTotalDistance(), vrpSolution.getTotalTwViolation(),
           vrpSolution.getTotalCapacityViolation());
    VrpConfig vrpConfig;
    for (int i = 0; i < 1000; ++i) {
        stringRemoval(vrpConfig, newSol);
        greedyInsertWithBlinks(vrpConfig, newSol, true);

        printf("%d, %.10f, %.10f, %.10f\n", i, newSol.getTotalDistance(), newSol.getTotalTwViolation(),
               newSol.getTotalCapacityViolation());

        if (newSol.getTotalObjectiveValue(vrpConfig) < vrpSolution.getTotalObjectiveValue(vrpConfig)) {
            vrpSolution.copySolution(newSol);
        } else {
            newSol.copySolution(vrpSolution);
        }
    }
}

int main(int argc, char* argv[]) {
    std::string instanceName = "LR2_10_5";
    // instanceName = "LR2_10_4";
    std::filesystem::path instanceFilePath = AMDAHL_DIR;
    instanceFilePath /= "data";
    instanceFilePath /= "instance";
    instanceFilePath /= instanceName + ".txt";
    PDPTW problem;
    ReadPDPTW(instanceFilePath.string(), problem);

    // Get solution 1 bks
    std::filesystem::path solutionFilePath = AMDAHL_DIR;
    solutionFilePath /= "data";
    solutionFilePath /= "bks";
    solutionFilePath /= ToLower(instanceName) + ".13_55038.21.txt";
    // solutionFilePath /= ToLower(instanceName) + ".8_26595.39.txt";
    Solution solution(problem);
    ReadSol(solutionFilePath.string(), solution);

    VrpProblem vrpProblem;
    ConvertVrpProblem(problem, vrpProblem);

    VrpSolution bks(&vrpProblem);
    ConvertVrpSolution(solution, bks);


    // Get solution 2 bks
    solutionFilePath = AMDAHL_DIR;
    solutionFilePath /= "data";
    solutionFilePath /= "bks";
    solutionFilePath /= ToLower(instanceName) + ".13_55197.1290.txt";
    // solutionFilePath /= ToLower(instanceName) + ".8_26595.39.txt";
    Solution solution2(problem);
    ReadSol(solutionFilePath.string(), solution2);

    VrpSolution bks2(&vrpProblem);
    ConvertVrpSolution(solution, bks2);

    VrpSolution initVrpSolution(&vrpProblem);
    initVrpSolution.routes.clear();
    initVrpSolution.vehicleNum = bks.vehicleNum;
    int avgRouteNodeNum = (int)vrpProblem.vrpData.pickNodeIds.size() / (int)bks.routes.size() + 1;
    for (int i = 0; i < (int)bks.routes.size(); ++i) {
        initVrpSolution.routes.emplace_back(i);
        RouteNode* prevNode = nullptr;
        for (int j = i * avgRouteNodeNum;
             j < std::min((int)vrpProblem.vrpData.pickNodeIds.size(), (i + 1) * avgRouteNodeNum); ++j) {
            int pickupNodeId = vrpProblem.vrpData.pickNodeIds[j];

            RouteNode* currentNode = &initVrpSolution.routeNodes[pickupNodeId];
            RouteNode* currentCorresRouteNode = currentNode->corresRouteNode;

            initVrpSolution.insertNode(i, pickupNodeId, currentCorresRouteNode->nodeId, prevNode, prevNode, false);

            prevNode = currentCorresRouteNode;
        }

        initVrpSolution.updateRouteData(i, initVrpSolution.routes[i].startNode, initVrpSolution.routes[i].endNode);
    }

    VrpSolution sisrSol(&vrpProblem);
    sisrSol.copySolution(initVrpSolution);
    orderXover ox(2, &vrpProblem.vrpData, &vrpProblem);
    ox.execute(&bks,&bks2, 100.0,1.0);
    std::cout<<"Init solution 1(BKS1)  time window violation: "<<bks.getTotalTwViolation()<<"\n         capacity violation: "<<bks.getTotalCapacityViolation()
            <<"\n         Distance: "<<bks.getTotalDistance()<<"   #Vehicle: "<<bks.vehicleNum<<"\n";
    std::cout<<"Init solution 2(BKS2)  time window violation: "<<bks2.getTotalTwViolation()<<"\n         capacity violation: "<<bks2.getTotalCapacityViolation()
            <<"\n         Distance: "<<bks2.getTotalDistance()<<"   #Vehicle: "<<bks2.vehicleNum<<"\n"; 
    std::cout<<"Init solution 2(RAND)  time window violation: "<<sisrSol.getTotalTwViolation()<<"\n         capacity violation: "<<sisrSol.getTotalCapacityViolation()
            <<"\n         Distance: "<<sisrSol.getTotalDistance()<<"   #Vehicle: "<<sisrSol.vehicleNum<<"\n"; 
}