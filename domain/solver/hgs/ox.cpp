#include "domain/solver/hgs/ox.h"
#include <csetjmp>
#include <cstddef>
#include <unordered_map>
#include <unordered_set>
#include <sstream>


// Crossover func
// 1. Find part in sol1 to fix
// 2. Exclude nodes in fixed part, then add nodes as ordered in sol2
std::vector<int> orderXover::oxCross(const VrpSolution* sol1, const VrpSolution* sol2){
    // Step 1: Find segment in First parent (length: fixRange)
    // Note: need to check feasibility in crossover
    std::vector<int> sol1_vec = retInt(sol1);

    int nNodes = sol1_vec.size();
    int starting = std::rand()%nNodes; 

    if (OUTPUT>0){
        std::cout<<"--------------------------------\n";
        std::cout<<"Xover Step1:::::\n";
    }

    std::vector<int> res;
    std::unordered_set<int> fixedNodes;
    for (int iNode = starting; iNode < starting+fixRange; ++iNode){
        int tempIdx = iNode % nNodes;
        res.emplace_back(sol1_vec[tempIdx]);
        fixedNodes.insert(sol1_vec[tempIdx]);
    }

    if (OUTPUT==2){
        std::cout<<"sol1: ";
        for (int i=0;i<sol1_vec.size();++i){
            std::cout<<sol1_vec[i]<<",";
        }
        std::cout<<"\n\n";
        std::cout<<"***Starting: "<<starting<<"  length: "<<fixRange<<"***\n";
        std::cout<<"Fixed: ";
        for (int i=0;i<res.size();++i){
            std::cout<<res[i]<<",";
        }
        std::cout<<"\n\n";
    }


    //Step 2: Find the resting ordering
    getOrderedPart(sol2, fixedNodes, res, starting);

    if (OUTPUT>0){
        std::cout<<"Xover Step1 Finished!!!!\n";
        std::cout<<"--------------------------------\n\n\n";
    }
    return res;
}

// Crossover Utilities

// getFixedPart takes the second solution in crossOver, fixed nodes 
//      and return the indicies of nodes
void orderXover::getOrderedPart(const VrpSolution* sol2, const std::unordered_set<int> fixedNodes, std::vector<int>& res, int starting){
    std::vector<int> sol2_vec = retInt(sol2);


    if (OUTPUT==2){
        std::cout<<"sol2: ";
        for (int i=0;i<sol2_vec.size();++i){
            std::cout<<sol2_vec[i]<<",";
        }
        std::cout<<"\n\n";
    }

    int nNodes = sol2_vec.size();

    // Starting insert nodes
    for (int iNode = starting+fixRange; iNode<starting+nNodes+fixRange; ++iNode){
        int tempIdx = iNode % nNodes;
        // std::cout<<"Checking: "<<sol2_vec[tempIdx]<<" at pos "<<tempIdx<<"("<<iNode<<")"<<"/"<<sol2_vec.size()<<"\n";
        if (fixedNodes.find(sol2_vec[tempIdx]) == fixedNodes.end()){
            res.emplace_back(sol2_vec[tempIdx]);
        }
    }


    if (OUTPUT==2){
        std::cout<<"Done Initial Ordering: ";
        for (int i=0;i<res.size();++i){
            std::cout<<res[i]<<",";
        }
        std::cout<<"\n\n";
    }


    //  Prepare for feas check, only store delivery node,  first: id, second: position
    std::unordered_map<int,int> visited_pos;

    // Check for feasibility
    for (int i=0;i<res.size();++i){
        int idx = res[i];
        if (pickDeliPair.find(idx)!=pickDeliPair.end()){
            // this is a pickup node, have to check if not delivered yet
            int target = pickDeliPair[idx];
            if (visited_pos.find(target)!=visited_pos.end()){
                if (OUTPUT==2){
                    std::cout<<"Found conflict:\n    Delivered at "<<target<<" before pick up at "<<idx<<"\n";
                }
                // delivered, have to switch
                int deliPos = visited_pos[target];
                visited_pos[target] = i;
                res[deliPos] = idx;
                res[i] = target;
            }
        }else if (idx!=0){
            visited_pos[idx] = i;
        }
    }


    if (OUTPUT==2){
        std::cout<<"After feasibility restoration: ";
        for (int i=0;i<res.size();++i){
            std::cout<<res[i]<<",";
        }
        std::cout<<"\n\n";
    }
}

// retInt takes in an arbitrary VrpSolution and return vector<int> of routeNodes
std::vector<int> orderXover::retInt(const VrpSolution* sol){
    std::vector<int> res;
    for (auto Route=sol->routes.cbegin();Route<sol->routes.cend();++Route){
        int endPoint = Route->endNode->nodeId;
        RouteNode* startNode = Route->startNode;
        res.emplace_back(startNode->nodeId);
        while (startNode->nodeId!=endPoint){
            startNode = startNode->nexNode;
            res.emplace_back(startNode->nodeId);
        }
    }
    return res;
}


//Shortest path decomposition method
void orderXover::decompose(const std::vector<int>& order, const VrpSolution* sol, double cPen, double tPen){
    //Step 1: construct the adjacency list with computed distance
    if (OUTPUT==2){
        std::cout<<"--------------------------------\n";
        std::cout<<"Xover Step2:::::\n";
        std::cout<<"Using ordering: ";
        for (int i=0;i<order.size();++i){
            std::cout<<order[i]<<",";
        }
        std::cout<<"\n\n";
    }
    if (OUTPUT==1){
        std::cout<<"--------------------------------\n";
        std::cout<<"Xover Step2:::::\n";
    }

    int nNode=order.size();
    std::vector<std::vector<std::pair<float,int>>> dMat(nNode+1);
    getAdj(order, dMat,sol,sol->vrpProblem->vrpData.nodeList,cPen,tPen, vp->vrpData.vehicleCapacity);
    if (OUTPUT==2){
        std::cout<<"Distance Matrix : \n";
        for (int i=0;i<dMat.size();++i){
            std::cout<<i<<": ";
            for (int j=0;j<dMat[i].size();++j){
                std::cout<<dMat[i][j].second<<":"<<dMat[i][j].first<<" , ";
            }
            std::cout<<"\n";
        }
        std::cout<<"\n\n****Shortest Path Started****\n";
    }
    if (OUTPUT==1) std::cout<<"\n\n****Shortest Path Started****\n";

    std::vector<int> vertices = spDA(dMat);

    if (OUTPUT==1){
        std::cout<<"NOTICE:::::Shortest Path algorithm finished\n";
        std::cout<<"vertices on path: ";
        for (int i=0;i<vertices.size();++i){
            std::cout<<vertices[i]<<",";
        }
        std::cout<<"\n****Shortest Path Ended****\n\n";
    }
    if (OUTPUT==1) std::cout<<"NOTICE:::::Shortest Path algorithm finished\n";
    //Step 2: split and generate solution
    if (OUTPUT==1){
        int lp=0;
        std::cout<<"Paths : \n";
        for (int i=0;i<vertices.size();++i){
            std::cout<<"Path "<<i<<": ";
            for (int j=lp;j<=vertices[i];++j){
                std::cout<<order[j]<<", ";
                lp=vertices[i]+1;
            }
            std::cout<<"\n";
        }
        std::cout<<"\n\n****Shortest Path Started****\n";
    }
    constructSol(vertices,order);

    if (OUTPUT>0){
        std::cout<<"Xover Step2 Finished!!!!\n";
        std::cout<<"--------------------------------\n\n\n";
    }
}


VrpSolution orderXover::constructSol(const std::vector<int> & vertices, const std::vector<int> & orderedNodes){
    int nVehicle = vertices.size();
    if (OUTPUT>0){
        std::cout<<"Constructing solution with "<<nVehicle<<" vehicle(s)\n";
    }
    VrpSolution vs(vp);
    vs.routes.clear();
    vs.vehicleNum = nVehicle;
    RouteNode* prevNode = nullptr;
    int lastPos=0;
    // Insert vehicle/routes one by one
    for (int iVe=0;iVe<nVehicle;++iVe){
        vs.routes.emplace_back(iVe);
        // Insert nodes into the route
        if (OUTPUT==1) std::cout<<"Inserting ROUTE: "<<iVe<<"\n";
        for (int iNodez = lastPos; iNodez<vertices[iVe]; ++iNodez){
            int iNode = orderedNodes[iNodez];
            if (vp->vrpData.nodeList[iNode].nodeType==Utils::VrpNodeType::kPickup){
                if (OUTPUT==2) std::cout<<"!!!Inserting node: "<<iNode<<" into Route "<<iVe<<"\n";
                RouteNode* currentNode = &vs.routeNodes[iNode];
                RouteNode* currentCorresRouteNode = currentNode->corresRouteNode;
                vs.insertNode(iVe, iNode, currentCorresRouteNode->nodeId, prevNode, prevNode, false);
                prevNode = currentCorresRouteNode;
            }
        }
        lastPos=vertices[iVe]+1;
        vs.updateRouteData(iVe, vs.routes[iVe].startNode, vs.routes[iVe].endNode);
    }
    if (OUTPUT>0){
        double distance = vs.getTotalDistance();
        std::cout<<"Solution constructed with "<<nVehicle<<" vehicle(s), total distance:"<<distance<<"\n";
        std::cout<<"    total time window violation: "<<vs.getTotalTwViolation()<<"\n";
        std::cout<<"    total capacity violation: "<<vs.getTotalCapacityViolation()<<"\n";
        // for (int i=0;i<vs.routes.size();++i){
        //     std::cout<<"Route "<<i<<":\n";
        //     // RouteNode * tn = vs.routes[i].startNode;
        //     // while (true){
        //     //     std::cout<<tn->nodeId<<",";
        //     //     tn=tn->nexNode;
        //     // }
        //     // std::cout<<vs.routes[i].nodeIds.size()<<"\n";
        //     // for (int j=0;j<vs.routes[i].nodeIds.size(); ++j){
        //     //     std::cout<<vs.routes[i].nodeIds[j]<<",";
        //     // }
        // }
        // std::cout<<"\n\n";
    }
    
    return vs;
}

std::vector<int> orderXover::spDA(const std::vector<std::vector<std::pair<float,int>>>& adj){
    int nNode=adj.size();
    std::vector<int> prevPos(nNode,-1);
    std::vector<float> curDis(nNode,1e+20);
    std::vector<int> load(nNode,0);
    std::vector<int> lastTime(nNode,0);
    // note: 0-2 is 0-1-2
    // Penalty1: capacity       TODO::
    // Penalty2: time window    TODO::
    std::priority_queue<std::pair<float,int>> queue;
    std::unordered_set<int> visited;
    queue.emplace(0,0);
    curDis[0] = 0;
    while (queue.size()!=0){
        std::pair<float,int> top = queue.top();
        queue.pop();
        visited.insert(top.second);
        if (OUTPUT==2)    std::cout<<" @@ Dequeued: "<<top.second<<"  distance: "<<top.first<<"  Current load: "<<load[top.second]<<"\n";
        float dist = top.first;
        int iNode = top.second;
        for (int iSuc=0; iSuc<adj[iNode].size(); ++iSuc){
            int tarIdx = adj[iNode][iSuc].second;

            // double ifCapacity = load[top.second];
            // if (vp->vrpData.nodeList[tarIdx].nodeType==Utils::VrpNodeType::kPickup) ifCapacity+=vp->vrpData.nodeList[tarIdx].demand;
            // else if (vp->vrpData.nodeList[tarIdx].nodeType==Utils::VrpNodeType::kDelivery) ifCapacity-=vp->vrpData.nodeList[tarIdx].demand;
            // double capPenalty = 0;
            // if (ifCapacity>capacity){
            //     capPenalty = (ifCapacity - capacity) * capPen;
            // }

            // double ifTime;
            // double startTime = vp->vrpData.nodeList[tarIdx].timeWindow.first;
            // double endTime = vp->vrpData.nodeList[tarIdx].timeWindow.second;
            // if (lastTime[iNode] < startTime){
            //     ifTime = startTime;
            // }else{
            //     ifTime = lastTime[iNode];
            // }
            // ifTime += vp->vrpData.nodeList[tarIdx].serveTime;
            // double timePenalty = 0;
            // if (ifTime>endTime){
            //     timePenalty = timePen * (ifTime-endTime);
            // }

            // Final distance
            // float ifDist = dist + adj[iNode][iSuc].first + capPenalty + timePenalty;
            float ifDist = dist + adj[iNode][iSuc].first;

            if (OUTPUT==2) std::cout<<" $$ Checking to : "<<tarIdx<<"  current distance: "<<curDis[tarIdx]<<"  new distance: "<<ifDist<<"\n";
                            // <<"   Time Penalty: "<<timePenalty<<"   capacity Penalty: "<<capPenalty<<"\n";
            if (ifDist<curDis[tarIdx]){
                curDis[tarIdx] = ifDist;
                prevPos[tarIdx] = iNode;
                // load[tarIdx] = ifCapacity;
                // lastTime[tarIdx] = ifTime;
            }
            if (visited.find(tarIdx)==visited.end()){
                queue.emplace(curDis[tarIdx],tarIdx);
            }
        }
    }
    std::vector<int> vertices;
    int currentVert = nNode-1; 
    while (currentVert != 0){
        vertices.insert(vertices.begin(),currentVert);
        currentVert = prevPos[currentVert];
    }
    return vertices;
}



void orderXover::getAdj(const std::vector<int>& order, std::vector<std::vector<std::pair<float,int>>>& dMat, const VrpSolution* sol, const std::vector<VrpNode> & nodeList, 
                            double cPen, double tPen, double maxCap){
    int nNode=nodeList.size();
    std::vector<std::unordered_map<int,float>> adj(nNode);
    // TODO:: check edge confilicting
    for (int j=1;j<nNode;++j){
        double dist = GetEuclideanDis(nodeList[0].lon,nodeList[0].lat,nodeList[j].lon,nodeList[j].lat);
        adj[0].emplace(j,dist);
    }
    for (int i=1;i<nNode;++i){
        for (int j=i+1;j<nNode;++j){
            // double dist = (nodeList[i].lon-nodeList[j].lon)*(nodeList[i].lon-nodeList[j].lon) +
            //                 (nodeList[i].lat-nodeList[j].lat)*(nodeList[i].lat-nodeList[j].lat);
            double dist = GetEuclideanDis(nodeList[i].lon,nodeList[i].lat,nodeList[j].lon,nodeList[j].lat);
            // std::cout<<"Computing distance from: ("<<nodeList[i].lon<<","<<nodeList[i].lat<<") to ("
            //             <<nodeList[j].lon<<","<<nodeList[j].lat<<")  distance: "<<dist<<"\n";
            adj[i].emplace(j,dist);
        }
    }
    if (OUTPUT==2){
        std::cout<<"Adjacency list: \n";
        for (int i=0;i<adj.size();++i){
            std::cout<<i<<": ";
            for (auto x : adj[i]){
                std::cout<<x.first<<":"<<x.second<<" , ";
            }
            std::cout<<"\n";
        }
    }
    for (int iz=0;iz<nNode;++iz){
        // std::cout<<"ZZZZZZZZZ"<<"\n";
        int i = order[iz];
        double summer = 0.0;

        double timePen = 0;
        double capPen = 0;

        double lastTime = 0;
        if (i>0) vp->vrpData.nodeList[i].timeWindow.first + vp->vrpData.nodeList[i].serveTime;  //???
        double currentCap = 0;
        for (int jz=iz+1;jz<nNode;++jz){
            int j = order[jz-1];
            // std::cout<<"cCCCCCC "<<j<<","<<i<<"  "<<jz<<"/"<<order.size()<<"    "<<adj[0].size()<<"\n";
            double dist = adj[0][i+1] + summer + adj[0][j];

            double startTime = vp->vrpData.nodeList[j].timeWindow.first;
            double endTime = vp->vrpData.nodeList[j].timeWindow.second;
            double serveTime = vp->vrpData.nodeList[j].serveTime;
            double travelTime = adj[i][j];
            double cap = vp->vrpData.nodeList[j].demand;
            // std::cout<<"XXXXXXXXXXx "<<travelTime<<"\n";

            // TODO:: add traveling time
            if (lastTime + serveTime + travelTime > endTime) {
                timePen += tPen * (lastTime + serveTime + travelTime - endTime);
                lastTime += serveTime + travelTime;
            }else if (lastTime + serveTime + travelTime < startTime){
                lastTime = startTime;
            }else{
                lastTime += serveTime + travelTime;
            }

            if (vp->vrpData.nodeList[j].nodeType==Utils::VrpNodeType::kPickup){
                currentCap += cap;
            }else
            if (vp->vrpData.nodeList[j].nodeType==Utils::VrpNodeType::kDelivery){
                currentCap -= cap;
            }
            if (currentCap> maxCap) {
                capPen = cPen * (currentCap - maxCap);
            }

            dMat[iz].emplace_back(dist+timePen+capPen,jz);
            // dMat[i].emplace_back(dist,j);
            if (j<nNode-1){
                if (OUTPUT==2) std::cout<<"i:"<<i<<" j:"<<j<<"  summer:"<<summer<<"  adj: "<<adj[j][j-i+1]<<"\n"; 
                summer+=adj[j][j+1];
            }

        }
    }
}