#include "domain/solver/memetic/LocalSearch.h"

#include <cassert>

#include "domain/solver/memetic/insert/Insert.h"
#include "domain/solver/memetic/removal/Removal.h"
#include "util/random_utils.h"

bool outRelocateMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, int pickupNodeId, int deliveryNodeId) {
    assert(pickupNodeId > 0);
    assert(deliveryNodeId > 0);
    assert(pickupNodeId != deliveryNodeId);
    assert(vrpSolution.routeNodes[pickupNodeId].corresRouteNode->nodeId == deliveryNodeId);
    assert(vrpSolution.routeNodes[deliveryNodeId].corresRouteNode->nodeId == pickupNodeId);

    RouteNode* pickupNode = &vrpSolution.routeNodes[pickupNodeId];
    RouteNode* deliveryNode = &vrpSolution.routeNodes[deliveryNodeId];
    assert(pickupNode->corresRouteNode == deliveryNode);
    assert(deliveryNode->corresRouteNode == pickupNode);
    assert(pickupNode->routeId == deliveryNode->routeId);

    int currentRouteId = pickupNode->routeId;

    double removalCost = calculateRemovalCost(vrpConfig, vrpSolution, currentRouteId, pickupNode, deliveryNode);

    int minInsertCostRouteId = -1;
    double minInsertCost = std::numeric_limits<double>::max();
    std::vector<InsertionData> insertions;
    RouteNode* pickupInsertPrevNode = nullptr;
    RouteNode* deliveryInsertPrevNode = nullptr;
    for (const VrpRoute& route : vrpSolution.routes) {
        if (route.routeId == currentRouteId) {
            continue;
        }

        insertions.clear();
        calInsertionData(vrpSolution, pickupNode, deliveryNode, route.routeId, insertions, true);
        for (const InsertionData& insertion : insertions) {
            double tmpCost = insertion.deltaDistance;
            tmpCost += vrpConfig.twPenaltyCoeff * insertion.deltaTwPenalty;
            tmpCost += vrpConfig.capacityPenaltyCoeff * insertion.deltaCapacityViolations;

            if (tmpCost < minInsertCost) {
                minInsertCost = tmpCost;
                pickupInsertPrevNode = insertion.pickupPrevNode;
                deliveryInsertPrevNode = insertion.deliveryPrevNode;
                minInsertCostRouteId = route.routeId;
            }
        }
    }

    if (pickupInsertPrevNode == nullptr || deliveryInsertPrevNode == nullptr) {
        return false;
    }

    if (minInsertCost >= removalCost) {
        return false;
    }

    vrpSolution.removeNode(currentRouteId, pickupNodeId, deliveryNodeId, true);
    vrpSolution.insertNode(minInsertCostRouteId, pickupNodeId, deliveryNodeId, pickupInsertPrevNode,
                           deliveryInsertPrevNode, true);

    return true;
}

bool outRelocateMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) {
    std::vector<int> assignedPickupNodeIds;
    assignedPickupNodeIds.reserve(vrpSolution.vrpProblem->vrpData.nodeNum / 2);

    for (const VrpRoute& route : vrpSolution.routes) {
        if (route.nodeNum <= 0) {
            continue;
        }

        RouteNode* currentNode = route.startNode;
        while (nullptr != currentNode) {
            if (Utils::VrpNodeType::kPickup == vrpSolution.vrpProblem->vrpData.nodeList[currentNode->nodeId].nodeType) {
                assignedPickupNodeIds.push_back(currentNode->nodeId);
            }
            currentNode = currentNode->nexNode;
        }
    }

    while (!assignedPickupNodeIds.empty()) {
        int randIndex = GetRandInt(0, (int)assignedPickupNodeIds.size() - 1);

        int selectedAssignedPickupNodeId = assignedPickupNodeIds[randIndex];
        int selectedAssignedDeliveryNodeId =
            vrpSolution.routeNodes[selectedAssignedPickupNodeId].corresRouteNode->nodeId;

        if (vrpSolution.routes[vrpSolution.routeNodes[selectedAssignedPickupNodeId].routeId].nodeNum <= 2) {
            assignedPickupNodeIds[randIndex] = assignedPickupNodeIds.back();
            assignedPickupNodeIds.pop_back();
            continue;
        }

        bool tmpRes =
            outRelocateMove(vrpConfig, vrpSolution, selectedAssignedPickupNodeId, selectedAssignedDeliveryNodeId);

        if (tmpRes) {
            return true;
        } else {
            assignedPickupNodeIds[randIndex] = assignedPickupNodeIds.back();
            assignedPickupNodeIds.pop_back();
        }
    }

    return false;
}

bool outExchangeTwoRequests(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, int firstPickupNodeId,
                            int firstDeliveryNodeId, int secondPickupNodeId, int secondDeliveryNodeId) {
    assert(firstPickupNodeId != secondPickupNodeId);
    assert(firstDeliveryNodeId != secondDeliveryNodeId);
    assert(vrpSolution.routeNodes[firstPickupNodeId].routeId == vrpSolution.routeNodes[firstDeliveryNodeId].routeId);
    assert(vrpSolution.routeNodes[secondPickupNodeId].routeId == vrpSolution.routeNodes[secondDeliveryNodeId].routeId);

    int firstRouteId = vrpSolution.routeNodes[firstPickupNodeId].routeId;
    int secondRouteId = vrpSolution.routeNodes[secondPickupNodeId].routeId;
    assert(firstRouteId != secondRouteId);

    double firstDistance = vrpSolution.routes[firstRouteId].distance;
    double firstTwViolation = vrpSolution.routes[firstRouteId].twViolation;
    double firstCapacityViolation = vrpSolution.routes[firstRouteId].endNode == nullptr
                                        ? 0.0
                                        : vrpSolution.routes[firstRouteId].endNode->accumulatedCapacityViolation;

    double secondDistance = vrpSolution.routes[secondRouteId].distance;
    double secondTwViolation = vrpSolution.routes[secondRouteId].twViolation;
    double secondCapacityViolation = vrpSolution.routes[secondRouteId].endNode == nullptr
                                         ? 0.0
                                         : vrpSolution.routes[secondRouteId].endNode->accumulatedCapacityViolation;

    vrpSolution.removeNode(firstRouteId, firstPickupNodeId, firstDeliveryNodeId, true);
    vrpSolution.removeNode(secondRouteId, secondPickupNodeId, secondDeliveryNodeId, true);

    double savingCostAfterRemoval = (firstDistance - vrpSolution.routes[firstRouteId].distance) +
                                    (secondDistance - vrpSolution.routes[secondRouteId].distance);
    savingCostAfterRemoval +=
        vrpConfig.twPenaltyCoeff * (firstTwViolation - vrpSolution.routes[firstRouteId].twViolation);
    savingCostAfterRemoval +=
        vrpConfig.twPenaltyCoeff * (secondTwViolation - vrpSolution.routes[secondRouteId].twViolation);
    savingCostAfterRemoval +=
        vrpConfig.capacityPenaltyCoeff *
        (firstCapacityViolation - (vrpSolution.routes[firstRouteId].endNode == nullptr
                                       ? 0.0
                                       : vrpSolution.routes[firstRouteId].endNode->accumulatedCapacityViolation));
    savingCostAfterRemoval +=
        vrpConfig.capacityPenaltyCoeff *
        (secondCapacityViolation - (vrpSolution.routes[secondRouteId].endNode == nullptr
                                        ? 0.0
                                        : vrpSolution.routes[secondRouteId].endNode->accumulatedCapacityViolation));

    std::vector<InsertionData> firstInsertions;
    calInsertionData(vrpSolution, &vrpSolution.routeNodes[firstPickupNodeId],
                     &vrpSolution.routeNodes[firstDeliveryNodeId], secondRouteId, firstInsertions, true);

    const InsertionData* firstBestInsertionData = nullptr;
    double firstMinInsertCost = std::numeric_limits<double>::max();
    for (const InsertionData& insertion : firstInsertions) {
        double tmpCost = insertion.deltaDistance;
        tmpCost += vrpConfig.twPenaltyCoeff * insertion.deltaTwPenalty;
        tmpCost += vrpConfig.capacityPenaltyCoeff * insertion.deltaCapacityViolations;

        if (tmpCost < firstMinInsertCost) {
            firstMinInsertCost = tmpCost;
            firstBestInsertionData = &insertion;
        }
    }

    std::vector<InsertionData> secondInsertions;
    calInsertionData(vrpSolution, &vrpSolution.routeNodes[secondPickupNodeId],
                     &vrpSolution.routeNodes[secondDeliveryNodeId], firstRouteId, secondInsertions, true);

    const InsertionData* secondBestInsertionData = nullptr;
    double secondMinInsertCost = std::numeric_limits<double>::max();
    for (const InsertionData& insertion : secondInsertions) {
        double tmpCost = insertion.deltaDistance;
        tmpCost += vrpConfig.twPenaltyCoeff * insertion.deltaTwPenalty;
        tmpCost += vrpConfig.capacityPenaltyCoeff * insertion.deltaCapacityViolations;

        if (tmpCost < secondMinInsertCost) {
            secondMinInsertCost = tmpCost;
            secondBestInsertionData = &insertion;
        }
    }

    if (firstBestInsertionData == nullptr || secondBestInsertionData == nullptr) {
        return false;
    }

    if (firstMinInsertCost + secondMinInsertCost >= savingCostAfterRemoval) {
        return false;
    }

    assert(firstBestInsertionData->routeId == secondRouteId);
    assert(secondBestInsertionData->routeId == firstRouteId);

    vrpSolution.insertNode(firstBestInsertionData->routeId, firstPickupNodeId, firstDeliveryNodeId,
                           firstBestInsertionData->pickupPrevNode, firstBestInsertionData->deliveryPrevNode, true);
    vrpSolution.insertNode(secondBestInsertionData->routeId, secondPickupNodeId, secondDeliveryNodeId,
                           secondBestInsertionData->pickupPrevNode, secondBestInsertionData->deliveryPrevNode, true);

    return true;
}

bool outExchangeMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution, int pickupNodeId, int deliveryNodeId) {
    assert(pickupNodeId > 0);
    assert(deliveryNodeId > 0);
    assert(pickupNodeId != deliveryNodeId);
    assert(vrpSolution.routeNodes[pickupNodeId].corresRouteNode->nodeId == deliveryNodeId);
    assert(vrpSolution.routeNodes[deliveryNodeId].corresRouteNode->nodeId == pickupNodeId);

    RouteNode* pickupNode = &vrpSolution.routeNodes[pickupNodeId];
    RouteNode* deliveryNode = &vrpSolution.routeNodes[deliveryNodeId];
    assert(pickupNode->corresRouteNode == deliveryNode);
    assert(deliveryNode->corresRouteNode == pickupNode);
    assert(pickupNode->routeId == deliveryNode->routeId);

    int currentRouteId = pickupNode->routeId;

    const std::vector<int>& adjacentList = vrpSolution.vrpProblem->adjacentList[pickupNodeId];

    int exchangeNeighborCounter = 0;
    VrpSolution newSolution(vrpSolution.vrpProblem);
    for (int adjacentNodeId : adjacentList) {
        if (0 == adjacentNodeId || adjacentNodeId == pickupNodeId || adjacentNodeId == deliveryNodeId) {
            continue;
        }
        if (vrpSolution.routeNodes[adjacentNodeId].routeId == currentRouteId) {
            continue;
        }
        if (Utils::VrpNodeType::kDelivery == vrpSolution.vrpProblem->vrpData.nodeList[adjacentNodeId].nodeType) {
            continue;
        }

        newSolution.copySolution(vrpSolution);
        bool tmpRes = outExchangeTwoRequests(vrpConfig, newSolution, pickupNodeId, deliveryNodeId, adjacentNodeId,
                                             newSolution.routeNodes[adjacentNodeId].corresRouteNode->nodeId);

        if (tmpRes) {
            vrpSolution.copySolution(newSolution);
            return true;
        } else {
            exchangeNeighborCounter += 1;
        }

        if (exchangeNeighborCounter > kExchangeNeighborNum) {
            break;
        }
    }

    return false;
}

bool outExchangeMove(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) {
    std::vector<int> assignedPickupNodeIds;
    assignedPickupNodeIds.reserve(vrpSolution.vrpProblem->vrpData.nodeNum / 2);

    for (const VrpRoute& route : vrpSolution.routes) {
        if (route.nodeNum <= 0) {
            continue;
        }

        RouteNode* currentNode = route.startNode;
        while (nullptr != currentNode) {
            if (Utils::VrpNodeType::kPickup == vrpSolution.vrpProblem->vrpData.nodeList[currentNode->nodeId].nodeType) {
                assignedPickupNodeIds.push_back(currentNode->nodeId);
            }
            currentNode = currentNode->nexNode;
        }
    }

    while (!assignedPickupNodeIds.empty()) {
        int randIndex = GetRandInt(0, (int)assignedPickupNodeIds.size() - 1);

        int selectedAssignedPickupNodeId = assignedPickupNodeIds[randIndex];
        int selectedAssignedDeliveryNodeId =
            vrpSolution.routeNodes[selectedAssignedPickupNodeId].corresRouteNode->nodeId;

        bool tmpRes =
            outExchangeMove(vrpConfig, vrpSolution, selectedAssignedPickupNodeId, selectedAssignedDeliveryNodeId);

        if (tmpRes) {
            return true;
        } else {
            assignedPickupNodeIds[randIndex] = assignedPickupNodeIds.back();
            assignedPickupNodeIds.pop_back();
        }
    }

    return false;
}

void localSearch(const VrpConfig& vrpConfig, VrpSolution& vrpSolution) {
    while (true) {
        bool tmpRes = outRelocateMove(vrpConfig, vrpSolution);
        if (tmpRes) {
            continue;
        } else {
            break;
        }
    }

//    while (true) {
//        bool tmpRes = outExchangeMove(vrpConfig, vrpSolution);
//        if (tmpRes) {
//            continue;
//        } else {
//            break;
//        }
//    }
}