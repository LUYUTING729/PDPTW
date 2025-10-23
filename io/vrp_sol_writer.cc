#include "io/vrp_sol_writer.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>

#include "io/sol_reader.h"
#include "io/sol_writer.h"
#include "util/date_utils.h"

bool WriteSol(const VrpSolution& solution, const std::string& sol_file_path) {
    std::filesystem::path path = sol_file_path;
    if (std::filesystem::is_directory(path)) {
        std::cout << "Sol file path " << sol_file_path << "is invalid" << std::endl;
        return false;
    }

    std::filesystem::path folder = path.parent_path();
    std::filesystem::create_directories(folder);

    std::ofstream ofs;
    ofs.open(sol_file_path, std::ios::out);

    if (!ofs.is_open()) {
        std::cout << "Failed to open " << sol_file_path << std::endl;
        ofs.close();
        return false;
    }

    ofs << std::left << std::setw(kTagWidth) << kInstanceNameTag << ": " << solution.vrpProblem->vrpData.name
        << std::endl;
    ofs << std::left << std::setw(kTagWidth) << kAuthorsTag << ": " << kAuthors << std::endl;
    ofs << std::left << std::setw(kTagWidth) << kDateTag << ": " << GetCurrentDateStr() << std::endl;
    ofs << std::left << std::setw(kTagWidth) << kReferenceTag << ": " << kReference << std::endl;
    ofs << kSolutionTag << std::endl;

    std::vector<int> route_written_seq;
    // 只包含有效的路由（startNode不为空）
    for (int i = 0; i < (int)solution.routes.size(); ++i) {
        if (solution.routes[i].startNode != nullptr) {
            route_written_seq.push_back(i);
        }
    }
    
    std::sort(route_written_seq.begin(), route_written_seq.end(), [&solution](int a, int b) {
        // 添加空指针检查
        if (solution.routes[a].startNode == nullptr || solution.routes[b].startNode == nullptr) {
            return a < b;  // 如果有空指针，按索引排序
        }
        int first_node_id_in_route_a = solution.routes[a].startNode->nodeId;
        int first_node_id_in_route_b = solution.routes[b].startNode->nodeId;
        return first_node_id_in_route_a < first_node_id_in_route_b;
    });

    int route_index = 1;
    for (int route_seq : route_written_seq) {
        const VrpRoute& route = solution.routes[route_seq];
        
        // 再次检查startNode是否为空
        if (route.startNode == nullptr) {
            std::cerr << "Warning: Skipping route " << route_seq << " with null startNode" << std::endl;
            continue;
        }
        
        ofs << kRouteTag << " " << std::to_string(route_index) << " :";

        RouteNode* currentNode = route.startNode;
        while (nullptr != currentNode) {
            ofs << " " << std::to_string(currentNode->nodeId);
            currentNode = currentNode->nexNode;
        }
        ofs << std::endl;
        route_index += 1;
    }

    ofs.close();
    return true;
}