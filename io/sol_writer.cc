#include "io/sol_writer.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>

#include "io/sol_reader.h"
#include "util/date_utils.h"
#include "util/string_utils.h"

bool WriteSol(const Solution& solution, const std::string& sol_file_path) {
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

    ofs << std::left << std::setw(kTagWidth) << kInstanceNameTag << ": " << solution.problem_.instance_name
        << std::endl;
    ofs << std::left << std::setw(kTagWidth) << kAuthorsTag << ": " << kAuthors << std::endl;
    ofs << std::left << std::setw(kTagWidth) << kDateTag << ": " << GetCurrentDateStr() << std::endl;
    ofs << std::left << std::setw(kTagWidth) << kReferenceTag << ": " << kReference << std::endl;
    ofs << kSolutionTag << std::endl;

    std::vector<int> route_written_seq(solution.routes_.size(), 0);
    std::iota(route_written_seq.begin(), route_written_seq.end(), 0);
    std::sort(route_written_seq.begin(), route_written_seq.end(), [&solution](int a, int b) {
        int first_node_id_in_route_a = solution.problem_.new_original_task_index_map[solution.routes_[a][1]];
        int first_node_id_in_route_b = solution.problem_.new_original_task_index_map[solution.routes_[b][1]];
        return first_node_id_in_route_a < first_node_id_in_route_b;
    });
    int route_index = 1;

    for (const int route_seq : route_written_seq) {
        const std::vector<idx_t>& route = solution.routes_[route_seq];
        ofs << kRouteTag << " " << std::to_string(route_index) << " :";
        for (idx_t task_index : route) {
            if (0 == task_index) {
                continue;
            }

            task_index = solution.problem_.new_original_task_index_map[task_index];
            ofs << " " << std::to_string(task_index);
        }
        ofs << std::endl;
        route_index += 1;
    }

    ofs.close();
    return true;
}