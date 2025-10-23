#include "analysis/solution_analysis.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <vector>

#include "config.h"
#include "io/pdptw_reader.h"
#include "io/sol_reader.h"
#include "model/pdptw.h"
#include "model/solution.h"
#include "util/string_utils.h"

void AnalysisSolution() {
    std::filesystem::path instance_folder = AMDAHL_DIR;
    instance_folder /= "data";
    instance_folder /= "instance";

    std::vector<PDPTW> problems;
    for (const auto& file : std::filesystem::directory_iterator(instance_folder)) {
        problems.emplace_back();
        ReadPDPTW(file.path().string(), problems.back());
    }

    std::filesystem::path bks_folder = AMDAHL_DIR;
    bks_folder /= "data";
    bks_folder /= "bks";

    std::cout << std::left << std::setw(16) << "Instance" << std::setw(16) << "Vehicle Num" << std::setw(16)
              << "Total Distance" << std::setw(20) << "TW Violations" << std::setw(20) << "Capacity Violations"
              << std::endl;
    for (const PDPTW& problem : problems) {
        std::string instance_name = ToLower(problem.instance_name);
        for (const auto& file : std::filesystem::directory_iterator(bks_folder)) {
            std::string file_path = file.path().string();
            std::string file_name = file.path().stem().string();
            if (file_name.size() < instance_name.size()) {
                continue;
            }

            if (0 != file_name.compare(0, instance_name.size(), instance_name, 0, instance_name.size())) {
                continue;
            }

            if (file_name.size() > instance_name.size()) {
                char tmp_char = file_name[instance_name.size()];
                if ('0' <= tmp_char && '9' >= tmp_char) {
                    continue;
                }
            }

            Solution solution(problem);
            ReadSol(file_path, solution);

            std::cout.precision(2);
            std::cout.flags(std::ostream::fixed);
            std::cout << std::left << std::setw(16) << instance_name << std::setw(16) << solution.routes_.size()
                      << std::setw(16) << solution.total_distance_ << std::setw(20) << solution.total_tw_violations_
                      << std::setw(20) << solution.total_capacity_violations_ << std::endl;
        }
    }
}