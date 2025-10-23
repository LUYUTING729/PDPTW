#include "analysis/problem_analysis.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

#include "config.h"
#include "io/pdptw_reader.h"
#include "model/pdptw.h"
void AnalysisProblem() {
    std::filesystem::path instance_folder = AMDAHL_DIR;
    instance_folder /= "data";
    instance_folder /= "instance";

    std::vector<PDPTW> problems;
    for (const auto& file : std::filesystem::directory_iterator(instance_folder)) {
        problems.emplace_back();
        ReadPDPTW(file.path().string(), problems.back());
    }

    idx_t largest_task_num = 0;
    std::cout << std::left << std::setw(20) << "Instance name" << std::setw(20) << "Task Num" << std::setw(20)
              << "Vehicle Num" << std::endl;
    for (const PDPTW& problem : problems) {
        std::cout << std::left << std::setw(20) << problem.instance_name << std::setw(20) << problem.task_num
                  << std::setw(20) << problem.vehicle_num << std::endl;
        largest_task_num = std::max(largest_task_num, problem.task_num);
    }

    std::cout << "Largest task num: " << largest_task_num << std::endl;
}