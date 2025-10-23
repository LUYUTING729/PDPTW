#ifndef AMDAHL_SRC_IO_PDPTW_READER_H_
#define AMDAHL_SRC_IO_PDPTW_READER_H_

#include <string>

#include "model/pdptw.h"

struct TaskData {
    idx_t task_index{-1};
    pos_t pos_x{0};
    pos_t pos_y{0};
    demand_t demand{0};
    double tw_earliest{0.0};
    double tw_latest{0.0};
    double service_time{0.0};
    idx_t pickup_sibling_index{-1};
    idx_t delivery_sibling_index{-1};
};

void ParseVehicleData(char* line_data, int line_data_len, int& vehicle_num, demand_t& vehicle_capacity);

void ParseTaskData(char* line_data, int line_data_len, idx_t& task_index, pos_t& pos_x, pos_t& pos_y, demand_t& demand,
                   double& time_window_earliest, double& time_window_latest, double& service_time,
                   idx_t& pickup_sibling_index, idx_t& delivery_sibling_index);

bool ReadPDPTW(const std::string& problem_file_path, PDPTW& problem);

#endif