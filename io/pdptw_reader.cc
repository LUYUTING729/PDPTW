#include "io/pdptw_reader.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <unordered_set>

#include "util/distance_utils.h"
#include "util/string_utils.h"

void ParseVehicleData(char* line_data, int line_data_len, int& vehicle_num, demand_t& vehicle_capacity) {
    int start_pos = 0;
    int end_pos = 0;

    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char char_bak = *(line_data + end_pos);
    char* str_end_ptr = nullptr;

    if (kNullChar == char_bak) {
        vehicle_num = (int)strtol(line_data + start_pos, &str_end_ptr, 10);
    } else {
        *(line_data + end_pos) = kNullChar;
        vehicle_num = (int)strtol(line_data + start_pos, &str_end_ptr, 10);
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    if (char_bak == kNullChar) {
        vehicle_capacity = static_cast<demand_t>(strtod(line_data + start_pos, &str_end_ptr));
    } else {
        *(line_data + end_pos) = kNullChar;
        vehicle_capacity = static_cast<demand_t>(strtod(line_data + start_pos, &str_end_ptr));
        *(line_data + end_pos) = char_bak;
    }
}

void ParseTaskData(char* line_data, int line_data_len, idx_t& task_index, pos_t& pos_x, pos_t& pos_y, demand_t& demand,
                   double& time_window_earliest, double& time_window_latest, double& service_time,
                   idx_t& pickup_sibling_index, idx_t& delivery_sibling_index) {
    int start_pos = 0;
    int end_pos = 0;

    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char char_bak = *(line_data + end_pos);
    char* str_end_ptr = nullptr;

    if (kNullChar == char_bak) {
        task_index = (idx_t)strtol(line_data + start_pos, &str_end_ptr, 10);
    } else {
        *(line_data + end_pos) = kNullChar;
        task_index = (idx_t)strtol(line_data + start_pos, &str_end_ptr, 10);
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        pos_x = static_cast<pos_t>(strtod(line_data + start_pos, &str_end_ptr));
    } else {
        *(line_data + end_pos) = kNullChar;
        pos_x = static_cast<pos_t>(strtod(line_data + start_pos, &str_end_ptr));
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        pos_y = static_cast<pos_t>(strtod(line_data + start_pos, &str_end_ptr));
    } else {
        *(line_data + end_pos) = kNullChar;
        pos_y = static_cast<pos_t>(strtod(line_data + start_pos, &str_end_ptr));
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        demand = static_cast<demand_t>(strtod(line_data + start_pos, &str_end_ptr));
    } else {
        *(line_data + end_pos) = kNullChar;
        demand = static_cast<demand_t>(strtod(line_data + start_pos, &str_end_ptr));
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        time_window_earliest = strtod(line_data + start_pos, &str_end_ptr);
    } else {
        *(line_data + end_pos) = kNullChar;
        time_window_earliest = strtod(line_data + start_pos, &str_end_ptr);
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        time_window_latest = strtod(line_data + start_pos, &str_end_ptr);
    } else {
        *(line_data + end_pos) = kNullChar;
        time_window_latest = strtod(line_data + start_pos, &str_end_ptr);
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        service_time = strtod(line_data + start_pos, &str_end_ptr);
    } else {
        *(line_data + end_pos) = kNullChar;
        service_time = strtod(line_data + start_pos, &str_end_ptr);
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        pickup_sibling_index = (idx_t)strtol(line_data + start_pos, &str_end_ptr, 10);
    } else {
        *(line_data + end_pos) = kNullChar;
        pickup_sibling_index = (idx_t)strtol(line_data + start_pos, &str_end_ptr, 10);
        *(line_data + end_pos) = char_bak;
    }

    start_pos = end_pos;
    GetNextEntryStartEnd(line_data, line_data_len, start_pos, end_pos);

    char_bak = *(line_data + end_pos);
    if (kNullChar == char_bak) {
        delivery_sibling_index = (idx_t)strtol(line_data + start_pos, &str_end_ptr, 10);
    } else {
        *(line_data + end_pos) = kNullChar;
        delivery_sibling_index = (idx_t)strtol(line_data + start_pos, &str_end_ptr, 10);
        *(line_data + end_pos) = char_bak;
    }
}

bool ReadPDPTW(const std::string& problem_file_path, PDPTW& problem) {
    std::ifstream ifs;
    ifs.open(problem_file_path, std::ios::in);

    if (!ifs.is_open()) {
        std::cout << "Failed to open " << problem_file_path << std::endl;
        ifs.close();
        return false;
    }

    std::filesystem::path path = problem_file_path;
    problem.instance_name = path.stem().string();

    std::string line_data;
    bool first_line = true;

    idx_t depot_task_index;
    std::unordered_map<idx_t, TaskData> task_data_map;
    while (getline(ifs, line_data)) {
        line_data.erase(line_data.find_last_not_of(kWhiteSpaceChars) + 1);
        if (first_line) {
            ParseVehicleData(&line_data[0], (int)line_data.length(), problem.vehicle_num, problem.vehicle_capacity);
            first_line = false;
        } else {
            TaskData tmp_task_data;
            ParseTaskData(&line_data[0], (int)line_data.length(), tmp_task_data.task_index, tmp_task_data.pos_x,
                          tmp_task_data.pos_y, tmp_task_data.demand, tmp_task_data.tw_earliest, tmp_task_data.tw_latest,
                          tmp_task_data.service_time, tmp_task_data.pickup_sibling_index,
                          tmp_task_data.delivery_sibling_index);

            if (0 == tmp_task_data.pickup_sibling_index && 0 == tmp_task_data.delivery_sibling_index) {
                depot_task_index = tmp_task_data.task_index;
            }
            task_data_map[tmp_task_data.task_index] = tmp_task_data;
        }
    }

    problem.task_num = (int)task_data_map.size();
    problem.new_original_task_index_map.resize(problem.task_num);
    problem.task_demands.resize(problem.task_num);
    problem.task_positions.resize(problem.task_num * 2);
    problem.task_time_windows.resize(problem.task_num * 2);
    problem.task_service_times.resize(problem.task_num);

    problem.original_new_task_index_map[depot_task_index] = 0;
    problem.new_original_task_index_map[0] = depot_task_index;
    problem.task_positions[0] = task_data_map[depot_task_index].pos_x;
    problem.task_positions[1] = task_data_map[depot_task_index].pos_y;
    problem.task_service_times[0] = task_data_map[depot_task_index].service_time;
    problem.task_time_windows[0] = task_data_map[depot_task_index].tw_earliest;
    problem.task_time_windows[1] = task_data_map[depot_task_index].tw_latest;

    idx_t task_counter = 1;
    for (const auto& [index, task_data] : task_data_map) {
        if (index == depot_task_index) {
            continue;
        }

        idx_t delivery_sibling_index = task_data.delivery_sibling_index;
        if (0 == delivery_sibling_index) {
            continue;
        }

        problem.original_new_task_index_map[index] = task_counter;
        problem.original_new_task_index_map[delivery_sibling_index] = (idx_t)(task_counter + 1);
        problem.new_original_task_index_map[task_counter] = index;
        problem.new_original_task_index_map[task_counter + 1] = delivery_sibling_index;

        problem.task_positions[2 * task_counter] = task_data_map[index].pos_x;
        problem.task_positions[2 * task_counter + 1] = task_data_map[index].pos_y;
        problem.task_demands[task_counter] = task_data_map[index].demand;
        problem.task_service_times[task_counter] = task_data_map[index].service_time;
        problem.task_time_windows[2 * task_counter] = task_data_map[index].tw_earliest;
        problem.task_time_windows[2 * task_counter + 1] = task_data_map[index].tw_latest;

        problem.task_positions[2 * task_counter + 2] = task_data_map[delivery_sibling_index].pos_x;
        problem.task_positions[2 * task_counter + 3] = task_data_map[delivery_sibling_index].pos_y;
        problem.task_demands[task_counter + 1] = task_data_map[delivery_sibling_index].demand;
        problem.task_service_times[task_counter + 1] = task_data_map[delivery_sibling_index].service_time;
        problem.task_time_windows[2 * task_counter + 2] = task_data_map[delivery_sibling_index].tw_earliest;
        problem.task_time_windows[2 * task_counter + 3] = task_data_map[delivery_sibling_index].tw_latest;

        task_counter += 2;
    }

    ifs.close();

    problem.distance_matrix.resize(problem.task_num * problem.task_num);
    for (idx_t i = 0; i < problem.task_num; ++i) {
        idx_t offset = i * problem.task_num;
        pos_t first_x = problem.task_positions[2 * i];
        pos_t first_y = problem.task_positions[2 * i + 1];
        for (idx_t j = 0; j < problem.task_num; ++j) {
            pos_t second_x = problem.task_positions[2 * j];
            pos_t second_y = problem.task_positions[2 * j + 1];
            real_t distance = GetEuclideanDis(first_x, first_y, second_x, second_y);
            problem.distance_matrix[offset + j] = distance;
        }
    }

    std::unordered_set<idx_t> time_window_tightened;
    for (idx_t i = 1; i < problem.task_num; ++i) {
        if (i % 2 == 0) {
            continue;
        }
        real_t earliest_pickup_time = problem.task_time_windows[2 * i];
        real_t latest_pickup_time = problem.task_time_windows[2 * i + 1];
        real_t earliest_delivery_time = problem.task_time_windows[2 * (i + 1)];
        real_t latest_delivery_time = problem.task_time_windows[2 * (i + 1) + 1];

        real_t actual_earliest_delivery_time =
            earliest_pickup_time + problem.task_service_times[i] + GetDistance(problem, i, i + 1);

        if (earliest_delivery_time < actual_earliest_delivery_time) {
            time_window_tightened.insert(i);
            problem.task_time_windows[2 * (i + 1)] = actual_earliest_delivery_time;
        }

        real_t actual_latest_pickup_time =
            latest_delivery_time - problem.task_service_times[i] - GetDistance(problem, i, i + 1);
        if (actual_latest_pickup_time < latest_pickup_time) {
            time_window_tightened.insert(i);
            problem.task_time_windows[2 * i + 1] = actual_latest_pickup_time;
        }
    }
    std::cout << "Time windows tightened: " << time_window_tightened.size() << "/"
              << std::to_string((problem.task_num - 1) / 2) << std::endl;

    problem.feasible_arcs.assign(problem.task_num * problem.task_num, 1);
    problem.feasible_arcs[0] = 0;
    real_t first_earliest_arrival_time;
    real_t first_earliest_start_service_time;
    real_t second_earliest_arrival_time;
    real_t second_earliest_start_service_time;
    real_t third_earliest_arrival_time;
    real_t third_earliest_start_service_time;
    real_t fourth_earliest_arrival_time;
    real_t fourth_earliest_start_service_time;
    for (idx_t i = 1; i < problem.task_num; ++i) {
        idx_t offset = i * problem.task_num;

        if (0 == i % 2) {
            problem.feasible_arcs[i] = 0;
        } else {
            problem.feasible_arcs[offset] = 0;
        }

        for (idx_t j = 1; j < problem.task_num; ++j) {
            if (i == j) {
                problem.feasible_arcs[offset + j] = 0;
                continue;
            }

            if (0 == i % 2 && j == i - 1) {
                problem.feasible_arcs[offset + j] = 0;
                continue;
            }

            if (1 == i % 2) {
                if (problem.task_demands[i] + std::fabs(problem.task_demands[j]) > problem.vehicle_capacity) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
            } else {
                if (0 == j % 2 &&
                    std::abs(problem.task_demands[i]) + std::abs(problem.task_demands[j]) > problem.vehicle_capacity) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
            }

            if (1 == i % 2 && j == i + 1) {
                continue;
            }

            if (1 == i % 2 && 1 == j % 2) {
                first_earliest_start_service_time =
                    std::max(GetEarliestStartTime(problem, i),
                             problem.task_time_windows[0] + problem.task_service_times[0] + GetDistance(problem, 0, i));

                second_earliest_arrival_time =
                    first_earliest_start_service_time + problem.task_service_times[i] + GetDistance(problem, i, j);
                if (second_earliest_arrival_time > GetLatestStartTime(problem, j)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
                second_earliest_start_service_time =
                    std::max(second_earliest_arrival_time, GetEarliestStartTime(problem, j));
                bool feasible_arc = false;

                third_earliest_arrival_time =
                    second_earliest_start_service_time + problem.task_service_times[j] + GetDistance(problem, j, i + 1);
                if (third_earliest_arrival_time <= GetLatestStartTime(problem, i + 1)) {
                    third_earliest_start_service_time =
                        std::max(third_earliest_arrival_time, GetEarliestStartTime(problem, i + 1));
                    fourth_earliest_arrival_time = third_earliest_start_service_time +
                                                   problem.task_service_times[i + 1] +
                                                   GetDistance(problem, i + 1, j + 1);
                    if (fourth_earliest_arrival_time <= GetLatestStartTime(problem, j + 1)) {
                        fourth_earliest_start_service_time =
                            std::max(fourth_earliest_arrival_time, GetEarliestStartTime(problem, j + 1));
                        if (fourth_earliest_start_service_time + problem.task_service_times[j + 1] +
                                GetDistance(problem, j + 1, 0) <=
                            GetLatestStartTime(problem, 0)) {
                            feasible_arc = true;
                        }
                    }
                }

                if (!feasible_arc) {
                    third_earliest_arrival_time = second_earliest_start_service_time + problem.task_service_times[j] +
                                                  GetDistance(problem, j, j + 1);
                    if (third_earliest_arrival_time <= GetLatestStartTime(problem, j + 1)) {
                        third_earliest_start_service_time =
                            std::max(third_earliest_arrival_time, GetEarliestStartTime(problem, j + 1));
                        fourth_earliest_arrival_time = third_earliest_start_service_time +
                                                       problem.task_service_times[j + 1] +
                                                       GetDistance(problem, j + 1, i + 1);
                        if (fourth_earliest_arrival_time <= GetLatestStartTime(problem, i + 1)) {
                            fourth_earliest_start_service_time =
                                std::max(fourth_earliest_arrival_time, GetEarliestStartTime(problem, i + 1));
                            if (fourth_earliest_start_service_time + problem.task_service_times[i + 1] +
                                    GetDistance(problem, i + 1, 0) <=
                                GetLatestStartTime(problem, 0)) {
                                feasible_arc = true;
                            }
                        }
                    }
                }

                if (!feasible_arc) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
            }

            if (1 == i % 2 && 0 == j % 2) {
                first_earliest_arrival_time =
                    GetEarliestStartTime(problem, 0) + problem.task_service_times[0] + GetDistance(problem, 0, j - 1);
                first_earliest_start_service_time =
                    std::max(first_earliest_arrival_time, GetEarliestStartTime(problem, j - 1));
                second_earliest_arrival_time = first_earliest_start_service_time + problem.task_service_times[j - 1] +
                                               GetDistance(problem, j - 1, i);
                if (second_earliest_arrival_time > GetLatestStartTime(problem, i)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }

                second_earliest_start_service_time =
                    std::max(second_earliest_arrival_time, GetEarliestStartTime(problem, i));
                third_earliest_arrival_time =
                    second_earliest_start_service_time + problem.task_service_times[i] + GetDistance(problem, i, j);
                if (third_earliest_arrival_time > GetLatestStartTime(problem, j)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }

                third_earliest_start_service_time =
                    std::max(third_earliest_arrival_time, GetEarliestStartTime(problem, j));
                fourth_earliest_arrival_time =
                    third_earliest_start_service_time + problem.task_service_times[j] + GetDistance(problem, j, i + 1);
                if (fourth_earliest_arrival_time > GetLatestStartTime(problem, i + 1)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }

                fourth_earliest_start_service_time =
                    std::max(fourth_earliest_arrival_time, GetEarliestStartTime(problem, i + 1));
                if (fourth_earliest_start_service_time + problem.task_service_times[i + 1] +
                        GetDistance(problem, i + 1, 0) >
                    GetLatestStartTime(problem, 0)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
            }

            if (0 == i % 2 && 1 == j % 2) {
                first_earliest_arrival_time =
                    GetEarliestStartTime(problem, 0) + problem.task_service_times[0] + GetDistance(problem, 0, i - 1);
                first_earliest_start_service_time =
                    std::max(first_earliest_arrival_time, GetEarliestStartTime(problem, i - 1));
                second_earliest_arrival_time = first_earliest_start_service_time + problem.task_service_times[i - 1] +
                                               GetDistance(problem, i - 1, i);

                second_earliest_start_service_time =
                    std::max(second_earliest_arrival_time, GetEarliestStartTime(problem, i));
                third_earliest_arrival_time =
                    second_earliest_start_service_time + problem.task_service_times[i] + GetDistance(problem, i, j);

                if (third_earliest_arrival_time > GetLatestStartTime(problem, j)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
                third_earliest_start_service_time =
                    std::max(third_earliest_arrival_time, GetEarliestStartTime(problem, j));
                fourth_earliest_arrival_time =
                    third_earliest_start_service_time + problem.task_service_times[j] + GetDistance(problem, j, j + 1);
                if (fourth_earliest_arrival_time > GetLatestStartTime(problem, j + 1)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
                fourth_earliest_start_service_time =
                    std::max(fourth_earliest_arrival_time, GetEarliestStartTime(problem, j + 1));
                if (fourth_earliest_start_service_time + problem.task_service_times[j + 1] +
                        GetDistance(problem, j + 1, 0) >
                    GetLatestStartTime(problem, 0)) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
            }

            if (0 == i % 2 && 0 == j % 2) {
                bool feasible_arc = false;
                first_earliest_arrival_time =
                    GetEarliestStartTime(problem, 0) + problem.task_service_times[0] + GetDistance(problem, 0, i - 1);
                first_earliest_start_service_time =
                    std::max(first_earliest_arrival_time, GetEarliestStartTime(problem, i - 1));
                second_earliest_arrival_time = first_earliest_start_service_time + problem.task_service_times[i - 1] +
                                               GetDistance(problem, i - 1, j - 1);
                if (second_earliest_arrival_time <= GetLatestStartTime(problem, j - 1)) {
                    second_earliest_start_service_time =
                        std::max(second_earliest_arrival_time, GetEarliestStartTime(problem, j - 1));
                    third_earliest_arrival_time = second_earliest_start_service_time +
                                                  problem.task_service_times[j - 1] + GetDistance(problem, j - 1, i);
                    if (third_earliest_arrival_time <= GetLatestStartTime(problem, i)) {
                        third_earliest_start_service_time =
                            std::max(third_earliest_arrival_time, GetEarliestStartTime(problem, i));
                        fourth_earliest_arrival_time = third_earliest_start_service_time +
                                                       problem.task_service_times[i] + GetDistance(problem, i, j);
                        if (fourth_earliest_arrival_time <= GetLatestStartTime(problem, j)) {
                            fourth_earliest_start_service_time =
                                std::max(fourth_earliest_arrival_time, GetEarliestStartTime(problem, j));
                            if (fourth_earliest_start_service_time + problem.task_service_times[j] +
                                    GetDistance(problem, j, 0) <=
                                GetLatestStartTime(problem, 0)) {
                                feasible_arc = true;
                            }
                        }
                    }
                }

                if (!feasible_arc) {
                    first_earliest_arrival_time = GetEarliestStartTime(problem, 0) + problem.task_service_times[0] +
                                                  GetDistance(problem, 0, j - 1);
                    first_earliest_start_service_time =
                        std::max(first_earliest_arrival_time, GetEarliestStartTime(problem, j - 1));
                    second_earliest_arrival_time = first_earliest_start_service_time +
                                                   problem.task_service_times[j - 1] +
                                                   GetDistance(problem, j - 1, i - 1);
                    if (second_earliest_arrival_time <= GetLatestStartTime(problem, i - 1)) {
                        second_earliest_start_service_time =
                            std::max(second_earliest_arrival_time, GetEarliestStartTime(problem, i - 1));
                        third_earliest_arrival_time = second_earliest_start_service_time +
                                                      problem.task_service_times[i - 1] +
                                                      GetDistance(problem, i - 1, i);
                        if (third_earliest_arrival_time <= GetLatestStartTime(problem, i)) {
                            third_earliest_start_service_time =
                                std::max(third_earliest_arrival_time, GetEarliestStartTime(problem, i));
                            fourth_earliest_arrival_time = third_earliest_start_service_time +
                                                           problem.task_service_times[i] + GetDistance(problem, i, j);
                            if (fourth_earliest_arrival_time <= GetLatestStartTime(problem, j)) {
                                fourth_earliest_start_service_time =
                                    std::max(fourth_earliest_arrival_time, GetEarliestStartTime(problem, j));
                                if (fourth_earliest_start_service_time + problem.task_service_times[j] +
                                        GetDistance(problem, j, 0) <=
                                    GetLatestStartTime(problem, 0)) {
                                    feasible_arc = true;
                                }
                            }
                        }
                    }
                }

                if (!feasible_arc) {
                    problem.feasible_arcs[offset + j] = 0;
                    continue;
                }
            }
        }
    }

    problem.adjacency_list.resize(problem.task_num);
    for (idx_t i = 0; i < problem.task_num; ++i) {
        problem.adjacency_list[i].resize(problem.task_num);
        std::iota(problem.adjacency_list[i].begin(), problem.adjacency_list[i].end(), 0);
        int tmp_offset = i * problem.task_num;
        std::sort(problem.adjacency_list[i].begin(), problem.adjacency_list[i].begin() + problem.task_num,
                  [&](idx_t a, idx_t b) {
                      return problem.distance_matrix[tmp_offset + a] < problem.distance_matrix[tmp_offset + b];
                  });
    }
    return true;
}