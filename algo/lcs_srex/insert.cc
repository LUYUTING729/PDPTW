#include "algo/lcs_srex/insert.h"

#include <algorithm>

#include "util/random_utils.h"

namespace lcs_srex {

void CalInsertionData(const SREXSol& sol, idx_t pickup_task_index, idx_t delivery_task_index, idx_t route_index,
                      std::vector<InsertionData>& insertions, bool allow_infeasible) {
    const RouteData& route_data = sol.route_data_[route_index];

    if (0 == route_data.task_num) {
        insertions.emplace_back();
        InsertionData& insertion_data = insertions.back();
        insertion_data.route_index = route_index;
        insertion_data.pickup_prev_task_index = 0;
        insertion_data.delivery_prev_task_index = 0;
        insertion_data.delta_tw_penalty = 0.0;
        insertion_data.delta_capacity_violations = 0;
        insertion_data.delta_distance = GetDistance(sol.problem_, 0, pickup_task_index) +
                                        GetDistance(sol.problem_, pickup_task_index, delivery_task_index) +
                                        GetDistance(sol.problem_, delivery_task_index, 0);
        insertion_data.new_vehicle = true;
        return;
    }

    idx_t pickup_prev_task_index = 0;
    while (true) {
        idx_t pickup_prev_next_task_index = 0;
        if (0 == pickup_prev_task_index) {
            pickup_prev_next_task_index = route_data.first_task;
        } else {
            pickup_prev_next_task_index = sol.task_data_[pickup_prev_task_index].next_task_index;
        }

        if (!allow_infeasible && !IsArcFeasible(sol.problem_, pickup_prev_task_index, pickup_task_index)) {
            pickup_prev_task_index = pickup_prev_next_task_index;
            if (0 == pickup_prev_task_index) {
                break;
            } else {
                continue;
            }
        }

        real_t prev_departure_time = GetEarliestStartTime(sol.problem_, 0);
        real_t prev_forward_tw_penalty_slack = 0.0;
        demand_t prev_vehicle_load = 0;
        demand_t prev_accumulated_capacity_violations = 0;

        if (0 != pickup_prev_task_index) {
            prev_departure_time = sol.task_data_[pickup_prev_task_index].departure_time;
            prev_forward_tw_penalty_slack = sol.task_data_[pickup_prev_task_index].forward_tw_penalty_slack;
            prev_vehicle_load = sol.task_data_[pickup_prev_task_index].vehicle_load;
            prev_accumulated_capacity_violations =
                sol.task_data_[pickup_prev_task_index].accumulated_capacity_violations;
        }

        real_t current_arrival_time =
            prev_departure_time + GetDistance(sol.problem_, pickup_prev_task_index, pickup_task_index);
        demand_t current_vehicle_load = prev_vehicle_load + sol.problem_.task_demands[pickup_task_index];

        real_t current_forward_tw_penalty_slack =
            prev_forward_tw_penalty_slack +
            std::max(0.0, current_arrival_time - GetLatestStartTime(sol.problem_, pickup_task_index));
        demand_t current_accumulated_capacity_violations =
            prev_accumulated_capacity_violations + std::max(0, current_vehicle_load - sol.problem_.vehicle_capacity);
        real_t current_start_service_time =
            std::min(std::max(current_arrival_time, GetEarliestStartTime(sol.problem_, pickup_task_index)),
                     GetLatestStartTime(sol.problem_, pickup_task_index));

        if (!allow_infeasible) {
            bool infeasible = current_forward_tw_penalty_slack > 0.0 || current_accumulated_capacity_violations > 0;
            if (!infeasible) {
                real_t tmp_route_tw_penalty = prev_forward_tw_penalty_slack;
                idx_t tmp_next_task_index = route_data.first_task;
                if (0 != pickup_prev_task_index) {
                    tmp_next_task_index = sol.task_data_[pickup_prev_task_index].next_task_index;
                }

                real_t tmp_next_latest_start_service_time = sol.problem_.task_time_windows[1];
                if (0 != tmp_next_task_index) {
                    tmp_route_tw_penalty += sol.task_data_[tmp_next_task_index].backward_tw_penalty_slack;
                    tmp_next_latest_start_service_time = sol.task_data_[tmp_next_task_index].latest_start_service_time;
                }
                tmp_route_tw_penalty += std::max(
                    0.0, std::max(GetEarliestStartTime(sol.problem_, pickup_task_index), current_arrival_time) -
                             std::min(tmp_next_latest_start_service_time -
                                          sol.problem_.task_service_times[pickup_task_index] -
                                          GetDistance(sol.problem_, pickup_task_index, tmp_next_task_index),
                                      GetLatestStartTime(sol.problem_, pickup_task_index)));

                if (tmp_route_tw_penalty > 0.0) {
                    infeasible = true;
                }
            }

            if (infeasible) {
                pickup_prev_task_index = pickup_prev_next_task_index;

                if (0 == pickup_prev_task_index) {
                    break;
                } else {
                    continue;
                }
            }
        }

        idx_t delivery_prev_task_index = pickup_task_index;

        prev_departure_time = current_start_service_time + sol.problem_.task_service_times[pickup_task_index];
        prev_forward_tw_penalty_slack = current_forward_tw_penalty_slack;
        prev_vehicle_load = current_vehicle_load;
        prev_accumulated_capacity_violations = current_accumulated_capacity_violations;

        while (true) {
            current_arrival_time =
                prev_departure_time + GetDistance(sol.problem_, delivery_prev_task_index, delivery_task_index);
            current_vehicle_load = prev_vehicle_load + sol.problem_.task_demands[delivery_task_index];
            current_forward_tw_penalty_slack =
                prev_forward_tw_penalty_slack +
                std::max(0.0, current_arrival_time - GetLatestStartTime(sol.problem_, delivery_task_index));
            current_accumulated_capacity_violations = prev_accumulated_capacity_violations +
                                                      std::max(0, current_vehicle_load - sol.problem_.vehicle_capacity);
            current_start_service_time =
                std::min(std::max(current_arrival_time, GetEarliestStartTime(sol.problem_, delivery_task_index)),
                         GetLatestStartTime(sol.problem_, delivery_task_index));

            real_t tmp_route_tw_penalty = prev_forward_tw_penalty_slack;
            idx_t tmp_next_task_index;
            if (pickup_task_index != delivery_prev_task_index) {
                tmp_next_task_index = sol.task_data_[delivery_prev_task_index].next_task_index;
            } else {
                if (0 != pickup_prev_task_index) {
                    tmp_next_task_index = sol.task_data_[pickup_prev_task_index].next_task_index;
                } else {
                    tmp_next_task_index = route_data.first_task;
                }
            }

            real_t tmp_next_latest_start_service_time = sol.problem_.task_time_windows[1];
            if (0 != tmp_next_task_index) {
                tmp_route_tw_penalty += sol.task_data_[tmp_next_task_index].backward_tw_penalty_slack;
                tmp_next_latest_start_service_time = sol.task_data_[tmp_next_task_index].latest_start_service_time;
            }
            tmp_route_tw_penalty +=
                std::max(0.0, std::max(current_arrival_time, GetEarliestStartTime(sol.problem_, delivery_task_index)) -
                                  std::min(tmp_next_latest_start_service_time -
                                               sol.problem_.task_service_times[delivery_task_index] -
                                               GetDistance(sol.problem_, delivery_task_index, tmp_next_task_index),
                                           GetLatestStartTime(sol.problem_, delivery_task_index)));

            if (!allow_infeasible) {
                bool infeasible = current_forward_tw_penalty_slack > 0.0 || current_accumulated_capacity_violations > 0;
                if (infeasible) {
                    break;
                }

                if (!infeasible) {
                    if (tmp_route_tw_penalty > 0.0) {
                        infeasible = true;
                    }
                }

                if (infeasible) {
                    if (0 == tmp_next_task_index) {
                        break;
                    } else {
                        current_arrival_time = prev_departure_time +
                                               GetDistance(sol.problem_, delivery_prev_task_index, tmp_next_task_index);
                        current_vehicle_load = prev_vehicle_load + sol.problem_.task_demands[tmp_next_task_index];
                        current_forward_tw_penalty_slack =
                            prev_forward_tw_penalty_slack +
                            std::max(0.0, current_arrival_time - GetLatestStartTime(sol.problem_, tmp_next_task_index));
                        current_accumulated_capacity_violations =
                            prev_accumulated_capacity_violations +
                            std::max(0, current_vehicle_load - sol.problem_.vehicle_capacity);
                        current_start_service_time = std::min(
                            std::max(current_arrival_time, GetEarliestStartTime(sol.problem_, tmp_next_task_index)),
                            GetLatestStartTime(sol.problem_, tmp_next_task_index));

                        prev_departure_time = current_start_service_time + sol.problem_.task_service_times[tmp_next_task_index];
                        prev_forward_tw_penalty_slack = current_forward_tw_penalty_slack;
                        prev_vehicle_load = current_vehicle_load;
                        prev_accumulated_capacity_violations = current_accumulated_capacity_violations;
                        delivery_prev_task_index = tmp_next_task_index;
                        continue;
                    }
                }
            }

            demand_t tmp_old_capacity_violations;
            if (delivery_prev_task_index == pickup_task_index) {
                if (0 == pickup_prev_task_index) {
                    tmp_old_capacity_violations = 0;
                } else {
                    tmp_old_capacity_violations =
                        sol.task_data_[pickup_prev_task_index].accumulated_capacity_violations;
                }
            } else {
                tmp_old_capacity_violations = sol.task_data_[delivery_prev_task_index].accumulated_capacity_violations;
            }

            insertions.emplace_back();
            InsertionData& insertion_data = insertions.back();
            insertion_data.route_index = route_index;
            insertion_data.pickup_prev_task_index = pickup_prev_task_index;
            if (delivery_prev_task_index == pickup_task_index) {
                insertion_data.delivery_prev_task_index = pickup_prev_task_index;
            } else {
                insertion_data.delivery_prev_task_index = delivery_prev_task_index;
            }
            insertion_data.delta_capacity_violations =
                current_accumulated_capacity_violations - tmp_old_capacity_violations;
            insertion_data.delta_tw_penalty = tmp_route_tw_penalty - route_data.tw_violations;
            insertion_data.delta_distance = 0.0;
            if (pickup_prev_task_index == delivery_prev_task_index) {
                insertion_data.delta_distance = GetDistance(sol.problem_, pickup_prev_task_index, pickup_task_index) +
                                                GetDistance(sol.problem_, pickup_task_index, delivery_task_index);
                if (0 == pickup_prev_task_index) {
                    insertion_data.delta_distance +=
                        GetDistance(sol.problem_, delivery_task_index, route_data.first_task);
                    insertion_data.delta_distance -= GetDistance(sol.problem_, 0, route_data.first_task);
                } else {
                    insertion_data.delta_distance += GetDistance(
                        sol.problem_, delivery_task_index, sol.task_data_[pickup_prev_task_index].next_task_index);
                    insertion_data.delta_distance -= GetDistance(
                        sol.problem_, pickup_prev_task_index, sol.task_data_[pickup_prev_task_index].next_task_index);
                }
            } else {
                if (0 == pickup_prev_task_index) {
                    insertion_data.delta_distance +=
                        GetDistance(sol.problem_, 0, pickup_task_index) +
                        GetDistance(sol.problem_, pickup_task_index, route_data.first_task) -
                        GetDistance(sol.problem_, 0, route_data.first_task);
                } else {
                    insertion_data.delta_distance +=
                        GetDistance(sol.problem_, pickup_prev_task_index, pickup_task_index) +
                        GetDistance(sol.problem_, pickup_task_index,
                                    sol.task_data_[pickup_prev_task_index].next_task_index) -
                        GetDistance(sol.problem_, pickup_prev_task_index,
                                    sol.task_data_[pickup_prev_task_index].next_task_index);
                }

                insertion_data.delta_distance +=
                    GetDistance(sol.problem_, delivery_prev_task_index, delivery_task_index) +
                    GetDistance(sol.problem_, delivery_task_index,
                                sol.task_data_[delivery_prev_task_index].next_task_index) -
                    GetDistance(sol.problem_, delivery_prev_task_index,
                                sol.task_data_[delivery_prev_task_index].next_task_index);
            }
            insertion_data.new_vehicle = false;

            if (0 == tmp_next_task_index) {
                break;
            }

            current_arrival_time =
                prev_departure_time + GetDistance(sol.problem_, delivery_prev_task_index, tmp_next_task_index);
            current_vehicle_load = prev_vehicle_load + sol.problem_.task_demands[tmp_next_task_index];
            current_forward_tw_penalty_slack =
                prev_forward_tw_penalty_slack +
                std::max(0.0, current_arrival_time - GetLatestStartTime(sol.problem_, tmp_next_task_index));
            current_accumulated_capacity_violations = prev_accumulated_capacity_violations +
                                                      std::max(0, current_vehicle_load - sol.problem_.vehicle_capacity);
            current_start_service_time =
                std::min(std::max(current_arrival_time, GetEarliestStartTime(sol.problem_, tmp_next_task_index)),
                         GetLatestStartTime(sol.problem_, tmp_next_task_index));

            prev_departure_time = current_start_service_time + sol.problem_.task_service_times[tmp_next_task_index];
            prev_forward_tw_penalty_slack = current_forward_tw_penalty_slack;
            prev_vehicle_load = current_vehicle_load;
            prev_accumulated_capacity_violations = current_accumulated_capacity_violations;
            delivery_prev_task_index = tmp_next_task_index;
        }
    }
}

void BestInsert(SREXSol& sol) {
    std::unordered_map<idx_t, std::unordered_map<idx_t, std::vector<InsertionData>>> insert_data_map;
    std::unordered_map<idx_t, idx_t> unserved_task_index_map;
    for (idx_t i = 0; i < sol.unserved_task_num_; ++i) {
        idx_t unserved_task_index = sol.unserved_tasks_[i];
        unserved_task_index_map[unserved_task_index] = i;

        if (1 == unserved_task_index % 2) {
            for (idx_t route_index = 0; route_index < (idx_t)sol.route_data_.size(); ++route_index) {
                CalInsertionData(sol, unserved_task_index, unserved_task_index + 1, route_index,
                                 insert_data_map[unserved_task_index][route_index], false);
            }
        }
    }

    while (0 < sol.unserved_task_num_) {
        InsertionData* best_insertion_data = nullptr;
        idx_t best_insertion_pickup_index = -1;
        real_t min_cost = kDoubleMax;

        for (auto& [task_index, route_insert_data_map] : insert_data_map) {
            for (auto& [route_index, insertions] : route_insert_data_map) {
                for (InsertionData& insertion_data : insertions) {
                    if (insertion_data.delta_tw_penalty > 0.0) {
                        continue;
                    }

                    if (insertion_data.delta_capacity_violations > 0) {
                        continue;
                    }

                    real_t cost = insertion_data.delta_distance;
                    if (insertion_data.new_vehicle) {
                        cost += kEmptyVehicleCost;
                    }

                    if (cost < min_cost) {
                        min_cost = cost;
                        best_insertion_data = &insertion_data;
                        best_insertion_pickup_index = task_index;
                    }
                }
            }
        }

        if (nullptr == best_insertion_data) {
            break;
        } else {
            idx_t pickup_prev_task_index = best_insertion_data->pickup_prev_task_index;
            idx_t delivery_prev_task_index = best_insertion_data->delivery_prev_task_index;
            idx_t route_index = best_insertion_data->route_index;

            sol.InsertTask(route_index, best_insertion_pickup_index, pickup_prev_task_index,
                           best_insertion_pickup_index + 1, delivery_prev_task_index, true);

            idx_t last_unserved_task_index = sol.unserved_tasks_[sol.unserved_task_num_ - 1];
            if (last_unserved_task_index != best_insertion_pickup_index) {
                sol.unserved_tasks_[unserved_task_index_map[best_insertion_pickup_index]] = last_unserved_task_index;
                unserved_task_index_map[last_unserved_task_index] =
                    unserved_task_index_map[best_insertion_pickup_index];
            }
            sol.unserved_task_num_ -= 1;

            last_unserved_task_index = sol.unserved_tasks_[sol.unserved_task_num_ - 1];
            if (last_unserved_task_index != best_insertion_pickup_index + 1) {
                sol.unserved_tasks_[unserved_task_index_map[best_insertion_pickup_index + 1]] =
                    last_unserved_task_index;
                unserved_task_index_map[last_unserved_task_index] =
                    unserved_task_index_map[best_insertion_pickup_index + 1];
            }
            sol.unserved_task_num_ -= 1;

            insert_data_map[best_insertion_pickup_index].clear();
            insert_data_map[best_insertion_pickup_index + 1].clear();

            for (auto& [task_index, route_insert_data_map] : insert_data_map) {
                for (auto& [tmp_route_index, insertions] : route_insert_data_map) {
                    if (tmp_route_index != route_index) {
                        continue;
                    }
                    insertions.clear();
                    CalInsertionData(sol, task_index, task_index + 1, tmp_route_index, insertions, false);
                }
            }
        }
    }
}

void GreedyInsertionWithBlinks(SREXSol& sol) {
    double sort_probability = GetRandDouble(0.0, 1.0);
    if (sort_probability < kSortBasedOnRandomProbability) {
        std::shuffle(sol.unserved_tasks_.begin(), sol.unserved_tasks_.begin() + sol.unserved_task_num_,
                     GetRandomGenerator());
    } else if (sort_probability < kSortBasedOnRandomProbability + kSortBasedOnDemandProbability) {
        std::sort(sol.unserved_tasks_.begin(), sol.unserved_tasks_.begin() + sol.unserved_task_num_,
                  [&sol](idx_t task_index_1, idx_t task_index_2) {
                      return sol.problem_.task_demands[task_index_1] > sol.problem_.task_demands[task_index_2];
                  });
    } else if (sort_probability <
               kSortBasedOnRandomProbability + kSortBasedOnRandomProbability + kSortBasedOnFarDistanceProbability) {
        std::sort(sol.unserved_tasks_.begin(), sol.unserved_tasks_.begin() + sol.unserved_task_num_,
                  [&sol](idx_t task_index_1, idx_t task_index_2) {
                      return GetDistance(sol.problem_, 0, task_index_1) < GetDistance(sol.problem_, 0, task_index_2);
                  });
    } else {
        std::sort(sol.unserved_tasks_.begin(), sol.unserved_tasks_.begin() + sol.unserved_task_num_,
                  [&sol](idx_t task_index_1, idx_t task_index_2) {
                      return GetDistance(sol.problem_, 0, task_index_1) > GetDistance(sol.problem_, 0, task_index_2);
                  });
    }

    std::unordered_map<idx_t, std::unordered_map<idx_t, std::vector<InsertionData>>> insert_data_map;
    std::unordered_map<idx_t, idx_t> unserved_task_index_map;
    for (idx_t i = 0; i < sol.unserved_task_num_; ++i) {
        idx_t unserved_task_index = sol.unserved_tasks_[i];
        unserved_task_index_map[unserved_task_index] = i;

        if (1 == unserved_task_index % 2) {
            for (idx_t route_index = 0; route_index < (idx_t)sol.route_data_.size(); ++route_index) {
                CalInsertionData(sol, unserved_task_index, unserved_task_index + 1, route_index,
                                 insert_data_map[unserved_task_index][route_index], false);
            }
        }
    }

    for (idx_t i = 0; i < sol.unserved_task_num_; ++i) {
        idx_t unserved_task_index = sol.unserved_tasks_[i];

        if (0 == unserved_task_index % 2) {
            continue;
        }

        InsertionData* best_insertion_data = nullptr;
        idx_t best_insertion_pickup_index = -1;
        real_t min_cost = kDoubleMax;

        if (insert_data_map.find(unserved_task_index) == insert_data_map.end()) {
            for (idx_t route_index = 0; route_index < (idx_t)sol.route_data_.size(); ++route_index) {
                CalInsertionData(sol, unserved_task_index, unserved_task_index + 1, route_index,
                                 insert_data_map[unserved_task_index][route_index], false);
            }
        }

        for (auto& [route_index, insertions] : insert_data_map[unserved_task_index]) {
            for (InsertionData& insertion_data : insertions) {
                if (insertion_data.delta_tw_penalty > 0.0) {
                    continue;
                }

                if (insertion_data.delta_capacity_violations > 0) {
                    continue;
                }

                double tmp_prob = GetRandDouble(0.0, 1.0);
                if (tmp_prob >= 1 - kBlinkRate) {
                    continue;
                }

                real_t cost = insertion_data.delta_distance;
                if (insertion_data.new_vehicle) {
                    cost += kEmptyVehicleCost;
                }

                if (cost < min_cost) {
                    min_cost = cost;
                    best_insertion_data = &insertion_data;
                    best_insertion_pickup_index = unserved_task_index;
                }
            }
        }

        if (nullptr == best_insertion_data) {
            break;
        } else {
            idx_t pickup_prev_task_index = best_insertion_data->pickup_prev_task_index;
            idx_t delivery_prev_task_index = best_insertion_data->delivery_prev_task_index;
            idx_t route_index = best_insertion_data->route_index;

            sol.InsertTask(route_index, best_insertion_pickup_index, pickup_prev_task_index,
                           best_insertion_pickup_index + 1, delivery_prev_task_index, true);

            idx_t last_unserved_task_index = sol.unserved_tasks_[sol.unserved_task_num_ - 1];
            if (last_unserved_task_index != best_insertion_pickup_index) {
                sol.unserved_tasks_[unserved_task_index_map[best_insertion_pickup_index]] = last_unserved_task_index;
                unserved_task_index_map[last_unserved_task_index] =
                    unserved_task_index_map[best_insertion_pickup_index];
            }
            sol.unserved_task_num_ -= 1;

            last_unserved_task_index = sol.unserved_tasks_[sol.unserved_task_num_ - 1];
            if (last_unserved_task_index != best_insertion_pickup_index + 1) {
                sol.unserved_tasks_[unserved_task_index_map[best_insertion_pickup_index + 1]] =
                    last_unserved_task_index;
                unserved_task_index_map[last_unserved_task_index] =
                    unserved_task_index_map[best_insertion_pickup_index + 1];
            }
            sol.unserved_task_num_ -= 1;

            insert_data_map[best_insertion_pickup_index].clear();
            insert_data_map[best_insertion_pickup_index + 1].clear();

            for (auto& [task_index, route_insert_data_map] : insert_data_map) {
                for (auto& [tmp_route_index, insertions] : route_insert_data_map) {
                    if (tmp_route_index != route_index) {
                        continue;
                    }
                    insertions.clear();
                    CalInsertionData(sol, task_index, task_index + 1, tmp_route_index, insertions, false);
                }
            }
        }
    }
}

}  // namespace lcs_srex