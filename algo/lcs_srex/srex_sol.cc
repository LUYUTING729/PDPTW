#include "algo/lcs_srex/srex_sol.h"

#include <cassert>
#include <numeric>

namespace lcs_srex {
SREXSol::SREXSol(const Solution& solution) : problem_(solution.problem_) {
    task_data_.resize(problem_.task_num);
    route_data_.resize(solution.routes_.size());

    unserved_task_num_ = problem_.task_num - 1;
    unserved_tasks_.resize(unserved_task_num_);
    std::iota(unserved_tasks_.begin(), unserved_tasks_.end(), 1);

    std::vector<uint8_t> visited_flags(0);
    idx_t route_index = 0;
    for (const auto& route : solution.routes_) {
        if (route.empty()) {
            continue;
        }

        idx_t prev_task_index = 0;
        for (size_t j = 1; j < route.size() - 1; ++j) {
            idx_t current_task_index = route[j];
            visited_flags[current_task_index] = 1;
            TaskData& task_data = task_data_[current_task_index];
            task_data.route_index = route_index;
            task_data.prev_task_index = prev_task_index;

            if (j == route.size() - 2) {
                task_data.next_task_index = 0;
            }

            if (j > 1) {
                task_data_[prev_task_index].next_task_index = current_task_index;
            }
            prev_task_index = current_task_index;
        }

        route_data_[route_index].first_task = route[1];
        route_data_[route_index].last_task = route[route.size() - 2];
        route_data_[route_index].task_num = (idx_t)route.size() - 2;

        route_index += 1;
    }

    route_data_.resize(route_index);

    for (idx_t i = 0; i < unserved_task_num_;) {
        if (visited_flags[unserved_tasks_[i]]) {
            unserved_tasks_[i] = unserved_tasks_[unserved_task_num_ - 1];
            unserved_task_num_ -= 1;
        } else {
            ++i;
        }
    }

    for (idx_t i = 0; i < route_index; ++i) {
        UpdateRouteData(i, route_data_[i].first_task, route_data_[i].last_task);
    }
    CalTotalValue();
}

void SREXSol::UpdateRouteData(idx_t route_index, idx_t forward_start_task_index, idx_t backward_start_task_index) {
    total_value_valid_ = false;

    RouteData& route_data = route_data_[route_index];

    idx_t current_index = route_data.first_task;
    idx_t prev_index = 0;
    real_t prev_departure_time = GetEarliestStartTime(problem_, 0);
    real_t prev_forward_tw_penalty_slack = 0.0;
    demand_t prev_load = 0;
    demand_t prev_accumulated_capacity_violations = 0;
    real_t prev_accumulated_distance = 0.0;

    if (route_data.first_task != forward_start_task_index) {
        current_index = forward_start_task_index;
        prev_index = task_data_[current_index].prev_task_index;
        prev_departure_time = task_data_[prev_index].departure_time;
        prev_forward_tw_penalty_slack = task_data_[prev_index].forward_tw_penalty_slack;
        prev_load = task_data_[prev_index].vehicle_load;
        prev_accumulated_capacity_violations = task_data_[prev_index].accumulated_capacity_violations;
        prev_accumulated_distance = task_data_[prev_index].accumulated_distance;
    }

    real_t arc_distance;
    real_t latest_start_time;
    while (0 != current_index) {
        TaskData& task_data = task_data_[current_index];
        arc_distance = GetDistance(problem_, prev_index, current_index);

        task_data.accumulated_distance = prev_accumulated_distance + arc_distance;
        task_data.arrival_time = prev_departure_time + arc_distance;

        latest_start_time = GetLatestStartTime(problem_, current_index);
        task_data.start_service_time = std::min(
            std::max(task_data.arrival_time, GetEarliestStartTime(problem_, current_index)), latest_start_time);
        task_data.departure_time = task_data.start_service_time + problem_.task_service_times[current_index];
        task_data.forward_tw_penalty_slack =
            prev_forward_tw_penalty_slack + std::max(0.0, task_data.arrival_time - latest_start_time);
        task_data.vehicle_load = prev_load + problem_.task_demands[current_index];
        task_data.accumulated_capacity_violations =
            prev_accumulated_capacity_violations + std::max(0, task_data.vehicle_load - problem_.vehicle_capacity);
        task_data.route_index = route_index;

        prev_index = current_index;
        prev_departure_time = task_data.departure_time;
        prev_load = task_data.vehicle_load;
        prev_forward_tw_penalty_slack = task_data.forward_tw_penalty_slack;
        prev_accumulated_capacity_violations = task_data.accumulated_capacity_violations;
        prev_accumulated_distance = task_data.accumulated_distance;

        current_index = task_data.next_task_index;
    }

    arc_distance = GetDistance(problem_, prev_index, 0);
    route_data.distance = prev_accumulated_distance + arc_distance;
    route_data.return_to_depot_time = prev_departure_time + arc_distance;
    route_data.tw_violations =
        prev_forward_tw_penalty_slack + std::max(0.0, route_data.return_to_depot_time - problem_.task_time_windows[1]);

    current_index = route_data.last_task;
    idx_t next_index = 0;
    real_t next_latest_start_service_time = problem_.task_time_windows[1];
    real_t next_backward_tw_penalty_slack = 0.0;

    if (route_data.last_task != backward_start_task_index) {
        current_index = backward_start_task_index;
        next_index = task_data_[current_index].next_task_index;
        next_latest_start_service_time = task_data_[next_index].latest_start_service_time;
        next_backward_tw_penalty_slack = task_data_[next_index].backward_tw_penalty_slack;
    }

    real_t earliest_start_time;
    while (0 != current_index) {
        TaskData& task_data = task_data_[current_index];
        arc_distance = GetDistance(problem_, current_index, next_index);

        latest_start_time = next_latest_start_service_time - arc_distance - problem_.task_service_times[current_index];

        earliest_start_time = GetEarliestStartTime(problem_, current_index);
        task_data.latest_start_service_time =
            std::max(earliest_start_time, std::min(latest_start_time, GetLatestStartTime(problem_, current_index)));
        task_data.backward_tw_penalty_slack =
            next_backward_tw_penalty_slack + std::max(0.0, earliest_start_time - latest_start_time);

        task_data.route_index = route_index;

        next_index = current_index;
        next_latest_start_service_time = task_data.latest_start_service_time;
        next_backward_tw_penalty_slack = task_data.backward_tw_penalty_slack;

        current_index = task_data.prev_task_index;
    }
}

void SREXSol::RemoveRoute(idx_t route_index) {
    total_value_valid_ = false;

    idx_t current_task_index = route_data_[route_index].first_task;

    idx_t next_task_index_bak;
    while (0 != current_task_index) {
        unserved_tasks_[unserved_task_num_] = current_task_index;
        unserved_task_num_ += 1;
        next_task_index_bak = task_data_[current_task_index].next_task_index;
        ResetTaskData(task_data_[current_task_index]);

        current_task_index = next_task_index_bak;
    }

    ResetRouteData(route_data_[route_index]);
}

void SREXSol::RemoveTask(idx_t route_index, idx_t pickup_task_index, idx_t delivery_task_index, bool update_data) {
    total_value_valid_ = false;

    TaskData& pickup_task_data = task_data_[pickup_task_index];
    TaskData& delivery_task_data = task_data_[delivery_task_index];

    idx_t pickup_prev_task_index = pickup_task_data.prev_task_index;
    idx_t pickup_next_task_index = pickup_task_data.next_task_index;
    idx_t pickup_route_index = pickup_task_data.route_index;
    assert(0 <= pickup_prev_task_index);
    assert(0 <= pickup_next_task_index);
    assert(pickup_route_index == route_index);

    idx_t delivery_prev_task_index = delivery_task_data.prev_task_index;
    idx_t delivery_next_task_index = delivery_task_data.next_task_index;
    assert(0 <= delivery_prev_task_index);
    assert(0 <= delivery_next_task_index);
    assert(pickup_route_index == delivery_task_data.route_index);

    if (0 == pickup_prev_task_index) {
        if (pickup_next_task_index == delivery_task_index) {
            if (0 == delivery_next_task_index) {
                ResetRouteData(route_data_[route_index]);
            } else {
                route_data_[route_index].first_task = delivery_next_task_index;
                task_data_[delivery_next_task_index].prev_task_index = 0;
            }
        } else {
            route_data_[route_index].first_task = pickup_next_task_index;
            task_data_[pickup_next_task_index].prev_task_index = 0;

            task_data_[delivery_prev_task_index].next_task_index = delivery_next_task_index;
            if (0 == delivery_next_task_index) {
                route_data_[route_index].last_task = delivery_prev_task_index;
            } else {
                task_data_[delivery_next_task_index].prev_task_index = delivery_prev_task_index;
            }
        }
    } else {
        if (pickup_next_task_index == delivery_task_index) {
            task_data_[pickup_prev_task_index].next_task_index = delivery_next_task_index;

            if (0 == delivery_next_task_index) {
                route_data_[route_index].last_task = pickup_prev_task_index;
            } else {
                task_data_[delivery_next_task_index].prev_task_index = pickup_prev_task_index;
            }
        } else {
            task_data_[pickup_prev_task_index].next_task_index = pickup_next_task_index;
            task_data_[pickup_next_task_index].prev_task_index = pickup_prev_task_index;

            task_data_[delivery_prev_task_index].next_task_index = delivery_next_task_index;
            if (0 == delivery_next_task_index) {
                route_data_[route_index].last_task = delivery_prev_task_index;
            } else {
                task_data_[delivery_next_task_index].prev_task_index = delivery_prev_task_index;
            }
        }
    }

    if (0 < route_data_[route_index].task_num) {
        route_data_[route_index].task_num -= 2;
    }
    ResetTaskData(task_data_[pickup_task_index]);
    ResetTaskData(task_data_[delivery_task_index]);
    unserved_tasks_[unserved_task_num_] = pickup_task_index;
    unserved_task_num_ += 1;
    unserved_tasks_[unserved_task_num_] = delivery_task_index;
    unserved_task_num_ += 1;

    if (update_data && route_data_[route_index].task_num > 0) {
        if (0 == pickup_prev_task_index) {
            if (0 == delivery_next_task_index) {
                UpdateRouteData(route_index, route_data_[route_index].first_task, route_data_[route_index].last_task);
            } else {
                UpdateRouteData(route_index, route_data_[route_index].first_task,
                                task_data_[delivery_next_task_index].prev_task_index);
            }
        } else {
            if (0 == delivery_next_task_index) {
                UpdateRouteData(route_index, task_data_[pickup_prev_task_index].next_task_index,
                                route_data_[route_index].last_task);
            } else {
                UpdateRouteData(route_index, task_data_[pickup_prev_task_index].next_task_index,
                                task_data_[delivery_next_task_index].prev_task_index);
            }
        }
    }
}

void SREXSol::InsertTask(idx_t route_index, idx_t pickup_task_index, idx_t pickup_prev_task_index,
                         idx_t delivery_task_index, idx_t delivery_prev_task_index, bool update_data) {
    total_value_valid_ = false;

    idx_t pickup_next_task_index;
    idx_t delivery_next_task_index;
    if (0 == pickup_prev_task_index) {
        pickup_next_task_index = route_data_[route_index].first_task;
    } else {
        pickup_next_task_index = task_data_[pickup_prev_task_index].next_task_index;
    }

    if (0 == delivery_prev_task_index) {
        delivery_next_task_index = route_data_[route_index].first_task;
    } else {
        delivery_next_task_index = task_data_[delivery_prev_task_index].next_task_index;
    }

    if (0 == pickup_prev_task_index) {
        if (0 == delivery_prev_task_index) {
            route_data_[route_index].first_task = pickup_task_index;
            task_data_[pickup_task_index].prev_task_index = 0;

            task_data_[pickup_task_index].next_task_index = delivery_task_index;
            task_data_[delivery_task_index].prev_task_index = pickup_task_index;
        } else {
            route_data_[route_index].first_task = pickup_task_index;
            task_data_[pickup_task_index].prev_task_index = 0;

            task_data_[pickup_task_index].next_task_index = pickup_next_task_index;
            task_data_[pickup_next_task_index].prev_task_index = pickup_task_index;

            task_data_[delivery_prev_task_index].next_task_index = delivery_task_index;
            task_data_[delivery_task_index].prev_task_index = delivery_prev_task_index;
        }
    } else {
        if (pickup_prev_task_index == delivery_prev_task_index) {
            task_data_[pickup_prev_task_index].next_task_index = pickup_task_index;
            task_data_[pickup_task_index].prev_task_index = pickup_prev_task_index;

            task_data_[pickup_task_index].next_task_index = delivery_task_index;
            task_data_[delivery_task_index].prev_task_index = pickup_task_index;
        } else {
            task_data_[pickup_prev_task_index].next_task_index = pickup_task_index;
            task_data_[pickup_task_index].prev_task_index = pickup_prev_task_index;

            task_data_[pickup_task_index].next_task_index = pickup_next_task_index;
            task_data_[pickup_next_task_index].prev_task_index = pickup_task_index;

            task_data_[delivery_prev_task_index].next_task_index = delivery_task_index;
            task_data_[delivery_task_index].prev_task_index = delivery_prev_task_index;
        }
    }

    if (0 >= delivery_next_task_index) {
        task_data_[delivery_task_index].next_task_index = 0;
        route_data_[route_index].last_task = delivery_task_index;
    } else {
        task_data_[delivery_task_index].next_task_index = delivery_next_task_index;
        task_data_[delivery_next_task_index].prev_task_index = delivery_task_index;
    }

    task_data_[pickup_task_index].route_index = route_index;
    task_data_[delivery_task_index].route_index = route_index;
    route_data_[route_index].task_num += 2;

    for (idx_t i = 0; i < unserved_task_num_;) {
        idx_t tmp_task_index = unserved_tasks_[i];
        if (tmp_task_index == pickup_task_index || tmp_task_index == delivery_task_index) {
            unserved_tasks_[i] = unserved_tasks_[unserved_task_num_ - 1];
            unserved_task_num_ -= 1;
        } else {
            ++i;
        }
    }
    if (update_data) {
        UpdateRouteData(route_index, pickup_task_index, delivery_task_index);
    }
}

void SREXSol::CopyRoute(const SREXSol& from_sol, idx_t from_route_index, idx_t to_route_index) {
    assert(0 >= route_data_[to_route_index].first_task);
    assert(0 >= route_data_[to_route_index].task_num);
    assert(0 < from_sol.route_data_[from_route_index].first_task);

    total_value_valid_ = false;

    route_data_[to_route_index] = from_sol.route_data_[from_route_index];
    idx_t current_task_index = from_sol.route_data_[from_route_index].first_task;

    while (0 != current_task_index) {
        task_data_[current_task_index] = from_sol.task_data_[current_task_index];
        task_data_[current_task_index].route_index = to_route_index;

        current_task_index = task_data_[current_task_index].next_task_index;
    }

    for (int i = 0; i < unserved_task_num_;) {
        if (0 <= task_data_[unserved_tasks_[i]].route_index) {
            unserved_tasks_[i] = unserved_tasks_[unserved_task_num_ - 1];
            unserved_task_num_ -= 1;
        } else {
            ++i;
        }
    }
}

void SREXSol::CalTotalValue() {
    total_distance_ = 0.0;
    total_tw_violations_ = 0.0;
    total_capacity_violations_ = 0;
    for (const RouteData& route_data : route_data_) {
        if (0 >= route_data.task_num) {
            continue;
        }
        total_distance_ += route_data.distance;
        total_tw_violations_ += route_data.tw_violations;
        assert(0 < route_data.last_task);
        total_capacity_violations_ += task_data_[route_data.last_task].accumulated_capacity_violations;
    }

    total_value_valid_ = true;
}

real_t SREXSol::GetTotalDistance() {
    if (total_value_valid_) {
        return total_distance_;
    } else {
        CalTotalValue();
        return total_distance_;
    }
}

real_t SREXSol::GetTotalTWViolations() {
    if (total_value_valid_) {
        return total_tw_violations_;
    } else {
        CalTotalValue();
        return total_tw_violations_;
    }
}

demand_t SREXSol::GetTotalCapacityViolations() {
    if (total_value_valid_) {
        return total_capacity_violations_;
    } else {
        CalTotalValue();
        return total_capacity_violations_;
    }
}
}  // namespace lcs_srex