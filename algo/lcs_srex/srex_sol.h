#ifndef AMDAHL_SRC_ALGO_LCS_SREX_SREX_SOL_H_
#define AMDAHL_SRC_ALGO_LCS_SREX_SREX_SOL_H_

#include <vector>

#include "common/constants.h"
#include "model/pdptw.h"
#include "model/solution.h"

namespace lcs_srex {
struct TaskData {
    idx_t prev_task_index{-1};
    idx_t next_task_index{-1};
    real_t arrival_time{kDoubleMax};
    real_t start_service_time{kDoubleMax};
    real_t latest_start_service_time{kDoubleMax};
    real_t departure_time{kDoubleMax};
    real_t forward_tw_penalty_slack{0.0};
    real_t backward_tw_penalty_slack{0.0};
    demand_t vehicle_load{0};
    demand_t accumulated_capacity_violations{0};
    real_t accumulated_distance{0.0};
    idx_t route_index{-1};
};

inline void ResetTaskData(TaskData& task_data) {
    task_data.prev_task_index = -1;
    task_data.next_task_index = -1;
    task_data.arrival_time = kDoubleMax;
    task_data.start_service_time = kDoubleMax;
    task_data.latest_start_service_time = kDoubleMax;
    task_data.departure_time = kDoubleMax;
    task_data.forward_tw_penalty_slack = 0.0;
    task_data.backward_tw_penalty_slack = 0.0;
    task_data.vehicle_load = 0;
    task_data.accumulated_capacity_violations = 0;
    task_data.accumulated_distance = 0.0;
    task_data.route_index = -1;
}

struct RouteData {
    idx_t first_task{-1};
    idx_t last_task{-1};
    idx_t task_num{0};

    real_t distance{0.0};
    real_t return_to_depot_time{0.0};
    real_t tw_violations{0.0};
};

inline void ResetRouteData(RouteData& route_data) {
    route_data.first_task = -1;
    route_data.last_task = -1;
    route_data.task_num = 0;
    route_data.distance = 0.0;
    route_data.return_to_depot_time = 0.0;
    route_data.tw_violations = 0.0;
}

class SREXSol {
   public:
    explicit SREXSol(const Solution& solution);
    SREXSol(const SREXSol& solution) : problem_(solution.problem_) {
        task_data_ = solution.task_data_;
        route_data_ = solution.route_data_;
        unserved_tasks_ = solution.unserved_tasks_;
        unserved_task_num_ = solution.unserved_task_num_;

        total_value_valid_ = solution.total_value_valid_;
        total_distance_ = solution.total_distance_;
        total_tw_violations_ = solution.total_tw_violations_;
        total_capacity_violations_ = solution.total_capacity_violations_;
    }
    const PDPTW& problem_;

    std::vector<TaskData> task_data_;
    std::vector<RouteData> route_data_;
    std::vector<idx_t> unserved_tasks_;
    int unserved_task_num_{0};

    bool total_value_valid_{false};
    real_t total_distance_{0.0};
    real_t total_tw_violations_{0.0};
    demand_t total_capacity_violations_{0};

    void UpdateRouteData(idx_t route_index, idx_t forward_start_task_index, idx_t backward_start_task_index);
    void RemoveRoute(idx_t route_index);
    void RemoveTask(idx_t route_index, idx_t pickup_task_index, idx_t delivery_task_index, bool update_data);
    void InsertTask(idx_t route_index, idx_t pickup_task_index, idx_t pickup_prev_task_index, idx_t delivery_task_index,
                    idx_t delivery_prev_task_index, bool update_data);
    void CopyRoute(const SREXSol& from_sol, idx_t from_route_index, idx_t to_route_index);

    void CalTotalValue();
    real_t GetTotalDistance();
    real_t GetTotalTWViolations();
    demand_t GetTotalCapacityViolations();
};

}  // namespace lcs_srex

#endif