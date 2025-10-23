#include "algo/lcs_srex/crossover.h"

#include <cassert>
#include <unordered_set>

#include "algo/lcs_srex/insert.h"

namespace lcs_srex {
SREXSol Crossover(const SREXSol& first_sol, const SREXSol& second_sol, const RouteSubset& selective_route_subset) {
    std::unordered_set<int> tasks_in_first_route_subset;
    std::unordered_set<int> tasks_in_second_route_subset;
    std::vector<int> tasks_in_first_not_in_second;
    std::vector<int> tasks_in_second_not_in_first;

    for (idx_t route_index : selective_route_subset.first_route_indices_) {
        idx_t current_task_index = first_sol.route_data_[route_index].first_task;
        while (current_task_index) {
            tasks_in_first_route_subset.insert(current_task_index);
            current_task_index = first_sol.task_data_[current_task_index].next_task_index;
        }
    }

    for (idx_t route_index : selective_route_subset.second_route_indices_) {
        idx_t current_task_index = second_sol.route_data_[route_index].first_task;
        while (current_task_index) {
            if (tasks_in_first_route_subset.find(current_task_index) == tasks_in_first_route_subset.end()) {
                tasks_in_second_not_in_first.push_back(current_task_index);
            }
            tasks_in_second_route_subset.insert(current_task_index);
            current_task_index = second_sol.task_data_[current_task_index].next_task_index;
        }
    }

    for (idx_t task_index : tasks_in_first_route_subset) {
        if (tasks_in_second_route_subset.find(task_index) == tasks_in_second_route_subset.end()) {
            tasks_in_first_not_in_second.push_back(task_index);
        }
    }

    SREXSol first_child_sol = first_sol;

    for (idx_t route_index : selective_route_subset.first_route_indices_) {
        first_child_sol.RemoveRoute(route_index);
    }

    std::unordered_set<idx_t> affected_route_indices;
    for (idx_t task_index : tasks_in_second_not_in_first) {
        if (0 == task_index % 2) {
            idx_t tmp_route_index = first_child_sol.task_data_[task_index].route_index;
            affected_route_indices.insert(tmp_route_index);
            first_child_sol.RemoveTask(tmp_route_index, task_index - 1, task_index, false);
        }
    }

    for (idx_t route_index : affected_route_indices) {
        if (0 >= first_child_sol.route_data_[route_index].task_num) {
            ResetRouteData(first_child_sol.route_data_[route_index]);
        } else {
            assert(0 < first_child_sol.route_data_[route_index].first_task);
            assert(0 < first_child_sol.route_data_[route_index].last_task);
            first_child_sol.UpdateRouteData(route_index, first_child_sol.route_data_[route_index].first_task,
                                            first_child_sol.route_data_[route_index].last_task);
        }
    }

    assert(selective_route_subset.first_route_indices_.size() == selective_route_subset.second_route_indices_.size());

    for (size_t i = 0; i < selective_route_subset.first_route_indices_.size(); ++i) {
        idx_t first_route = selective_route_subset.first_route_indices_[i];
        idx_t second_route = selective_route_subset.second_route_indices_[i];

        first_child_sol.CopyRoute(second_sol, second_route, first_route);
    }

    BestInsert(first_child_sol);

    SREXSol second_child_sol = first_sol;

    for (idx_t route_index : selective_route_subset.first_route_indices_) {
        second_child_sol.RemoveRoute(route_index);
    }

    for (size_t i = 0; i < selective_route_subset.first_route_indices_.size(); ++i) {
        idx_t first_route = selective_route_subset.first_route_indices_[i];
        idx_t second_route = selective_route_subset.second_route_indices_[i];

        bool complete_route = true;

        const RouteData& route_in_second_sol = second_sol.route_data_[second_route];
        idx_t first_task_index = -1;
        idx_t prev_task_index = 0;
        idx_t last_same_with_second_task_index = -1;

        idx_t current_task_index = route_in_second_sol.first_task;
        while (0 != current_task_index) {
            if (tasks_in_first_route_subset.find(current_task_index) == tasks_in_first_route_subset.end()) {
                complete_route = false;
                continue;
            } else {
                if (-1 == first_task_index) {
                    first_task_index = current_task_index;
                }
                second_child_sol.task_data_[current_task_index] = second_sol.task_data_[current_task_index];
                second_child_sol.task_data_[current_task_index].prev_task_index = prev_task_index;
                if (0 != prev_task_index) {
                    second_child_sol.task_data_[prev_task_index].next_task_index = current_task_index;
                }
                second_child_sol.task_data_[current_task_index].route_index = first_route;

                if (complete_route) {
                    last_same_with_second_task_index = current_task_index;
                }

                prev_task_index = current_task_index;
                current_task_index = second_sol.task_data_[current_task_index].next_task_index;
            }
        }

        if (-1 != first_task_index) {
            second_child_sol.route_data_[first_route].first_task = first_task_index;
            second_child_sol.route_data_[first_route].last_task = prev_task_index;

            if (complete_route) {
                second_child_sol.route_data_[first_route] = route_in_second_sol;
            } else {
                second_child_sol.UpdateRouteData(
                    first_route, second_child_sol.task_data_[last_same_with_second_task_index].next_task_index,
                    second_child_sol.route_data_[first_route].last_task);
            }
        } else {
            ResetRouteData(second_child_sol.route_data_[first_route]);
        }
    }

    for (size_t i = 0; i < tasks_in_first_not_in_second.size(); ++i) {
        second_child_sol.unserved_tasks_[i] = tasks_in_first_not_in_second[i];
    }
    second_child_sol.unserved_task_num_ = (int)tasks_in_first_not_in_second.size();

    BestInsert(second_child_sol);

    real_t first_sol_total_distance = first_child_sol.GetTotalDistance();
    real_t second_sol_total_distance = second_child_sol.GetTotalDistance();
    if (first_sol_total_distance < second_sol_total_distance) {
        return first_child_sol;
    } else {
        return second_child_sol;
    }
}
}  // namespace lcs_srex