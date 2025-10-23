#include "algo/lcs_srex/removal.h"

#include <unordered_set>
namespace lcs_srex {

void StringRemoval(SREXSol& sol) {
    double avg_task_num = 0.0;
    for (const RouteData& route_data : sol.route_data_) {
        avg_task_num += route_data.task_num;
    }
    avg_task_num /= (double)sol.route_data_.size();

    double max_string_cardinality = GetMaxStringCardinality(avg_task_num);
    double max_string_num = GetMaxStringNum(max_string_cardinality);
    double string_num = GetStringNum(max_string_num);

    idx_t seed_task = GetRandInt(1, sol.problem_.task_num);
    std::unordered_set<idx_t> ruined_routes;

    std::vector<idx_t> to_removed_task_indices;
    to_removed_task_indices.reserve(sol.problem_.task_num);
    idx_t to_removed_task_num = 0;

    for (idx_t adjacent_task_index : sol.problem_.adjacency_list[seed_task]) {
        idx_t adjacent_route_index = sol.task_data_[adjacent_task_index].route_index;
        if (adjacent_route_index < 0) {
            continue;
        }

        if (ruined_routes.find(adjacent_route_index) != ruined_routes.end()) {
            continue;
        }

        double max_route_string_cardinality =
            GetMaxRouteStringCardinality(max_string_cardinality, sol.route_data_[adjacent_route_index].task_num);
        idx_t removed_string_cardinality = GetRemovedStringCardinality(max_route_string_cardinality);

        idx_t max_prev_task_num = removed_string_cardinality - 1;
        int prev_task_num = 0;
        idx_t prev_task_index = adjacent_task_index;
        while (0 != prev_task_index) {
            prev_task_index = sol.task_data_[prev_task_index].prev_task_index;
            prev_task_num++;
            if (prev_task_num >= max_prev_task_num) {
                break;
            }
        }

        prev_task_num = GetRandInt(0, prev_task_num);
        to_removed_task_num = 0;
        idx_t visited_task_num = 0;
        prev_task_index = adjacent_task_index;
        while (0 != prev_task_index) {
            if (visited_task_num >= prev_task_num) {
                break;
            }
            prev_task_index = sol.task_data_[prev_task_index].prev_task_index;
            if (0 == prev_task_index % 2) {
                to_removed_task_indices[to_removed_task_num] = prev_task_index - 1;
                to_removed_task_num++;
                to_removed_task_indices[to_removed_task_num] = prev_task_index;
                to_removed_task_num++;
            } else {
                to_removed_task_indices[to_removed_task_num] = prev_task_index;
                to_removed_task_num++;
                to_removed_task_indices[to_removed_task_num] = prev_task_index + 1;
                to_removed_task_num++;
            }
            visited_task_num++;
        }

        if (0 == adjacent_task_index % 2) {
            to_removed_task_indices[to_removed_task_num] = adjacent_task_index - 1;
            to_removed_task_num++;
            to_removed_task_indices[to_removed_task_num] = adjacent_task_index;
            to_removed_task_num++;
        } else {
            to_removed_task_indices[to_removed_task_num] = adjacent_task_index;
            to_removed_task_num++;
            to_removed_task_indices[to_removed_task_num] = adjacent_task_index + 1;
            to_removed_task_num++;
        }

        idx_t next_task_num = removed_string_cardinality - prev_task_index - 1;
        visited_task_num = 0;
        idx_t next_task_index = adjacent_task_index;
        while (0 != next_task_index) {
            if (visited_task_num >= next_task_num) {
                break;
            }
            next_task_index = sol.task_data_[next_task_index].next_task_index;
            if (0 == next_task_index % 2) {
                to_removed_task_indices[to_removed_task_num] = next_task_index - 1;
                to_removed_task_num++;
                to_removed_task_indices[to_removed_task_num] = next_task_index;
                to_removed_task_num++;
            } else {
                to_removed_task_indices[to_removed_task_num] = next_task_index;
                to_removed_task_num++;
                to_removed_task_indices[to_removed_task_num] = next_task_index + 1;
                to_removed_task_num++;
            }
            visited_task_num++;
        }

        for (idx_t i = 0; i < to_removed_task_num;) {
            sol.RemoveTask(adjacent_route_index, to_removed_task_indices[i], to_removed_task_indices[i + 1], false);
            i += 2;
        }

        if (sol.route_data_[adjacent_route_index].task_num > 0) {
            sol.UpdateRouteData(adjacent_route_index, sol.route_data_[adjacent_route_index].first_task,
                                sol.route_data_[adjacent_route_index].last_task);
        }

        ruined_routes.insert(adjacent_route_index);
        if ((idx_t)ruined_routes.size() >= string_num) {
            break;
        }
    }
}
}  // namespace lcs_srex