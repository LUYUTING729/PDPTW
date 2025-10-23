#ifndef AMDAHL_SRC_ALGO_LCS_SREX_REMOVAL_H_
#define AMDAHL_SRC_ALGO_LCS_SREX_REMOVAL_H_

#include "algo/lcs_srex/srex_sol.h"
#include "util/random_utils.h"

namespace lcs_srex {

constexpr idx_t kRemovedStringCardinalityLimit = 10;
constexpr idx_t kAverageRemovedTasks = 10;

static inline double GetMaxStringCardinality(double avg_task_num) {
    return std::min((double)kRemovedStringCardinalityLimit, avg_task_num);
}

static inline double GetMaxStringNum(double max_string_cardinality) {
    return 4.0 * kAverageRemovedTasks / (1 + max_string_cardinality) - 1.0;
}

static inline idx_t GetStringNum(double max_string_num) { return std::floor(GetRandDouble(1.0, max_string_num + 1.0)); }

static inline double GetMaxRouteStringCardinality(double max_string_cardinality, idx_t route_cardinality) {
    return std::min(max_string_cardinality, (double)route_cardinality);
}

static inline idx_t GetRemovedStringCardinality(double max_route_string_cardinality) {
    return std::floor(GetRandDouble(1.0, max_route_string_cardinality + 1.0));
}

void StringRemoval(SREXSol& sol);

}  // namespace lcs_srex

#endif