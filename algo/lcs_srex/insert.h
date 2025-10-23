#ifndef AMDAHL_SRC_ALGO_LCS_SREX_INSERT_H_
#define AMDAHL_SRC_ALGO_LCS_SREX_INSERT_H_

#include "algo/lcs_srex/srex_sol.h"

namespace lcs_srex {

constexpr real_t kEmptyVehicleCost = 1E4;
constexpr real_t kSortBasedOnRandomProbability = 0.4;
constexpr real_t kSortBasedOnDemandProbability = 0.3;
constexpr real_t kSortBasedOnFarDistanceProbability = 0.2;
constexpr real_t kSortBasedOnCloseDistanceProbability = 0.1;
constexpr real_t kBlinkRate = 0.01;

struct InsertionData {
    idx_t route_index{-1};
    idx_t pickup_prev_task_index{-1};
    idx_t delivery_prev_task_index{-1};
    real_t delta_tw_penalty{0.0};
    demand_t delta_capacity_violations{0};
    real_t delta_distance{0.0};
    bool new_vehicle{false};
};

void CalInsertionData(const SREXSol& sol, idx_t pickup_task_index, idx_t delivery_task_index, idx_t route_index,
                      std::vector<InsertionData>& insertions, bool allow_infeasible);

void BestInsert(SREXSol& sol);

void GreedyInsertionWithBlinks(SREXSol& sol);
}  // namespace lcs_srex
#endif