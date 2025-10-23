#ifndef AMDAHL_SRC_ALGO_LCS_SREX_CROSSOVER_H_
#define AMDAHL_SRC_ALGO_LCS_SREX_CROSSOVER_H_

#include "algo/lcs_srex/route_subset.h"
#include "algo/lcs_srex/srex_sol.h"

namespace lcs_srex {
SREXSol Crossover(const SREXSol& first_sol, const SREXSol& second_sol, const RouteSubset& selective_route_subset);
}
#endif