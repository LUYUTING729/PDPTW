#ifndef AMDAHL_SRC_ALGO_LCS_SREX_ROUTE_SUBSET_H_
#define AMDAHL_SRC_ALGO_LCS_SREX_ROUTE_SUBSET_H_

#include "algo/lcs_srex/lcs_dp.h"
#include "algo/lcs_srex/srex_sol.h"

namespace lcs_srex {
class RouteSubset {
   public:
    RouteSubset(const SREXSol& first_sol, const SREXSol& second_sol) : first_sol_(first_sol), second_sol_(second_sol) {
        first_arc_sequence_.resize(kMaxArcSeqLength, -1);
        second_arc_sequence_.resize(kMaxArcSeqLength, -1);

        first_arc_sequence_length_ = 0;
        second_arc_sequence_length_ = 0;
        lcs_values_.resize(kMaxArcSeqLength * kMaxArcSeqLength, kInitialLCSValue);
        lcs_value_ = 0;
    }
    const SREXSol& first_sol_;
    const SREXSol& second_sol_;

    std::vector<int> first_arc_sequence_;
    std::vector<int> second_arc_sequence_;

    idx_t first_arc_sequence_length_{0};
    idx_t second_arc_sequence_length_{0};

    std::vector<int> lcs_values_;
    int lcs_value_{0};

    std::vector<idx_t> first_route_indices_;
    std::vector<idx_t> second_route_indices_;
    std::vector<idx_t> first_remaining_route_indices_;
    std::vector<idx_t> second_remaining_route_indices_;

    double first_route_indices_hash_value_{kDoubleMax};
    double second_route_indices_hash_value_{kDoubleMax};

    void SelectRandomInitialRoutes();

    void CalArcSequences();
    static void CalArcSequences(const SREXSol& sol, const std::vector<idx_t>& route_indices,
                                std::vector<int>& arc_sequence, int& arc_sequence_length);
    void CalLCSValues();

    void SearchNeighborhood();

    static void AddArcSequences(const SREXSol& sol, idx_t route_index, std::vector<int>& arc_sequence,
                                int& arc_sequence_length);
    static void CalRouteIndicesHashValue(const std::vector<idx_t>& route_indices, double& hash_value);

    bool IsDuplicatedWith(const RouteSubset& other_route_subset);
};
}  // namespace lcs_srex

#endif