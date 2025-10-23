#include "algo/lcs_srex/route_subset.h"

#include <algorithm>
#include <random>

#include "util/random_utils.h"

namespace lcs_srex {
void RouteSubset::CalArcSequences(const SREXSol& sol, const std::vector<idx_t>& route_indices,
                                  std::vector<int>& arc_sequence, int& arc_sequence_length) {
    arc_sequence_length = 0;
    for (idx_t route_index : route_indices) {
        AddArcSequences(sol, route_index, arc_sequence, arc_sequence_length);
    }
}

void RouteSubset::AddArcSequences(const SREXSol& sol, idx_t route_index, std::vector<int>& arc_sequence,
                                  int& arc_sequence_length) {
    idx_t prev_task_index = 0;
    idx_t current_task_index = sol.route_data_[route_index].first_task;

    while (0 != current_task_index) {
        idx_t arc_hash = CalArcHashValue(prev_task_index, current_task_index);
        arc_sequence[arc_sequence_length] = arc_hash;
        arc_sequence_length += 1;

        prev_task_index = current_task_index;
        current_task_index = sol.task_data_[current_task_index].next_task_index;
    }

    arc_sequence[arc_sequence_length] = CalArcHashValue(prev_task_index, 0);
    arc_sequence_length += 1;
}

void RouteSubset::CalRouteIndicesHashValue(const std::vector<idx_t>& route_indices, double& hash_value) {
    hash_value = 0.0;
    for (idx_t route_index : route_indices) {
        hash_value += (route_index + 1.0) / kMaxTaskNum;
    }
}

void RouteSubset::CalArcSequences() {
    CalArcSequences(first_sol_, first_route_indices_, first_arc_sequence_, first_arc_sequence_length_);
    CalArcSequences(second_sol_, second_route_indices_, second_arc_sequence_, second_arc_sequence_length_);
}

void RouteSubset::CalLCSValues() {
    lcs_value_ =
        CalLCS(first_arc_sequence_, second_arc_sequence_, first_arc_sequence_length_, second_arc_sequence_length_,
               first_arc_sequence_length_ - 1, second_arc_sequence_length_ - 1, lcs_values_);
}

void RouteSubset::SelectRandomInitialRoutes() {
    int min_route_num = (int)std::min(first_sol_.route_data_.size(), second_sol_.route_data_.size());
    int select_route_num = GetRandInt(1, min_route_num);

    if (select_route_num == (int)first_sol_.route_data_.size()) {
        first_route_indices_.resize(select_route_num);
        std::iota(first_route_indices_.begin(), first_route_indices_.end(), 0);

        first_remaining_route_indices_.clear();
    } else {
        first_route_indices_.resize(first_sol_.route_data_.size());
        std::iota(first_route_indices_.begin(), first_route_indices_.end(), 0);
        std::shuffle(first_route_indices_.begin(), first_route_indices_.end(), GetRandomGenerator());

        first_remaining_route_indices_.assign(first_route_indices_.begin() + select_route_num,
                                              first_route_indices_.end());
        first_route_indices_.resize(select_route_num);
    }

    if (select_route_num == (int)second_sol_.route_data_.size()) {
        second_route_indices_.resize(select_route_num);
        std::iota(second_route_indices_.begin(), second_route_indices_.end(), 0);

        second_remaining_route_indices_.clear();
    } else {
        second_route_indices_.resize(second_sol_.route_data_.size());
        std::iota(second_route_indices_.begin(), second_route_indices_.end(), 0);
        std::shuffle(second_route_indices_.begin(), second_route_indices_.end(), GetRandomGenerator());

        second_remaining_route_indices_.assign(second_route_indices_.begin() + select_route_num,
                                               second_route_indices_.end());
        second_route_indices_.resize(select_route_num);
    }

    CalArcSequences();
    CalLCSValues();
}

void RouteSubset::SearchNeighborhood() {
    while (!first_remaining_route_indices_.empty() && !second_remaining_route_indices_.empty()) {
        idx_t old_first_arc_sequence_length = first_arc_sequence_length_;
        idx_t old_second_arc_sequence_length = second_arc_sequence_length_;

        int first_route_to_add = GetRandInt(0, (int)first_remaining_route_indices_.size() - 1);
        idx_t first_route_index_to_add = first_remaining_route_indices_[first_route_to_add];

        int second_route_to_added = GetRandInt(0, (int)second_remaining_route_indices_.size() - 1);
        idx_t second_route_index_to_add = second_remaining_route_indices_[second_route_to_added];

        AddArcSequences(first_sol_, first_route_to_add, first_arc_sequence_, first_arc_sequence_length_);
        AddArcSequences(second_sol_, second_route_to_added, second_arc_sequence_, second_arc_sequence_length_);

        int new_lcs_value = CalLCS(first_arc_sequence_, second_arc_sequence_, old_first_arc_sequence_length,
                                   old_second_arc_sequence_length, first_arc_sequence_length_ - 1,
                                   second_arc_sequence_length_ - 1, lcs_values_);
        if (new_lcs_value > lcs_value_) {
            lcs_value_ = new_lcs_value;
            first_route_indices_.push_back(first_route_index_to_add);
            second_route_indices_.push_back(second_route_index_to_add);

            first_remaining_route_indices_[first_route_to_add] = first_remaining_route_indices_.back();
            first_remaining_route_indices_.resize(first_remaining_route_indices_.size() - 1);

            second_remaining_route_indices_[second_route_to_added] = second_remaining_route_indices_.back();
            second_remaining_route_indices_.resize(second_remaining_route_indices_.size() - 1);
        } else {
            first_arc_sequence_length_ = old_first_arc_sequence_length;
            second_arc_sequence_length_ = old_second_arc_sequence_length;
            break;
        }
    }

    std::sort(first_route_indices_.begin(), first_route_indices_.end());
    std::sort(second_route_indices_.begin(), second_route_indices_.end());
    CalRouteIndicesHashValue(first_route_indices_, first_route_indices_hash_value_);
    CalRouteIndicesHashValue(second_route_indices_, second_route_indices_hash_value_);
}

bool RouteSubset::IsDuplicatedWith(const RouteSubset& other_route_subset) {
    if (first_route_indices_.size() != other_route_subset.first_route_indices_.size() ||
        second_route_indices_.size() != other_route_subset.second_route_indices_.size()) {
        return false;
    }
    if (first_route_indices_hash_value_ != other_route_subset.first_route_indices_hash_value_ ||
        second_route_indices_hash_value_ != other_route_subset.second_route_indices_hash_value_) {
        return false;
    }

    for (size_t i = 0; i < first_route_indices_.size(); ++i) {
        if (first_route_indices_[i] != other_route_subset.first_route_indices_[i]) {
            return false;
        }
    }

    for (size_t i = 0; i < second_route_indices_.size(); ++i) {
        if (second_route_indices_[i] != other_route_subset.second_route_indices_[i]) {
            return false;
        }
    }

    return true;
}
}  // namespace lcs_srex