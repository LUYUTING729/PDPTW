#ifndef AMDAHL_SRC_ALGO_LCS_SREX_LCS_DP_H_
#define AMDAHL_SRC_ALGO_LCS_SREX_LCS_DP_H_

#include <vector>

#include "common/constants.h"

namespace lcs_srex {
constexpr int kMaxArcSeqLength = kMaxTaskNum + kMaxVehicleNum;
constexpr int kInitialLCSValue = -1;

static inline int CalArcHashValue(int head, int tail) {
    if (head >= tail) {
        return head * head + head + tail;
    } else {
        return head + tail * tail;
    }
}

int CalLCS(const std::vector<int>& first_arc_seq, const std::vector<int>& second_arc_seq,
           int first_valid_lcs_value_length, int second_valid_lcs_value_length, int first_arc_seq_index,
           int second_arc_seq_index, std::vector<int>& lcs_values);
}  // namespace lcs_srex
#endif