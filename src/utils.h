#pragma once

#include <cstddef>
#include <vector>

namespace utils {

std::vector<size_t> calc_shifts(std::vector<double>&& src_angles,
                                std::vector<double>&& dst_angles);

void update_common_rotations(std::vector<double>& incremented_rotations,
                             const std::vector<double>& current_rotations);

}  // namespace utils
