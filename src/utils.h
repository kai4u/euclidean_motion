#pragma once

#include <cstddef>
#include <vector>

namespace utils {

std::vector<size_t> calc_shifts(std::vector<double>&& src_angles,
                                std::vector<double>&& dst_angles);

std::vector<double> common_rotations(std::vector<double>&& rotations,
                                     std::vector<double>&& current_rotations);

}  // namespace utils
