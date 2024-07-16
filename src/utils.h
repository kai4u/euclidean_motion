#pragma once

#include <cstddef>
#include <vector>

namespace utils {

std::vector<size_t> find_all_cyclic_shifts(
    const std::vector<double>& src_angles,
    const std::vector<double>& dst_angles);

std::vector<double> find_intersection(
    const std::vector<double>& incremented_rotations,
    const std::vector<double>& current_rotations);

}  // namespace utils
