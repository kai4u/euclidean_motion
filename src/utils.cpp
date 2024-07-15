#include "utils.h"

#include <cmath>

namespace utils {
std::vector<size_t> calc_shifts(std::vector<double>&& src_angles,
                                std::vector<double>&& dst_angles) {
  std::vector<size_t> res;

  for (size_t i = 0, len = dst_angles.size(); i < len; ++i) {
    dst_angles.emplace_back(dst_angles[i]);
  }

  for (size_t i = 0; i + src_angles.size() < dst_angles.size(); ++i) {
    size_t j = 0;
    for (; std::abs(src_angles[j] - dst_angles[i + j]) < 1e-9; j++)
      ;
    if (j == src_angles.size()) {
      res.emplace_back(i);
    }
  }
  return res;
}

void update_common_rotations(std::vector<double>& incremented_rotations,
                             const std::vector<double>& current_rotations) {
  if (incremented_rotations.empty()) {
    incremented_rotations = current_rotations;
    return;
  }

  std::vector<double> res;
  for (const auto& rotation : incremented_rotations) {
    for (const auto& current : current_rotations) {
      if (std::abs(rotation - current) < 1e-9) {
        res.emplace_back(rotation);
      }
    }
  }
  incremented_rotations = std::move(res);
}

}  // namespace utils
