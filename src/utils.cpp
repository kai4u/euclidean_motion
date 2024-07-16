#include "utils.h"

#include <cmath>

namespace {

constexpr double EPS = 1e-9;

}

namespace utils {

std::vector<size_t> find_all_cyclic_shifts(const std::vector<double>& src,
                                           const std::vector<double>& dst) {
  // TODO: make template with cmp, use z-function.
  std::vector<size_t> res;

  if (src.size() != dst.size()) {
    return res;
  }

  for (size_t i = 0; i < dst.size(); ++i) {
    size_t j = 0;
    while (j < src.size() &&
           std::abs(src[j] - dst[(i + j) % dst.size()]) < EPS) {
      j++;
    }
    if (j == src.size()) {
      res.emplace_back(i);
    }
  }
  return res;
}

std::vector<double> find_intersection(const std::vector<double>& incremented,
                                      const std::vector<double>& current) {
  // TODO: make template with cmp, pass sorted. two pointers
  std::vector<double> res;
  for (const auto& inc : incremented) {
    for (const auto& current : current) {
      if (std::abs(inc - current) < EPS) {
        res.emplace_back(inc);
      }
    }
  }
  return res;
}

}  // namespace utils
