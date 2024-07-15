#include <iostream>

#include "motion.h"

namespace {

std::ostream& operator<<(std::ostream& os, const Motion& motion) {
  os << "Translation: " << motion.translation.ToString() << std::endl;
  os << "Rotation: " << motion.rotation.angle() << std::endl;

  return os;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "You are expected to pass exactly 2 parameters to call main.";
    return 0;
  }

  auto src_set = EuclideanMotionSolver::read_points(argv[1]);
  auto dst_set = EuclideanMotionSolver::read_points(argv[2]);

  auto solver = EuclideanMotionSolver();
  solver.train(src_set);
  auto res = solver.predict(std::move(dst_set));

  if (res.empty()) {
    std::cout << "Ain't no motion between these sets of points." << std::endl;
  } else {
    for (const auto& motion : res) {
      std::cout << motion << std::endl;
    }
  }

  return 0;
}
