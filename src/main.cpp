#include <iostream>

#include "motion.h"

namespace {

std::ostream& operator<<(std::ostream& os, const Motion& motion) {
  os << "Translation: ";
  auto print_array = [&os](const auto& array) {
    os << "[";
    for (size_t i = 0; i < array.size(); ++i) {
      os << array[i] << (i + 1 == array.size() ? "" : ", ");
    }
    os << "]";
  };

  print_array(motion.translation);

  os << "\nRotation: [";
  for (size_t i = 0; i < motion.rotation.size(); ++i) {
    print_array(motion.rotation[i]);
    if (i + 1 < motion.rotation.size()) {
      os << ", ";
    }
  }
  os << "]\n";
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

  if (!res.has_value()) {
    std::cout << "Ain't no motion between these sets of points." << std::endl;
    ;
  } else {
    std::cout << res.value();
  }

  return 0;
}
