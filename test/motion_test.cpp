#include "motion.h"

#include "gtest/gtest.h"

class MotionTest : public testing::Test {
 public:
  MotionTest() = default;
  ~MotionTest() override = default;
};

TEST(MotionTest, Simple) {
  EuclideanMotionSolver solver;
  const EuclideanMotionSolver::Points src{{1., 0.}, {0., 1.}};
  const EuclideanMotionSolver::Points dst{{-1., 0.}, {0., 1.}};

  solver.train(src);
  const auto res = solver.predict(dst);
  ASSERT_TRUE(res.has_value());
}
