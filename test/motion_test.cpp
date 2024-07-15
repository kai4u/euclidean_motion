#include "motion.h"

#include <cmath>
#include <random>

#include "gtest/gtest.h"

class MotionTest : public testing::Test {
 public:
  MotionTest() = default;
  ~MotionTest() override = default;
};

TEST(MotionTest, AnySingle) {
  EuclideanMotionSolver solver;
  const EuclideanMotionSolver::Points src{{1., 0.}, {1., 1.}};
  const EuclideanMotionSolver::Points dst{{-1., 0.}};

  solver.train(src);
  const auto res = solver.predict(dst);
  EXPECT_TRUE(res.empty());
}

TEST(MotionTest, AnySingle2) {
  EuclideanMotionSolver solver;
  const EuclideanMotionSolver::Points src{{1., 0.}, {1., 1.}};
  const EuclideanMotionSolver::Points dst{{-1., 0.}, {1., 0.}};

  solver.train(src);
  const auto res = solver.predict(dst);
  EXPECT_TRUE(res.empty());
}

TEST(MotionTest, Single) {
  EuclideanMotionSolver solver;
  const EuclideanMotionSolver::Points src{{1., 0.}};
  const EuclideanMotionSolver::Points dst{{-1., 0.}};

  solver.train(src);
  const auto res = solver.predict(dst);
  ASSERT_EQ(res.size(), 1u);

  const auto& translation = res[0].translation;
  EXPECT_TRUE(std::abs(translation.x() + 2) < 1e-9) << translation.x();
  EXPECT_TRUE(std::abs(translation.y()) < 1e-9) << translation.y();

  EXPECT_TRUE(std::abs(res[0].rotation.angle()) < 1e-9)
      << res[0].rotation.angle();
}

TEST(MotionTest, Simple) {
  EuclideanMotionSolver solver;
  const EuclideanMotionSolver::Points src{{1., 0.}, {0., 1.}};
  const EuclideanMotionSolver::Points dst{{-1., 0.}, {0., 1.}};

  solver.train(src);
  const auto res = solver.predict(dst);
  ASSERT_EQ(res.size(), 2u);

  {
    const auto& translation = res[0].translation;
    EXPECT_TRUE(std::abs(translation.x() + 1) < 1e-9) << translation.x();
    EXPECT_TRUE(std::abs(translation.y() - 1) < 1e-9) << translation.y();

    EXPECT_TRUE(std::abs(res[0].rotation.angle() + M_PI_2) < 1e-9)
        << res[0].rotation.angle();
  }

  {
    const auto& translation = res[1].translation;
    EXPECT_TRUE(std::abs(translation.x()) < 1e-9) << translation.x();
    EXPECT_TRUE(std::abs(translation.y()) < 1e-9) << translation.y();

    EXPECT_TRUE(std::abs(res[1].rotation.angle() - M_PI_2) < 1e-9)
        << res[1].rotation.angle();
  }
}

TEST(MotionTest, MatureTest) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-10.0, 10.0);

  EuclideanMotionSolver::Points src;
  for (size_t i = 0; i < 100; ++i) {
    src.emplace_back(dis(gen), dis(gen));
  }

  EuclideanMotionSolver::Points dst;

  //   constexpr double angle = 0;
  constexpr double angle = M_PI * (-0.627);
  constexpr double x = 4.4324;
  constexpr double y = -100.434254;

  const Rotation r(angle);
  for (size_t i = 0; i < src.size(); ++i) {
    auto p = r.apply(src[i]);
    p += Point{x, y};
    dst.push_back(p);
  }

  std::shuffle(dst.begin(), dst.end(), gen);

  EuclideanMotionSolver solver;
  solver.train(src);
  const auto res = solver.predict(dst);
  ASSERT_EQ(res.size(), 1u);

  const auto& translation = res[0].translation;
  EXPECT_TRUE(std::abs(translation.x() - x) < 1e-9) << translation.x();
  EXPECT_TRUE(std::abs(translation.y() - y) < 1e-9) << translation.y();

  EXPECT_TRUE(std::abs(res[0].rotation.angle() - angle) < 1e-9)
      << res[0].rotation.angle();
}

TEST(Rotation, Simple) {
  Rotation rot(M_PI_2);
  const auto res = rot.apply({1., 1.});

  EXPECT_TRUE(std::abs(res.x() + 1) < 1e-9);
  EXPECT_TRUE(std::abs(res.y() - 1) < 1e-9);
}
