#include "utils.h"

#include "gtest/gtest.h"

TEST(CalcShifts, Single) {
  std::vector<double> l{1., 2., 3.};
  std::vector<double> r{2., 3., 1.};
  const auto shifts = utils::calc_shifts(std::move(l), std::move(r));
  ASSERT_EQ(shifts.size(), 1u);
  EXPECT_EQ(shifts[0], 2u);
}

TEST(CalcShifts, AnySingle) {
  std::vector<double> l{1., 2., 3.};
  std::vector<double> r{2., 3., 10.};
  const auto shifts = utils::calc_shifts(std::move(l), std::move(r));
  EXPECT_EQ(shifts.size(), 0u);
}

TEST(CalcShifts, Two) {
  std::vector<double> l{1., 2., 1., 2.};
  std::vector<double> r{2., 1., 2., 1.};
  const auto shifts = utils::calc_shifts(std::move(l), std::move(r));
  ASSERT_EQ(shifts.size(), 2u);
  EXPECT_EQ(shifts[0], 1u);
  EXPECT_EQ(shifts[1], 3u);
}

TEST(CalcShifts, All) {
  std::vector<double> l{1., 1., 1., 1.};
  std::vector<double> r{1., 1., 1., 1.};
  const auto shifts = utils::calc_shifts(std::move(l), std::move(r));
  ASSERT_EQ(shifts.size(), 4u);
  for (size_t i = 0; i < shifts.size(); ++i) {
    EXPECT_EQ(shifts[i], i) << "For i = " << i;
  }
}

TEST(CommonRotations, FirstEmpty) {
  std::vector<double> inc{};
  std::vector<double> curr{1., 2.};
  utils::update_common_rotations(inc, curr);
  ASSERT_EQ(inc.size(), curr.size());
  for (size_t i = 0; i < inc.size(); ++i) {
    EXPECT_EQ(inc[i], curr[i]) << "For i = " << i;
  }
}

TEST(CommonRotations, SecondEmpty) {
  std::vector<double> inc{1., 2.};
  std::vector<double> curr{};
  utils::update_common_rotations(inc, curr);
  EXPECT_TRUE(inc.empty());
}

TEST(CommonRotations, HasIntersection) {
  std::vector<double> inc{1., 2., 1.5};
  std::vector<double> curr{3., 1.5, 0.};
  utils::update_common_rotations(inc, curr);
  ASSERT_EQ(inc.size(), 1u);
  EXPECT_EQ(inc[0], 1.5);
}

TEST(CommonRotations, NoIntersection) {
  std::vector<double> inc{1., 2., 1.5};
  std::vector<double> curr{3., 2.5, 0.};
  utils::update_common_rotations(inc, curr);
  EXPECT_TRUE(inc.empty());
}
