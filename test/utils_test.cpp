#include "utils.h"

#include "gtest/gtest.h"

TEST(CalcShifts, Single) {
  const std::vector<double> l{1., 2., 3.};
  const std::vector<double> r{2., 3., 1.};
  const auto shifts = utils::find_all_cyclic_shifts(l, r);
  ASSERT_EQ(shifts.size(), 1u);
  EXPECT_EQ(shifts[0], 2u);
}

TEST(CalcShifts, AnySingle) {
  const std::vector<double> l{1., 2., 3.};
  const std::vector<double> r{2., 3., 10.};
  const auto shifts = utils::find_all_cyclic_shifts(l, r);
  EXPECT_EQ(shifts.size(), 0u);
}

TEST(CalcShifts, Two) {
  const std::vector<double> l{1., 2., 1., 2.};
  const std::vector<double> r{2., 1., 2., 1.};
  const auto shifts = utils::find_all_cyclic_shifts(l, r);
  ASSERT_EQ(shifts.size(), 2u);
  EXPECT_EQ(shifts[0], 1u);
  EXPECT_EQ(shifts[1], 3u);
}

TEST(CalcShifts, All) {
  const std::vector<double> l{1., 1., 1., 1.};
  const std::vector<double> r{1., 1., 1., 1.};
  const auto shifts = utils::find_all_cyclic_shifts(l, r);
  ASSERT_EQ(shifts.size(), 4u);
  for (size_t i = 0; i < shifts.size(); ++i) {
    EXPECT_EQ(shifts[i], i) << "For i = " << i;
  }
}

TEST(CommonRotations, FirstEmpty) {
  const std::vector<double> inc{};
  const std::vector<double> curr{1., 2.};
  const auto res = utils::find_intersection(inc, curr);
  EXPECT_TRUE(res.empty());
}

TEST(CommonRotations, SecondEmpty) {
  const std::vector<double> inc{1., 2.};
  const std::vector<double> curr{};
  const auto res = utils::find_intersection(inc, curr);
  EXPECT_TRUE(res.empty());
}

TEST(CommonRotations, HasIntersection) {
  const std::vector<double> inc{1., 2., 1.5};
  const std::vector<double> curr{3., 1.5, 0.};
  const auto res = utils::find_intersection(inc, curr);
  ASSERT_EQ(res.size(), 1u);
  EXPECT_EQ(res[0], 1.5);
}

TEST(CommonRotations, NoIntersection) {
  const std::vector<double> inc{1., 2., 1.5};
  const std::vector<double> curr{3., 2.5, 0.};
  const auto res = utils::find_intersection(inc, curr);
  EXPECT_TRUE(res.empty());
}
