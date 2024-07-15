#include "motion.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <vector>

#include "utils.h"

namespace {

Point cloud_center(const std::vector<Point>& points) {
  long double x = 0, y = 0;
  for (const auto& point : points) {
    x += point.x();
    y += point.y();
  }
  x /= points.size();
  y /= points.size();

  return {static_cast<double>(x), static_cast<double>(y)};
}

void move_to_center(std::vector<Point>& points) {
  const auto center = cloud_center(points);
  for (auto& point : points) {
    point -= center;
  }
}

}  // namespace

Point::Point(double x, double y) : x_(x), y_(y) {}

Point& Point::operator+=(const Point& rhs) {
  this->x_ += rhs.x_;
  this->y_ += rhs.y_;
  invalidate_cache();
  return *this;
}

Point Point::operator-() const {
  return {-x_, -y_};
}

Point& Point::operator-=(const Point& rhs) {
  this->operator+=(-rhs);
  invalidate_cache();
  return *this;
}

Point Point::operator-(const Point& rhs) const {
  Point res(*this);
  res.operator-=(rhs);
  return res;
}

double Point::x() const {
  return x_;
}

double Point::y() const {
  return y_;
}

double Point::len() const {
  if (len_.has_value()) {
    return len_.value();
  }

  len_ = std::sqrt(x_ * x_ + y_ * y_);
  return len_.value();
}

double Point::angle() const {
  if (angle_.has_value()) {
    return angle_.value();
  }

  angle_ = std::atan2(y_, x_);
  return angle_.value();
}

void Point::invalidate_cache() const {
  angle_ = std::nullopt;
  len_ = std::nullopt;
}

// static
EuclideanMotionSolver::Points EuclideanMotionSolver::read_points(
    const std::string& path) {
  std::ifstream fs(path);

  int num;
  fs >> num;

  Points res;
  res.reserve(num);
  for (int i = 0; i < num; ++i) {
    double x, y;
    fs >> x >> y;
    res.emplace_back(x, y);
  }

  return res;
}

std::optional<Motion> EuclideanMotionSolver::predict_impl() {
  auto& src = *src_points;
  auto& dst = *dst_points;

  if (src.size() == 0 || src.size() != dst.size()) {
    return std::nullopt;
  }

  // TODO: fix huinya.
  const auto res_motion = cloud_center(dst) - cloud_center(src);

  move_to_center(src);
  move_to_center(dst);

  const double EPS = 1e-9;

  auto cmp = [&](const auto& lhs, const auto& rhs) {
    return lhs.len() < rhs.len() - EPS ||
           (lhs.len() < rhs.len() + EPS && (lhs.angle() < rhs.angle()));
  };

  std::sort(src.begin(), src.end(), cmp);
  std::sort(dst.begin(), dst.end(), cmp);

  std::vector<double> possible_rotations;
  for (size_t i = 0; i < src.size();) {
    std::vector<double> src_angles, dst_angles;

    // TODO: function + unify angles.
    for (size_t src_j = i + 1;
         src_j < src.size() && std::abs(src[src_j].len() - src[i].len()) < EPS;
         ++src_j) {
      src_angles.emplace_back(src[src_j].angle() - src[src_j - 1].angle());
    }
    src_angles.emplace_back(src[i].angle() -
                            src[i + src_angles.size()].angle());

    for (size_t dst_j = i + 1;
         dst_j < dst.size() && std::abs(dst[dst_j].len() - dst[i].len()) < EPS;
         ++dst_j) {
      dst_angles.emplace_back(dst[dst_j].angle() - dst[dst_j - 1].angle());
    }
    dst_angles.emplace_back(dst[i].angle() -
                            dst[i + dst_angles.size()].angle());

    if (src_angles.size() != dst_angles.size()) {
      return std::nullopt;
    }

    const size_t len = src_angles.size();

    const auto shifts =
        utils::calc_shifts(std::move(src_angles), std::move(dst_angles));
    if (shifts.empty()) {
      return std::nullopt;
    }

    std::vector<double> rotations;
    rotations.reserve(shifts.size());
    for (const size_t j : shifts) {
      rotations.emplace_back(dst[i + j].angle() - src[i].angle());
    }

    possible_rotations = utils::common_rotations(std::move(possible_rotations),
                                                 std::move(rotations));

    if (possible_rotations.empty()) {
      return std::nullopt;
    }

    i += len;
  }
  return Motion{{res_motion.x(), res_motion.y()},
                {std::array{possible_rotations[0], 0.}, std::array{0., 0.}}};
}
