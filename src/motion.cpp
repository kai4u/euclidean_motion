#include "motion.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <vector>

#include "utils.h"

namespace {

constexpr double EPS = 1e-9;

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

double unify_angle(double angle) {
  if (angle > M_PI + EPS) {
    return angle - 2 * M_PI;
  } else if (angle < -M_PI + EPS) {
    return angle + 2 * M_PI;
  }
  return angle;
}

std::vector<double> filter_by_radius(const EuclideanMotionSolver::Points& v,
                                     size_t i) {
  EuclideanMotionSolver::Points tmp;
  for (size_t j = i; j < v.size() && v[j].len() - v[i].len() < EPS; ++j) {
    tmp.push_back(v[j]);
  }
  auto cmp = [](const auto& lhs, const auto& rhs) {
    return lhs.angle() < rhs.angle();
  };
  std::sort(tmp.begin(), tmp.end(), cmp);

  std::vector<double> res;
  for (size_t j = 0; j < tmp.size(); ++j) {
    res.emplace_back(
        unify_angle(tmp[(j + 1) % tmp.size()].angle() - tmp[j].angle()));
  }

  return res;
}

}  // namespace

Point::Point(double x, double y) : x_(x), y_(y) {}

Point Point::operator+(const Point& rhs) const {
  Point res(*this);
  res.operator+=(rhs);
  return res;
}

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

std::string Point::ToString() const {
  std::stringstream ss;
  ss << "[" << x_ << ", " << y_ << "]";
  return ss.str();
}

void Point::invalidate_cache() const {
  angle_ = std::nullopt;
  len_ = std::nullopt;
}

Rotation::Rotation(double angle) : angle_(angle) {}

Point Rotation::apply(const Point& point) const {
  return {std::cos(angle_) * point.x() - std::sin(angle_) * point.y(),
          std::sin(angle_) * point.x() + std::cos(angle_) * point.y()};
}

double Rotation::angle() const {
  return angle_;
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

std::vector<Motion> EuclideanMotionSolver::predict_impl() {
  auto& src = *src_points;
  auto& dst = *dst_points;

  if (src.size() == 0 || src.size() != dst.size()) {
    return {};
  }

  const auto src_center = cloud_center(src);
  const auto dst_center = cloud_center(dst);

  move_to_center(src);
  move_to_center(dst);

  auto cmp = [](const auto& lhs, const auto& rhs) {
    return lhs.len() < rhs.len();
  };

  std::sort(src.begin(), src.end(), cmp);
  std::sort(dst.begin(), dst.end(), cmp);

  std::vector<double> possible_rotations;
  for (size_t i = 0; i < src.size();) {
    if (std::abs(src[i].len() - dst[i].len()) > EPS) {
      return {};
    }

    auto src_angles = filter_by_radius(src, i);
    auto dst_angles = filter_by_radius(dst, i);

    if (src_angles.size() != dst_angles.size()) {
      return {};
    }

    const size_t len = src_angles.size();

    const auto shifts = utils::find_all_cyclic_shifts(src_angles, dst_angles);
    if (shifts.empty()) {
      return {};
    }

    std::vector<double> rotations;
    rotations.reserve(shifts.size());
    for (const size_t j : shifts) {
      rotations.emplace_back(unify_angle(dst[i + j].angle() - src[i].angle()));
    }

    if (possible_rotations.empty()) {
      possible_rotations = std::move(rotations);
    } else {
      possible_rotations =
          utils::find_intersection(possible_rotations, rotations);
    }

    if (possible_rotations.empty()) {
      return {};
    }

    i += len;
  }

  std::cout << "Possible rotations count: " << possible_rotations.size()
            << std::endl;

  std::vector<Motion> res;
  for (const auto angle : possible_rotations) {
    Rotation rotation(angle);
    const auto translation = dst_center - rotation.apply(src_center);
    res.emplace_back(translation, rotation);
  }

  return res;
}
