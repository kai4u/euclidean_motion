#pragma once

#include <array>
#include <memory>
#include <optional>
#include <vector>

struct Motion {
  std::array<double, 2> translation;
  std::array<std::array<double, 2>, 2> rotation;
};

class Point {
 public:
  Point(double x, double y);

  Point& operator+=(const Point& rhs);
  Point operator-() const;
  Point& operator-=(const Point& rhs);
  Point operator-(const Point& rhs) const;

  double len() const;
  double angle() const;
  double x() const;
  double y() const;

 private:
  void invalidate_cache() const;

  double x_, y_;
  mutable std::optional<double> len_;
  mutable std::optional<double> angle_;
};

class EuclideanMotionSolver {
 public:
  using Points = std::vector<Point>;
  EuclideanMotionSolver() = default;
  ~EuclideanMotionSolver() = default;

  template <typename T>
  void train(T&& points) {
    src_points = std::make_unique<Points>(std::forward<T>(points));
  }

  template <typename T>
  std::optional<Motion> predict(T&& points) {
    dst_points = std::make_unique<Points>(std::forward<T>(points));
    return predict_impl();
  }

  static Points read_points(const std::string& path);

 private:
  std::optional<Motion> predict_impl();

  std::unique_ptr<Points> src_points;
  std::unique_ptr<Points> dst_points;
};
