#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class Point {
 public:
  Point() = default;
  Point(double x, double y);

  Point operator+(const Point& rhs) const;
  Point& operator+=(const Point& rhs);
  Point operator-() const;
  Point& operator-=(const Point& rhs);
  Point operator-(const Point& rhs) const;

  double len() const;
  double angle() const;
  double x() const;
  double y() const;

  std::string ToString() const;

 private:
  void invalidate_cache() const;

  double x_, y_;
  mutable std::optional<double> len_;
  mutable std::optional<double> angle_;
};

class Rotation {
 public:
  Rotation() = default;
  Rotation(double angle);
  Point apply(const Point& point) const;
  double angle() const;

 private:
  double angle_;
};

struct Motion {
  Motion() = default;
  Motion(const Point& p, const Rotation& r) : translation(p), rotation(r) {}
  Point translation;
  Rotation rotation;
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
  std::vector<Motion> predict(T&& points) {
    dst_points = std::make_unique<Points>(std::forward<T>(points));
    return predict_impl();
  }

  static Points read_points(const std::string& path);

 private:
  std::vector<Motion> predict_impl();

  std::unique_ptr<Points> src_points;
  std::unique_ptr<Points> dst_points;
};
