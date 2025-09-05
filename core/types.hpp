#pragma once
#include "geom/point.hpp"
#include <cstddef>
#include <cstdint>
#include <limits>
struct alignas(64) TrackPoint {
  std::uint32_t id{0};
  double t{0.0};
  Point p;
  char _pad[64 - sizeof(std::uint32_t) - sizeof(double) - sizeof(Point)]{};
};
struct EntryResult {
  bool will_enter{false};
  double t_enter{std::numeric_limits<double>::quiet_NaN()};
  double t_exit{std::numeric_limits<double>::quiet_NaN()};
  Point p_enter{};
};
