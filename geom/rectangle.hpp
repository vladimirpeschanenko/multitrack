#pragma once
#include "point.hpp"
// FS isn't a problem here, because other threads will not change this data.
struct /*alignas(32)*/ Rectangle {
  double xmin{0.0}, xmax{0.0}, ymin{0.0}, ymax{0.0};
  inline bool contains(const Point &p) const noexcept {
    return p.x >= xmin && p.x <= xmax && p.y >= ymin && p.y <= ymax;
  }
};
