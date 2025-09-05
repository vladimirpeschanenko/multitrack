#pragma once

// FS isn't a problem here, because other threads will not change this data.
struct /*alignas(16)*/ Point {
  double x{0.0};
  double y{0.0};
};
