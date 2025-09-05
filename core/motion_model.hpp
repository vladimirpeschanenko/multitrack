#pragma once
#include "core/linear_fit.hpp"
#include "core/types.hpp"
#include <cmath>
#include <numbers>

/**
 * @brief Online constant-velocity motion model in 2D.
 *
 * Estimates the parametric trajectory x(t) = a_x + b_x*t, y(t) = a_y + b_y*t
 * from streaming detections (t_i, x_i, y_i) using two independent
 * incremental OLS fits (see @a LinearFit1D).
 *
 * Terminology:
 *  - b_x, b_y are the velocity components.
 *  - Speed is sqrt(b_x^2+b_y^2).
 *  - Heading (deg) atan(b_y,b_x) * 180 / pi (from +X axis, counter-clockwise).
 *
 * Not thread-safe; keep one instance per worker/thread.
 * Callers should check ready() before querying parameters/predictions.
 */
class MotionModel {
  LinearFit1D fx_, fy_;

public:
  inline void add(const TrackPoint &s) noexcept {
    fx_.add(s.t, s.p.x);
    fy_.add(s.t, s.p.y);
  }
  inline bool ready() const noexcept { return fx_.ready() && fy_.ready(); }
  inline void params(double &ax, double &bx, double &ay,
                     double &by) const noexcept {
    auto px = fx_.params();
    auto py = fy_.params();
    ax = px.first;
    bx = px.second;
    ay = py.first;
    by = py.second;
  }
  inline void velocity(double &vx, double &vy) const noexcept {
    double ax, bx, ay, by;
    params(ax, bx, ay, by);
    vx = bx;
    vy = by;
  }
  inline double speed() const noexcept {
    double vx, vy;
    velocity(vx, vy);
    return std::hypot(vx, vy);
  }
  inline double headingDeg() const noexcept {
    double vx, vy;
    velocity(vx, vy);
    return std::atan2(vy, vx) * 180.0 / std::numbers::pi;
  }
  inline void positionAt(double t, Point &out) const noexcept {
    double ax, bx, ay, by;
    params(ax, bx, ay, by);
    out.x = ax + bx * t;
    out.y = ay + by * t;
  }
};
