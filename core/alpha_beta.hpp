#pragma once
#include "core/types.hpp"
#include "geom/point.hpp"
#include <cmath>
#include <limits>

/**
 * @brief α–β (g–h) filter for 2D constant-velocity tracking.
 *
 * State at the last update time t_prev_:
 *   position (x_, y_), velocity (vx_, vy_)
 *
 * Update on sample (t, x, y):
 *   Predict:   x̂ = x_ + vx_ * dt,   ŷ = y_ + vy_ * dt
 *   Residual:  r = z - x̂
 *   Correct:   x_ ← x̂ + α r,        vx_ ← vx_ + (β/dt) r   (and same for y)
 *
 * Notes:
 * - O(1) per update, no heap allocs.
 * - Good for low latency; α controls smoothing, β controls responsiveness.
 * - Units follow your inputs (x,y units per t units).
 */
struct AlphaBeta {
  // Gains (tune per noise/jerk; typical: alpha∈[0.5,0.8], beta∈[0.2,0.35])
  double alpha{0.6};
  double beta{0.25};

  // Filter state
  bool have_{false};
  double t_prev_{0.0};
  double x_{0.0}, y_{0.0};
  double vx_{0.0}, vy_{0.0};

  // --- API (matches MotionModel & KalmanCV2D) -----------------------------

  inline void reset() noexcept {
    have_ = false;
    t_prev_ = 0.0;
    x_ = y_ = vx_ = vy_ = 0.0;
  }

  inline void set_gains(double a, double b) noexcept {
    alpha = a;
    beta = b;
  }

  inline void add(const TrackPoint &s) noexcept {
    if (!have_) {
      x_ = s.p.x;
      y_ = s.p.y;
      vx_ = 0.0;
      vy_ = 0.0;
      t_prev_ = s.t;
      have_ = true;
      return;
    }
    double dt = s.t - t_prev_;
    if (dt <= 0.0)
      dt = 1e-9; // guard against zero/negative dt

    // Predict
    const double x_pred = x_ + vx_ * dt;
    const double y_pred = y_ + vy_ * dt;

    // Innovation
    const double rx = s.p.x - x_pred;
    const double ry = s.p.y - y_pred;

    // Correct
    x_ = x_pred + alpha * rx;
    y_ = y_pred + alpha * ry;
    vx_ = vx_ + (beta / dt) * rx;
    vy_ = vy_ + (beta / dt) * ry;

    t_prev_ = s.t;
  }

  inline bool ready() const noexcept { return have_; }

  inline void velocity(double &vx, double &vy) const noexcept {
    vx = vx_;
    vy = vy_;
  }

  inline double speed() const noexcept { return std::hypot(vx_, vy_); }

  inline double headingDeg() const noexcept {
    return std::atan2(vy_, vx_) * 180.0 / 3.14159265358979323846;
  }

  // Provide ax,bx,ay,by such that x(t)=ax+bx*t, y(t)=ay+by*t
  inline void params(double &ax, double &bx, double &ay,
                     double &by) const noexcept {
    if (!have_) {
      const double NaN = std::numeric_limits<double>::quiet_NaN();
      ax = ay = bx = by = NaN;
      return;
    }
    bx = vx_;
    by = vy_;
    ax = x_ - vx_ * t_prev_;
    ay = y_ - vy_ * t_prev_;
  }

  // Predict position at arbitrary time t (linear CV propagation)
  inline void positionAt(double t, Point &out) const noexcept {
    const double dt = t - t_prev_;
    out.x = x_ + vx_ * dt;
    out.y = y_ + vy_ * dt;
  }
};
