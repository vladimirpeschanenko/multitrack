#pragma once
#include "core/motion_model.hpp"
#include "core/types.hpp"
#include "geom/rectangle.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

/**
 * @brief Time-window intersection for constant-velocity motion vs an
 * axis-aligned rectangle.
 *
 * The point follows a parametric, constant-velocity model from MotionModel:
 *   x(t) = a_x + b_x t ,  y(t) = a_y + b_y t .
 *
 * For an axis-aligned rectangle R = [xmin, xmax] × [ymin, ymax] and a lower
 * time bound tmin, Intersector computes:
 *   - the earliest time t_enter ≥ tmin the trajectory is inside R,
 *   - the last time t_exit ≥ t_enter it remains inside,
 *   - the entry point p_enter = (x(t_enter), y(t_enter)).
 *
 * Math (per axis): solve the inequality lo ≤ a + b t ≤ hi.
 *   • If |b| ~ 0 (no motion on that axis): interval is (-∞, +∞) when a∈[lo,hi],
 *     otherwise empty (+∞, -∞).
 *   • Else: t1 = (lo - a)/b, t2 = (hi - a)/b, ordered so t1 ≤ t2 → valid t ∈
 * [t1, t2].
 *
 * Combine axes by intersecting x- and y-intervals and clipping with [tmin, +∞):
 *   enter = max(tx_lo, ty_lo, tmin),  exit = min(tx_hi, ty_hi).
 * If enter ≤ exit → intersection exists and entry is at t = enter.
 */
class Intersector {
  /**
   * @brief Valid time interval on one axis for lo ≤ a + b t ≤ hi.
   * @param a  Intercept on the axis (position at t=0).
   * @param b  Slope on the axis (velocity component).
   * @param lo Lower bound of the rectangle on this axis.
   * @param hi Upper bound of the rectangle on this axis.
   * @return (t_lo, t_hi):
   *         - If moving (|b| >= eps): ordered times where the line
   * enters/leaves [lo,hi].
   *         - If static (|b| < eps) and a ∈ [lo,hi]: (-∞, +∞)  (always valid).
   *         - If static and a ∉ [lo,hi]: (+∞, -∞)  (empty interval).
   */
  static inline std::pair<double, double>
  axisInterval(double a, double b, double lo, double hi) noexcept {
    if (std::fabs(b) < 1e-12) {
      if (a >= lo && a <= hi)
        return {-std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()};
      return {std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity()};
    }
    double t1 = (lo - a) / b, t2 = (hi - a) / b;
    if (t1 > t2)
      std::swap(t1, t2);
    return {t1, t2};
  }

public:
  /**
   * @brief Earliest entry of the motion path into a rectangle after (or at)
   * tmin.
   *
   * @param m     MotionModel providing x(t), y(t) = a + b t.
   * @param r     Axis-aligned rectangle (inclusive bounds).
   * @param tmin  Do not report entries earlier than this time (e.g., “now”).
   * @return EntryResult with:
   *         - will_enter: true iff the intersection interval is non-empty,
   *         - t_enter: earliest time ≥ tmin inside r,
   *         - t_exit : last time still inside r,
   *         - p_enter: entry coordinates at t_enter.
   */
  static inline EntryResult firstEntry(const MotionModel &m, const Rectangle &r,
                                       double tmin) noexcept {
    double ax, bx, ay, by;
    m.params(ax, bx, ay, by);
    auto ix = axisInterval(ax, bx, r.xmin, r.xmax);
    auto iy = axisInterval(ay, by, r.ymin, r.ymax);
    const double enter = std::max({ix.first, iy.first, tmin});
    const double exit = std::min(ix.second, iy.second);
    EntryResult out;
    out.will_enter = (enter <= exit);
    if (out.will_enter) {
      out.t_enter = enter;
      out.t_exit = exit;
      m.positionAt(enter, out.p_enter);
    }
    return out;
  }
};
