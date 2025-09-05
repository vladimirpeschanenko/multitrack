#pragma once
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

/**
 *  @brief Incremental OLS for a 1D line  y = a + b*t
 *
 * Fits a straight line to streaming samples (t_i, y_i) using
 * Ordinary Least Squares (OLS) **incrementally** (no history kept).
 * Per update is O(1) time and O(1) memory.
 *
 */
class LinearFit1D {
  std::size_t n_ = 0;
  double sum_t_ = 0.0, sum_y_ = 0.0, sum_t2_ = 0.0, sum_ty_ = 0.0;

public:
  inline void reset() noexcept {
    n_ = 0;
    sum_t_ = sum_y_ = sum_t2_ = sum_ty_ = 0.0;
  }

  inline void add(double t, double y) noexcept {
    ++n_;
    sum_t_ += t;
    sum_y_ += y;
    sum_t2_ += t * t;
    sum_ty_ += t * y;
  }

  inline bool ready() const noexcept { return n_ >= 2; }

  // b = (n * sum_ty_ - sum_t_* sum_y_) / (n_ * sum_t2_ - sum_t_ * sum_t)
  // a = (sum_y_ - b * sum_t_) / n_
  // if denominator of b ≈ 0 (all t equal), fall back to constant fit b=0,
  // a=avg(y).
  inline std::pair<double, double> params() const noexcept {
    if (!ready())
      return {std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN()};
    const double denom = static_cast<double>(n_) * sum_t2_ - sum_t_ * sum_t_;
    if (std::fabs(denom) < 1e-12)
      return {sum_y_ / static_cast<double>(n_), 0.0};
    const double b =
        (static_cast<double>(n_) * sum_ty_ - sum_t_ * sum_y_) / denom;
    const double a = (sum_y_ - b * sum_t_) / static_cast<double>(n_);
    return {a, b};
  }
};
