#pragma once
#include "core/types.hpp"
#include "geom/point.hpp"
#include <algorithm>
#include <cmath>

/**
 * @brief 2D Constant-Velocity Kalman filter (state x=[x,y,vx,vy]^T).
 *
 * Model:
 *   x_k+1 = F(dt) x_k + w,     z_k = H x_k + v
 *   F(dt) = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
 *   H     = [[1,0,0,0],[0,1,0,0]]
 * Process noise (white acceleration) block-diagonal per axis:
 *   Q(dt,q) = q * [[dt^4/4, 0,       dt^3/2, 0      ],
 *                  [0,       dt^4/4, 0,       dt^3/2],
 *                  [dt^3/2,  0,       dt^2,   0      ],
 *                  [0,       dt^3/2,  0,       dt^2  ]]
 * Measurement noise:
 *   R = r2 * I2
 *
 * All operations are O(1), no dynamic allocation. Good for low-latency paths.
 */
class KalmanCV2D {
  // Tunables (you may tweak at runtime)
  double meas_var{1.0};   // r^2 — measurement variance (x,y)
  double accel_var{1.0};  // q   — acceleration PSD (drives responsiveness)
  double vel_var0{100.0}; // initial velocity variance

  // Filter state
  bool have_{false};
  double t_prev_{0.0};
  // x = [x,y,vx,vy]^T
  double x_{0.0}, y_{0.0}, vx_{0.0}, vy_{0.0};
  // P (symmetric 4x4) stored as full matrix (unrolled)
  double P_[4][4]{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

public:
  // Match MotionModel API: x(t) = ax + bx*t, y(t) = ay + by*t
  inline void params(double &ax, double &bx, double &ay,
                     double &by) const noexcept {
    if (!have_) {
      ax = ay = bx = by = std::numeric_limits<double>::quiet_NaN();
      return;
    }
    bx = vx_;
    by = vy_;
    // We have state at t_prev_: x(t_prev_) = x_, so ax = x_ - vx_*t_prev_
    ax = x_ - vx_ * t_prev_;
    ay = y_ - vy_ * t_prev_;
  }
  // --- API compatible with MotionModel ------------------------------------
  inline void add(const TrackPoint &s) noexcept {
    if (!have_) {
      // Initialize state with first measurement; zero velocity.
      x_ = s.p.x;
      y_ = s.p.y;
      vx_ = 0.0;
      vy_ = 0.0;
      t_prev_ = s.t;
      // P: high uncertainty on velocity, moderate on position
      const double ppos = std::max(1.0, meas_var);
      P_[0][0] = ppos;
      P_[1][1] = ppos;
      P_[2][2] = vel_var0;
      P_[3][3] = vel_var0;
      P_[0][1] = P_[1][0] = 0;
      P_[0][2] = P_[2][0] = 0;
      P_[0][3] = P_[3][0] = 0;
      P_[1][2] = P_[2][1] = 0;
      P_[1][3] = P_[3][1] = 0;
      P_[2][3] = P_[3][2] = 0;
      have_ = true;
      return;
    }

    // Predict to time s.t
    double dt = s.t - t_prev_;
    if (dt < 0)
      dt = 0; // guard out-of-order
    if (dt == 0)
      dt = 1e-9; // keep Q well-behaved
    predict(dt);

    // Update with measurement z=[x,y]
    update(s.p.x, s.p.y);

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

  // Predict position at arbitrary time t (no covariance propagation here)
  inline void positionAt(double t, Point &out) const noexcept {
    double dt = t - t_prev_;
    out.x = x_ + vx_ * dt;
    out.y = y_ + vy_ * dt;
  }

  // Optional: adjust noises at runtime
  inline void set_noises(double r2, double q, double v0 = 100.0) noexcept {
    meas_var = r2;
    accel_var = q;
    vel_var0 = v0;
  }

private:
  // x = F x ; P = FPF^T + Q
  inline void predict(double dt) noexcept {
    // State
    x_ = x_ + vx_ * dt;
    y_ = y_ + vy_ * dt;
    // Covariance: P = F P F^T + Q
    // Unroll for F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    const double q = accel_var;

    // Compute A = F*P
    double A00 = P_[0][0] + dt * P_[2][0];
    double A01 = P_[0][1] + dt * P_[2][1];
    double A02 = P_[0][2] + dt * P_[2][2];
    double A03 = P_[0][3] + dt * P_[2][3];

    double A10 = P_[1][0] + dt * P_[3][0];
    double A11 = P_[1][1] + dt * P_[3][1];
    double A12 = P_[1][2] + dt * P_[3][2];
    double A13 = P_[1][3] + dt * P_[3][3];

    double A20 = P_[2][0];
    double A21 = P_[2][1];
    double A22 = P_[2][2];
    double A23 = P_[2][3];

    double A30 = P_[3][0];
    double A31 = P_[3][1];
    double A32 = P_[3][2];
    double A33 = P_[3][3];

    // P = A*F^T
    P_[0][0] = A00 + dt * A02;
    P_[0][1] = A01 + dt * A03;
    P_[0][2] = A02;
    P_[0][3] = A03;

    P_[1][0] = A10 + dt * A12;
    P_[1][1] = A11 + dt * A13;
    P_[1][2] = A12;
    P_[1][3] = A13;

    P_[2][0] = A20 + dt * A22;
    P_[2][1] = A21 + dt * A23;
    P_[2][2] = A22;
    P_[2][3] = A23;

    P_[3][0] = A30 + dt * A32;
    P_[3][1] = A31 + dt * A33;
    P_[3][2] = A32;
    P_[3][3] = A33;

    // Q add (block-diagonal per axis)
    const double q11 = q * (dt4 * 0.25);
    const double q13 = q * (dt3 * 0.5);
    const double q33 = q * (dt2);

    P_[0][0] += q11;
    P_[0][2] += q13;
    P_[2][0] += q13;
    P_[2][2] += q33;
    P_[1][1] += q11;
    P_[1][3] += q13;
    P_[3][1] += q13;
    P_[3][3] += q33;
  }

  // z = [zx, zy]; H = [[1,0,0,0],[0,1,0,0]]
  inline void update(double zx, double zy) noexcept {
    // Innovation
    const double yx = zx - x_;
    const double yy = zy - y_;

    // S = H P H^T + R  => 2x2: [[P00+r2, P01],[P10, P11+r2]]
    const double r2 = meas_var;
    double S00 = P_[0][0] + r2;
    double S01 = P_[0][1];
    double S10 = P_[1][0];
    double S11 = P_[1][1] + r2;

    // Inverse S^{-1} (symmetric → S01==S10)
    const double det = S00 * S11 - S01 * S10;
    double iS00 = S11 / det;
    double iS01 = -S01 / det;
    double iS10 = -S10 / det;
    double iS11 = S00 / det;

    // K = P H^T S^{-1} → 4x2
    // First column of K (for x)
    double K0x = P_[0][0] * iS00 + P_[0][1] * iS10;
    double K1x = P_[1][0] * iS00 + P_[1][1] * iS10;
    double K2x = P_[2][0] * iS00 + P_[2][1] * iS10;
    double K3x = P_[3][0] * iS00 + P_[3][1] * iS10;
    // Second column (for y)
    double K0y = P_[0][0] * iS01 + P_[0][1] * iS11;
    double K1y = P_[1][0] * iS01 + P_[1][1] * iS11;
    double K2y = P_[2][0] * iS01 + P_[2][1] * iS11;
    double K3y = P_[3][0] * iS01 + P_[3][1] * iS11;

    // State update: x = x + K*y
    x_ += K0x * yx + K0y * yy;
    y_ += K1x * yx + K1y * yy;
    vx_ += K2x * yx + K2y * yy;
    vy_ += K3x * yx + K3y * yy;

    // Joseph-form covariance update: P = (I-KH)P(I-KH)^T + K R K^T
    // Compute (I - K H) which only affects first two columns of K
    // We unroll the final simplified form for H picking x,y rows.
    const double r = r2;
    // Precompute KRK^T additions (K * rI * K^T)
    const double rK0xK0x = r * (K0x * K0x + K0y * K0y);
    const double rK0xK1x = r * (K0x * K1x + K0y * K1y);
    const double rK0xK2x = r * (K0x * K2x + K0y * K2y);
    const double rK0xK3x = r * (K0x * K3x + K0y * K3y);

    const double rK1xK1x = r * (K1x * K1x + K1y * K1y);
    const double rK1xK2x = r * (K1x * K2x + K1y * K2y);
    const double rK1xK3x = r * (K1x * K3x + K1y * K3y);

    const double rK2xK2x = r * (K2x * K2x + K2y * K2y);
    const double rK2xK3x = r * (K2x * K3x + K2y * K3y);

    const double rK3xK3x = r * (K3x * K3x + K3y * K3y);

    // (I-KH)P ≡ P - K*(HP). Since H selects rows 0,1, HP = [P00 P01 P02 P03;
    // P10 P11 P12 P13] Compute P' = P - K*HP
    double P00 = P_[0][0] - (K0x * P_[0][0] + K0y * P_[1][0]);
    double P01 = P_[0][1] - (K0x * P_[0][1] + K0y * P_[1][1]);
    double P02 = P_[0][2] - (K0x * P_[0][2] + K0y * P_[1][2]);
    double P03 = P_[0][3] - (K0x * P_[0][3] + K0y * P_[1][3]);

    double P10 = P_[1][0] - (K1x * P_[0][0] + K1y * P_[1][0]);
    double P11 = P_[1][1] - (K1x * P_[0][1] + K1y * P_[1][1]);
    double P12 = P_[1][2] - (K1x * P_[0][2] + K1y * P_[1][2]);
    double P13 = P_[1][3] - (K1x * P_[0][3] + K1y * P_[1][3]);

    double P20 = P_[2][0] - (K2x * P_[0][0] + K2y * P_[1][0]);
    double P21 = P_[2][1] - (K2x * P_[0][1] + K2y * P_[1][1]);
    double P22 = P_[2][2] - (K2x * P_[0][2] + K2y * P_[1][2]);
    double P23 = P_[2][3] - (K2x * P_[0][3] + K2y * P_[1][3]);

    double P30 = P_[3][0] - (K3x * P_[0][0] + K3y * P_[1][0]);
    double P31 = P_[3][1] - (K3x * P_[0][1] + K3y * P_[1][1]);
    double P32 = P_[3][2] - (K3x * P_[0][2] + K3y * P_[1][2]);
    double P33 = P_[3][3] - (K3x * P_[0][3] + K3y * P_[1][3]);

    // P = P' + K R K^T  (keep symmetry)
    P_[0][0] = P00 + rK0xK0x;
    P_[0][1] = P01 + rK0xK1x;
    P_[1][0] = P_[0][1];
    P_[0][2] = P02 + rK0xK2x;
    P_[2][0] = P_[0][2];
    P_[0][3] = P03 + rK0xK3x;
    P_[3][0] = P_[0][3];

    P_[1][1] = P11 + rK1xK1x;
    P_[1][2] = P12 + rK1xK2x;
    P_[2][1] = P_[1][2];
    P_[1][3] = P13 + rK1xK3x;
    P_[3][1] = P_[1][3];

    P_[2][2] = P22 + rK2xK2x;
    P_[2][3] = P23 + rK2xK3x;
    P_[3][2] = P_[2][3];

    P_[3][3] = P33 + rK3xK3x;
  }
};
