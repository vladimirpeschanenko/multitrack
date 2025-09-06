#pragma once
#include "core/intersector.hpp"
#include "core/motion_model.hpp"
#include "core/types.hpp"
#include "geom/rectangle.hpp"
#include "util/stats.hpp"
#include <iomanip>
#include <iostream>

/**
 * In general, to keep the hot path lock-free, we should not print from the
 * worker threads at all. Instead, here we should push a preformatted message
 * into its own SPSC ring. A single logger thread drains all rings and does the
 * (blocking) console write
 * */
class ConsoleSink {
  alignas(64) mutable std::mutex mu_;

public:
  template <typename T>
  inline void on_snapshot(std::uint32_t id, double t_now, const T &model,
                          const Rectangle &rect, double eps) noexcept {
    std::ostringstream ostr;
    if (!model.ready()) {
      ostr << "[id=" << id << " t=" << t_now << "] warming up...\n";
    } else {
      double vx, vy;
      model.velocity(vx, vy);
      const double speed = model.speed();
      const double head = model.headingDeg();
      auto entry = Intersector::firstEntry(model, rect, 0.0);
      ostr.setf(std::ios::fixed);
      ostr << std::setprecision(3);
      ostr << "[id=" << id << " t=" << t_now << "] v=(" << vx << "," << vy
           << ") speed=" << speed << " heading=" << head << " deg\n";
      if (entry.will_enter)
        ostr << "  Enter @ t=" << entry.t_enter << " at (" << entry.p_enter.x
             << "," << entry.p_enter.y << ")\n";
      else
        ostr << "  Not on course to enter.\n";
      ostr << "  EPS=" << eps << "\n";
    }
    const std::string s = ostr.str();
    std::lock_guard<std::mutex> lk(mu_);
    std::fwrite(s.data(), 1, s.size(), stdout);
    std::fflush(stdout);
  }
  inline void on_finish(const IOStats &io,
                        const std::vector<WorkerStats> &workers) noexcept {
    std::ostringstream ostr;
    ostr << "\n=== IO Stats ===\n"
         << "lines=" << io.lines << " parsed=" << io.parsed
         << " dropped=" << io.dropped << " errors=" << io.errors
         << "\n=== Worker Stats ===\n";
    for (const auto &w : workers)
      ostr << "id=" << w.id << " ingested=" << w.ingested
           << " snapshots=" << w.snapshots << " last_EPS=" << w.eps << "\n";
    // overall average EPS
    std::size_t total = 0;
    for (const auto &w : workers)
      total += w.ingested;
    if (io.wall_sec > 0.0)
      ostr << "=== Overall avg EPS: " << (double(total) / io.wall_sec) << "\n";
    const std::string s = ostr.str();
    std::lock_guard<std::mutex> lk(mu_);
    std::fwrite(s.data(), 1, s.size(), stdout);
    std::fflush(stdout);
  }
};
