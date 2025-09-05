#pragma once
#include "core/intersector.hpp"
#include "core/motion_model.hpp"
#include "core/types.hpp"
#include "geom/rectangle.hpp"
#include "util/stats.hpp"
#include <iomanip>
#include <iostream>

class ConsoleSink {
public:
  inline void on_snapshot(std::uint32_t id, double t_now,
                          const MotionModel &model, const Rectangle &rect,
                          double eps) noexcept {
    if (!model.ready()) {
      std::cout << "[id=" << id << " t=" << t_now << "] warming up...\n";
      return;
    }
    double vx, vy;
    model.velocity(vx, vy);
    const double speed = model.speed();
    const double head = model.headingDeg();
    auto entry = Intersector::firstEntry(model, rect, 0.0);
    std::cout.setf(std::ios::fixed);
    std::cout << std::setprecision(3);
    std::cout << "[id=" << id << " t=" << t_now << "] v=(" << vx << "," << vy
              << ") speed=" << speed << " heading=" << head << " deg\n";
    if (entry.will_enter)
      std::cout << "  Enter @ t=" << entry.t_enter << " at (" << entry.p_enter.x
                << "," << entry.p_enter.y << ")\n";
    else
      std::cout << "  Not on course to enter.\n";
    std::cout << "  EPS=" << eps << "\n";
  }
  inline void on_finish(const IOStats &io,
                        const std::vector<WorkerStats> &workers) noexcept {
    std::cout << "\n=== IO Stats ===\n"
              << "lines=" << io.lines << " parsed=" << io.parsed
              << " dropped=" << io.dropped << " errors=" << io.errors
              << "\n=== Worker Stats ===\n";
    for (const auto &w : workers)
      std::cout << "id=" << w.id << " ingested=" << w.ingested
                << " snapshots=" << w.snapshots << " last_EPS=" << w.eps
                << "\n";
    // overall average EPS
    std::size_t total = 0;
    for (const auto &w : workers)
      total += w.ingested;
    if (io.wall_sec > 0.0)
      std::cout << "=== Overall avg EPS: " << (double(total) / io.wall_sec)
                << "\n";
  }
};
