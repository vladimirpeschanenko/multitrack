#pragma once
#include "core/motion_model.hpp"
#include "core/types.hpp"
#include "geom/rectangle.hpp"
#include "util/spsc_ring.hpp"
#include "util/stats.hpp"
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

template <typename Sink, std::size_t RingCap> class alignas(64) Worker {
  const std::uint32_t id;
  Rectangle rect;
  Sink &sink;
  SpscRing<TrackPoint, RingCap> ring;
  MotionModel model;
  std::thread th;
  // EPS state (worker-local → no contention)
  alignas(64) std::size_t last_ingested{0};
  std::chrono::steady_clock::time_point last_tp{
      std::chrono::steady_clock::now()};
  char _pad_eps[64 - sizeof(std::size_t) -
                sizeof(std::chrono::steady_clock::time_point)]{};
  alignas(64) std::atomic<bool> stop{false};
  char _pad_stop[64 - sizeof(std::atomic<bool>)]{};
  WorkerStats stats;

public:
  Worker(std::uint32_t id_, const Rectangle &r, Sink &s)
      : id(id_), rect(r), sink(s) {
    stats.id = id;
  }
  inline bool tryPushToRing(const TrackPoint &tp) { return ring.try_push(tp); }
  inline const WorkerStats &getStats() { return stats; }
  void start() {
    th = std::thread([this] {
      TrackPoint tp;
      while (!stop.load(std::memory_order_acquire) || !ring.empty()) {
        if (ring.try_pop(tp)) {
          model.add(tp);
          ++stats.ingested;
          if ((stats.ingested & 0x3F) == 0) {
            // compute EPS since last snapshot
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_tp).count();
            if (dt > 0.0) {
              stats.eps = double(stats.ingested - last_ingested) / dt;
              last_ingested = stats.ingested;
              last_tp = now;
            }
            sink.on_snapshot(id, tp.t, model, rect, stats.eps);
            ++stats.snapshots;
          }
        } else {
          std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
      }
      if (stats.ingested) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_tp).count();
        if (dt > 0.0) {
          stats.eps = double(stats.ingested - last_ingested) / dt;
        }
        sink.on_snapshot(id, tp.t, model, rect, stats.eps);
        ++stats.snapshots;
      }
    });
  }
  void shutdown() {
    stop.store(true, std::memory_order_release);
    if (th.joinable())
      th.join();
  }
};
