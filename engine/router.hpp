#pragma once
#include "core/types.hpp"
#include "geom/rectangle.hpp"
#include "util/stats.hpp"
#include <memory>
#include <unordered_map>
#include <vector>

template <typename WorkerT> class alignas(64) Router {
  IOStats &io;
  std::unordered_map<std::uint32_t, std::unique_ptr<WorkerT>> workers;
  std::vector<WorkerStats> final_stats;

public:
  Router(IOStats &s) : io(s) { workers.reserve(64); }
  inline const std::vector<WorkerStats> &getFinalStats() {
    return final_stats;
  };
  template <typename Sink>
  WorkerT &ensure_worker(std::uint32_t id, const Rectangle &rect, Sink &sink) {
    auto it = workers.find(id);
    if (it == workers.end()) {
      auto w = std::make_unique<WorkerT>(id, rect, sink);
      auto &ref = *w;
      workers.emplace(id, std::move(w));
      ref.start();
      return ref;
    }
    return *it->second;
  }
  void shutdown_all() {
    final_stats.clear();
    final_stats.reserve(workers.size());
    for (auto &kv : workers)
      kv.second->shutdown();
    for (auto &kv : workers)
      final_stats.push_back(kv.second->getStats());
  }
};
