#pragma once
#include <cstddef>
#include <cstdint>
struct alignas(64) IOStats {
  std::size_t lines{0}, parsed{0}, dropped{0}, errors{0};
  double wall_sec{0.0}; // total wall time (filled in main)
  char _pad[64 - sizeof(std::size_t) * 4 - sizeof(double)]{};
};
struct alignas(64) WorkerStats {
  std::uint32_t id{0};
  std::size_t ingested{0}, snapshots{0};
  double eps{0.0}; // events per second (updated by worker)
  char _pad[64 - sizeof(std::uint32_t) - sizeof(std::size_t) * 2 -
            sizeof(double)]{};
};
