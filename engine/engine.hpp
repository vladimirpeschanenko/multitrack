#pragma once
#include "engine/router.hpp"
template <class T>
concept SourceLike = requires(T obj, TrackPoint &out) {
  // must have: int next(TrackPoint&)
  { obj.next(out) } -> std::same_as<int>;
  requires noexcept(obj.next(out));
};
// The same could be done for Sink, Router and WorkerT

// --- Generic run ---
template <SourceLike Source, typename Sink, typename WorkerT>
static int run_crtp(Source &src, const Rectangle &rect, Router<WorkerT> &router,
                    IOStats &io, Sink &sink) {
  TrackPoint tp;
  for (;;) {
    int rc = src.next(tp);
    if (rc == 1) {
      auto &w = router.ensure_worker(tp.id, rect, sink);
      int spins = 0; // if ring is full
      while (!w.tryPushToRing(tp)) {
        if (++spins > 1000) {
          ++io.dropped;
          break;
        }
      }
    } else if (rc == 0) {
      break;
    } else if (rc == -1) {
      ++io.errors;
    }
  }
  return 0;
}