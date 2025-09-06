#include "core/alpha_beta.hpp"
#include "core/kalman.hpp"
#include "engine/engine.hpp"
#include "engine/router.hpp"
#include "engine/worker.hpp"
#include "geom/rectangle.hpp"
#include "io/file_source.hpp"
#include "sinks/console_sink.hpp"
#include "util/parse.hpp"
#include "util/stats.hpp"

#include <iostream>
#include <string>
#include <thread>

struct Cmd {
  std::string source;
  std::string rectFile;
};

static bool parse_args(int argc, char **argv, Cmd &cmd) {
  if (argc < 3)
    return false;
  cmd.source = argv[1];
  cmd.rectFile = argv[2];
  return true;
}

int main(int argc, char **argv) {
  Cmd cmd;
  if (!parse_args(argc, argv, cmd)) {
    std::cerr << "Usage: " << argv[0]
              << " <source> <rect.txt>\n  source: file:<path>\n";
    return 1;
  }
  Rectangle rect;
  if (!load_rectangle_from_file(cmd.rectFile, rect)) {
    std::cerr << "Failed to load rectangle.\n";
    return 2;
  }
  IOStats io;
  ConsoleSink sink;
  // using WorkerT = Worker<ConsoleSink, 1024, MotionModel>;
  // using WorkerT = Worker<ConsoleSink, 1024, AlphaBeta>;
  using WorkerT = Worker<ConsoleSink, 1024, KalmanCV2D>;
  Router<WorkerT> router(io);
  auto t0 = std::chrono::steady_clock::now();
  if (cmd.source.rfind("file:", 0) == 0) {
    FileSource src(io, cmd.source.substr(5));
    if (!src.good()) {
      std::cerr << "Cannot open file.\n";
      return 3;
    }
    run_crtp(src, rect, router, io, sink);
  } else {
    std::cerr << "Unknown source.\n";
    return 5;
  }

  router.shutdown_all();
  io.wall_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
          .count();
  sink.on_finish(io, router.getFinalStats());
  return 0;
}
