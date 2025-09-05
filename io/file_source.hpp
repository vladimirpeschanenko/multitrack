#pragma once
#include "core/types.hpp"
#include "util/parse.hpp"
#include "util/stats.hpp"
#include <fstream>
#include <string>

class FileSource {
  std::ifstream in;
  IOStats &stats;

public:
  explicit FileSource(IOStats &io, const std::string &path)
      : in(path), stats(io) {}
  inline bool good() const noexcept { return in.good(); }
  inline int next(TrackPoint &out) noexcept {
    std::string line;
    if (!std::getline(in, line))
      return 0;
    ++stats.lines;
    if (line.empty() || line.find_first_not_of(" \t\r\n") == std::string::npos)
      return -1;
    if (parse_track_line(line, out)) {
      ++stats.parsed;
      return 1;
    }
    ++stats.errors;
    return -1;
  }
};
