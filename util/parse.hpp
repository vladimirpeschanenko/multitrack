#pragma once
#include "core/types.hpp"
#include "geom/rectangle.hpp"
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <string>

inline bool parse_track_line(std::string &s, TrackPoint &out) noexcept {
  unsigned id;
  double t, x, y;

  // Try: id,t,x,y
  /*if (std::sscanf(s.c_str(), " %u , %lf , %lf , %lf", &id, &t, &x, &y) == 4) {
    out.id = id;
    out.t = t;
    out.p = {x, y};
    return true;
  }*/

  // t,x,y  (id=0)
  if (std::sscanf(s.c_str(), " %lf , %lf , %lf", &t, &x, &y) == 3) {
    out.id = 0;
    out.t = t;
    out.p = {x, y};
    return true;
  }
  return false;
}

inline bool load_rectangle_from_file(const std::string &path, Rectangle &r) {
  std::ifstream in(path);
  if (!in)
    return false;

  std::string line;
  double xmin = 0, xmax = 0, ymin = 0, ymax = 0;
  bool first = true;
  int count = 0;

  while (std::getline(in, line)) {
    double x, y;
    if (std::sscanf(line.c_str(), " %lf , %lf", &x, &y) != 2)
      continue;

    if (first) {
      xmin = xmax = x;
      ymin = ymax = y;
      first = false;
    } else {
      if (x < xmin)
        xmin = x;
      if (x > xmax)
        xmax = x;
      if (y < ymin)
        ymin = y;
      if (y > ymax)
        ymax = y;
    }
    ++count;
  }

  if (count != 4)
    return false;
  r.xmin = xmin;
  r.ymin = ymin;
  r.xmax = xmax;
  r.ymax = ymax;
  return true;
}
