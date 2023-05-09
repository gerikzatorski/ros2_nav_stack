// slam_utils.cpp

#include "slam_2d/slam_utils.hpp"

namespace slam_2d {

std::vector<Eigen::Vector2i> bresenham(const Eigen::Vector2i& start,
                                       const Eigen::Vector2i& end)
{
  std::vector<Eigen::Vector2i> path {};

  int x1 = start[0];
  int y1 = start[1];
  int x2 = end[0];
  int y2 = end[1];

  int w = x2 - x1;
  int h = y2 - y1;

  int dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0;

  if (w < 0)
    dx1 = -1;
  else if (w > 0)
    dx1 = 1;

  if (h < 0)
    dy1 = -1;
  else if (h > 0)
    dy1 = 1;

  if (w < 0)
    dx2 = -1;
  else if (w > 0)
    dx2 = 1;

  int longest = abs(w);
  int shortest = abs(h);
  if (!(longest > shortest)) {
    longest = abs(h);
    shortest = abs(w);
    if (h < 0)
      dy2 = -1;
    else if (h > 0)
      dy2 = 1;
    dx2 = 0;
  }

  int numerator = longest >> 1;

  for (int i = 0; i <= longest; ++i) {
    path.emplace_back(x1, y1);

    numerator += shortest;
    if (!(numerator < longest)) {
      numerator -= longest;
      x1 += dx1;
      y1 += dy1;
    } else {
      x1 += dx2;
      y1 += dy2;
    }
  }

  return path;
}

std::vector<Eigen::Vector2i> bresenham_split(Eigen::Vector2i start, Eigen::Vector2i end)
{
  std::vector<Eigen::Vector2i> path {};

  int x0 = start[0];
  int y0 = start[1];
  int x1 = end[0];
  int y1 = end[1];

  int dx = x1 - x0;
  int dy = y1 - y0;

  if (abs(dy) < abs(dx))
    if (x0 > x1) {
      bresenham_low(x1, y1, x0, y0, path);
      std::reverse(path.begin(), path.end()); // order matters to caller
    }
    else
      bresenham_low(x0, y0, x1, y1, path);
  else
    if (y0 > y1) {
      bresenham_high(x1, y1, x0, y0, path);
      std::reverse(path.begin(), path.end()); // order matters to caller
    }
    else
      bresenham_high(x0, y0, x1, y1, path);

  return path;
}

void bresenham_low(int x0, int y0, int x1, int y1, std::vector<Eigen::Vector2i>& path)
{
  int dx = x1 - x0;
  int dy = y1 - y0;
  int yi = 1;

  if (dy < 0) {
    yi = -1;
    dy = -dy;
  }
  int D = 2*dy - dx;
  int y = y0;

  int step;
  if (x1 > x0)
    step = 1;
  else
    step = -1;

  for (int x = x0; x != x1; x += step) {
    path.emplace_back(x, y);
    if (D > 0) {
      y = y + yi;
      D = D + 2*(dy - dx);
    }
    else {
      D = D + 2*dy;
    }
  }
}

void bresenham_high(int x0, int y0, int x1, int y1, std::vector<Eigen::Vector2i>& path)
{
  int dx = x1 - x0;
  int dy = y1 - y0;
  int xi = 1;

  if (dx < 0) {
    xi = -1;
    dx = -dx;
  }
  int D = 2*dx - dy;
  int x = x0;

  int step;
  if (y1 > y0)
    step = 1;
  else
    step = -1;

  for (int y = y0; y != y1; y += step) {
    path.emplace_back(x, y);
    if (D > 0) {
      x = x + xi;
      D = D + 2*(dx-dy);
    }
    else {
      D = D + 2*dx;
    }
  }
}

}  // namespace slam_2d
