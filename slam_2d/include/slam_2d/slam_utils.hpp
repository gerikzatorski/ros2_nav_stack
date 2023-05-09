// slam_utils.hpp

#ifndef SLAM_UTILS_HPP
#define SLAM_UTILS_HPP

#define _USE_MATH_DEFINES

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

namespace slam_2d {

inline double norm_pdf(double a, double stddev)
{
  return 1 / sqrt(2 * M_PI * stddev * stddev)
    * exp(-0.5 * a * a / stddev / stddev);
}

// Bresenham's line algorithm for all octants
std::vector<Eigen::Vector2i> bresenham(const Eigen::Vector2i& start,
                                       const Eigen::Vector2i& end);

// Alternate, currently unused
std::vector<Eigen::Vector2i> bresenham_split(Eigen::Vector2i start, Eigen::Vector2i end);
void bresenham_low(int x0, int y0, int x1, int y1, std::vector<Eigen::Vector2i>& path);
void bresenham_high(int x0, int y0, int x1, int y1, std::vector<Eigen::Vector2i>& path);

}  // namespace slam_2d

#endif  // SLAM_UTILS_HPP
