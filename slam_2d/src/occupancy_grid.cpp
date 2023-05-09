// occupancy_grid.cpp

#include <limits>
#include <algorithm>

#include "slam_2d/occupancy_grid.hpp"

namespace slam_2d {

// To avoid math with infinity
static int big_int;

OccupancyGrid::OccupancyGrid(double resolution,
                             size_t width,
                             size_t height,
                             Eigen::Vector3d origin)
  : resolution(resolution),
    width(width),
    height(height),
    origin(origin)
{
  for (size_t i = 0; i < size(); ++i) {
    Cell c;
    c.x = origin[0] + resolution/2 + (i % width) * resolution;
    c.y = origin[1] + resolution/2 + (i / width) * resolution;
    c.log_odds = 0;
    cells.push_back(c);
  }

  sedt = std::vector<std::vector<int>>(width, std::vector<int>(height, 0));
}

size_t
OccupancyGrid::size() const
{
  return width * height;
}

Cell&
OccupancyGrid::operator[](int index)
{
  return cells[index];
}

void
OccupancyGrid::update_distance_field()
{
  big_int = width * height;

  for (size_t i = 0; i < width; ++i) {
    for (size_t j = 0; j < height; ++j) {
      if (cells[i + width * j].log_odds > 0)
        sedt[i][j] = 0;
      else
        sedt[i][j] = big_int;
    }
  }

  for (size_t i = 0; i < sedt.size(); ++i)
    horizontal_pass(sedt[i]);

  std::vector<std::vector<int>> sedt2 = transpose(sedt);

  for (size_t i = 0; i < sedt2.size(); ++i)
    horizontal_pass(sedt2[i]);

  sedt = transpose(sedt2);
}

int
OccupancyGrid::occupied_squared_distance_lookup(double x, double y)
{
  Eigen::Vector2i indices = indices_by_pose(x, y);

  return sedt[indices[0]][indices[1]];
}

Eigen::Vector2i
OccupancyGrid::indices_by_pose(double x, double y)
{
  size_t idx = (x - origin[0]) / resolution;
  size_t idy = (y - origin[1]) / resolution;
  return Eigen::Vector2i (idx, idy);
}

Cell&
OccupancyGrid::cell_by_indices(int idx, int idy)
{
  return cells[ (int) (idx + idy * width) ];
}

Cell&
OccupancyGrid::cell_by_indices(Eigen::Vector2i indices)
{
  return cells[ (int) (indices[0] + indices[1] * width) ];
}

nav_msgs::msg::MapMetaData
OccupancyGrid::ros_map_meta_data_msg() const
{
  auto msg = nav_msgs::msg::MapMetaData();

  msg.width = width;
  msg.height = height;
  msg.resolution = resolution;
  msg.origin.position.x = origin[0];
  msg.origin.position.y = origin[1];

  return msg;
}

template <class T>
std::vector<std::vector<T>>
transpose(const std::vector<std::vector<T> > data)
{
  std::vector<std::vector<T>> result(data[0].size(),
                                     std::vector<T>(data.size()));
  for (size_t i = 0; i < data[0].size(); ++i)
    for (size_t j = 0; j < data.size(); ++j) {
      result[i][j] = data[j][i];
    }
  return result;
}

void
find_hull_parabolas(std::vector<int>& row,
                    std::vector<int>& v,
                    std::vector<double>& z)
{
  size_t k = 0;  // track rightmost parabola
  v.push_back(0);
  z.push_back(-big_int);
  z.push_back(+big_int);

  size_t q = 1;
  while (q < row.size()) {
    double s = ( static_cast<double>(row[q] + q*q) - static_cast<double>(row[v[k]] + v[k]*v[k]) )
      / (2*q - 2*v[k]);

    // If the intersection is before z[k], then the parabola from v[k] should not be part
    // of the new lower envelope
    if (s <= z[k]) {
      k--;
      continue;
    }

    k++;

    if (k < v.size())
      v[k] = q;
    else
      v.push_back(q);

    if (k < z.size())
      z[k] = s;
    else
      z.push_back(s);

    z.push_back(+big_int);

    q++;
  }
}

void
march_parabolas(std::vector<int>& row,
                std::vector<int>& v,
                std::vector<double>& z)
{
  size_t k = 0;

  for (size_t q = 0; q < row.size(); ++q) {
    while (z[k+1] < q)
      k++;

    int dx = (int) (q - v[k]);
    row[q] = dx*dx + row[v[k]];
  }
}

void
horizontal_pass(std::vector<int>& row)
{
  std::vector<int> hull_vertices {};
  std::vector<double> hull_intersections {};

  find_hull_parabolas(row, hull_vertices, hull_intersections);
  march_parabolas(row, hull_vertices, hull_intersections);
}

}  // namespace slam_2d
