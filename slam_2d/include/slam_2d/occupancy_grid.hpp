// occupancy_grid.hpp

#ifndef OCCUPANCYGRID_HPP
#define OCCUPANCYGRID_HPP

#include <vector>
#include <eigen3/Eigen/Dense>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

namespace slam_2d {
    
struct Cell
{
  double x;
  double y;
  double log_odds;
};
    
class OccupancyGrid
{
public:
  OccupancyGrid() = default;

  OccupancyGrid(double resolution,
                size_t width,
                size_t height,
                Eigen::Vector3d origin);

  // Return the number of cells in grid
  size_t size() const;

  // Return the cell at given row-major index
  Cell& operator[](int index);

  // Update squared Euclidean distance field (i.e. sedt)
  void update_distance_field();

  // Return the squared distance to nearest occupied cell given coordinates
  int occupied_squared_distance_lookup(double x, double y);

  // Return the indices of the cell that covers the given coordinates
  Eigen::Vector2i indices_by_pose(double x, double y);

  // Return the cell at given grid indices
  Cell& cell_by_indices(int idx, int idy);
  Cell& cell_by_indices(Eigen::Vector2i indices);
    
  nav_msgs::msg::MapMetaData ros_map_meta_data_msg() const;

  std::vector<std::vector<int>> sedt; // Squared Euclidean distance field

private:
  double resolution;        // Grid resolution (m/cell)
  uint32_t width;           // Grid width (cells)
  uint32_t height;          // Grid height (cells)
  Eigen::Vector3d origin;   // The origin of the grid (m, m, rad) i.e. the bottom
                            // left corner of cell (0,0) in the world frame
  std::vector<Cell> cells;  // Cells in row-major order
};

// Functions used to precompute distance field
template <class T>
std::vector<std::vector<T>> transpose(const std::vector<std::vector<T>> data);

void find_hull_parabolas(std::vector<int>& row,
                         std::vector<int>& v,
                         std::vector<double>& z);

void march_parabolas(std::vector<int>& row,
                     std::vector<int>& v,
                     std::vector<double>& z);

void horizontal_pass(std::vector<int>& row);

}  // namespace slam_2d

#endif  // OCCUPANCYGRID_HPP
