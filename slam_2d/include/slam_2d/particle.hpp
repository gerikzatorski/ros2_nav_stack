// particle.hpp

#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <eigen3/Eigen/Dense>

#include "slam_2d/occupancy_grid.hpp"

namespace slam_2d {

struct Particle
{
  Particle();

  Eigen::Vector3d pose;

  OccupancyGrid map;

  // Overload the insertion operator
  friend std::ostream& operator<<(std::ostream& os, const Particle& p);
};

}  // namespace slam_2d

#endif  // PARTICLE_HPP
