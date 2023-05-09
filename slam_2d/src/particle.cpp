// particle.cpp

#include "slam_2d/particle.hpp"

namespace slam_2d {

Particle::Particle()
{
  pose = Eigen::Vector3d {0, 0, 0};
  map = OccupancyGrid(0.25, 160, 160, Eigen::Vector3d {-20, -20, 0});
}


std::ostream& operator<<(std::ostream& os, const Particle& p)
{
  os << p.pose[0] << " " << p.pose[1] << " " << p.pose[2];
  return os;
}

}  // namespace slam_2d
