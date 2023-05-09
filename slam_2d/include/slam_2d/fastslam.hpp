// fastslam.hpp

#ifndef FASTSLAM_HPP
#define FASTSLAM_HPP

#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "slam_2d/particle.hpp"
#include "slam_2d/occupancy_grid.hpp"

namespace slam_2d {

class FastSlam
{
public:
  FastSlam() = default;
    
  FastSlam(int n,
           double alpha0,
           double alpha1,
           double alpha2,
           double alpha3,
           double sigma,
           double zhit,
           double zmax,
           double zrand);
    
  void odom_update(const nav_msgs::msg::Odometry::SharedPtr msg);

  void scan_update(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
                   const geometry_msgs::msg::TransformStamped tf_msg);

  // Using heaviest weighted particle, create ros occupancy grid message
  nav_msgs::msg::OccupancyGrid ros_og_msg();

  // Using heaviest weight particle, create ros transform
  tf2::Transform ros_transform_data() const;

private:
  // For particle k, generate random sample x[t], given a fixed control
  // u[t] (multipled by dt) and pose x[t-1]
  void sample_motion_model(Particle& p, const std::vector<double>& ut);

  // Sample motion using odometry feedback (currently unused)
  void sample_odom_motion(Particle& p, const nav_msgs::msg::Odometry::SharedPtr msg);

  // Compute likelihood of the measurement z[t] given the pose x[t]
  // and the map m[t-1] and update weight (i.e. importance factor)
  void measurement_model_map(const sensor_msgs::msg::LaserScan::SharedPtr zt,
                             const Eigen::Vector3d& xt,
                             OccupancyGrid& m_prev,
                             double& weight);

  // Calculate probability of measurement zt using the likelihood field
  double likelihood_field_range_finder_model(const sensor_msgs::msg::LaserScan::SharedPtr zt,
                                             const Eigen::Vector3d& xt,
                                             OccupancyGrid& m_prev);

  // Compute a new occupancy grid map given the current pose x[t],
  // the map m[t-1], and the measurement z[t]
  void update_occupancy_grid(const sensor_msgs::msg::LaserScan::SharedPtr zt,
                             const Eigen::Vector3d& xt,
                             OccupancyGrid& m_prev);

  void resampling();
  void low_variance_resampling();

  void normalize_weights();
    
  size_t num_particles;
  std::vector<Particle> particles;
  std::vector<double> weights;

  // motion noise parameters
  double alpha0;
  double alpha1;
  double alpha2;
  double alpha3;
  double sigma;

  // mixing parameters (must sum to 1.0)
  double zhit;
  double zmax;
  double zrand;

  Eigen::Vector3d lidar_offset;

  std::unique_ptr<rclcpp::Time> t_prev {nullptr};
  nav_msgs::msg::Odometry::SharedPtr prev_odom {nullptr};
};

}  // namespace slam_2d

#endif  // FASTSLAM_HPP
