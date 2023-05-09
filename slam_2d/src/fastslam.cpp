// fastslam.cpp

#include <random>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "slam_2d/slam_utils.hpp"
#include "slam_2d/fastslam.hpp"

namespace slam_2d {

static std::random_device rd;
static std::mt19937 gen(rd());

static const double l_0 = log(0.5 / (1 - 0.5));
static const double l_occ = log(0.6 / (1 - 0.6));
static const double l_free = log(0.4 / (1 - 0.4));

FastSlam::FastSlam(int n,
                   double alpha0,
                   double alpha1,
                   double alpha2,
                   double alpha3,
                   double sigma,
                   double zhit,
                   double zmax,
                   double zrand)
  : num_particles(n),
    alpha0(alpha0),
    alpha1(alpha1),
    alpha2(alpha2),
    alpha3(alpha3),
    sigma(sigma),
    zhit(zhit),
    zmax(zmax),
    zrand(zrand)
{

  for (size_t i = 0; i < num_particles; ++i) {
    particles.push_back(Particle());
    weights.push_back(1.0 / num_particles);
  }
}

void
FastSlam::odom_update(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Skip first update
  if (!t_prev) {
    t_prev = std::make_unique<rclcpp::Time>(msg->header.stamp.sec,
                                            msg->header.stamp.nanosec);
    return;
  }

  rclcpp::Duration dt =
    rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec) - *t_prev;

  std::vector<double> ut = {
    msg->twist.twist.linear.x * dt.seconds(), // dx
    msg->twist.twist.linear.y * dt.seconds(), // dy
    msg->twist.twist.angular.z * dt.seconds() // dtheta
  };

  for (auto& p : particles) {
    sample_motion_model(p, ut);
    // sample_odom_motion(p, msg);
  }

  *t_prev = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
}

void
FastSlam::scan_update(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
                      const geometry_msgs::msg::TransformStamped tf_msg)
{
  // TODO: account for lidar angle transform
  lidar_offset[0] = tf_msg.transform.translation.x;
  lidar_offset[1] = tf_msg.transform.translation.y;

  // for (size_t k = 0; k < num_particles; ++k) {
  //     measurement_model_map(scan_msg, particles[k].pose, particles[k].map, weights[k]);
  //     update_occupancy_grid(scan_msg, particles[k].pose, particles[k].map);
  // }

  // TODO: improve naive thread implementation
  std::vector<std::thread> thread_pool_1;
  for (size_t k = 0; k < num_particles; ++k)
    thread_pool_1.emplace_back(&FastSlam::measurement_model_map,
                               this,
                               std::ref(scan_msg),
                               std::ref(particles[k].pose),
                               std::ref(particles[k].map),
                               std::ref(weights[k]));
  for (auto& t : thread_pool_1)
    t.join();

  std::vector<std::thread> thread_pool_2;
  for (size_t k = 0; k < num_particles; ++k)
    thread_pool_2.emplace_back(&FastSlam::update_occupancy_grid,
                               this,
                               std::ref(scan_msg),
                               std::ref(particles[k].pose),
                               std::ref(particles[k].map));
  for (auto& t : thread_pool_2)
    t.join();

  normalize_weights();
  low_variance_resampling();

  // variance reduction technique for resampling
  // std::vector<double> squared_weights(weights.size(), 0.0);
  // std::transform(weights.begin(), weights.end(), squared_weights.begin(),
  //                [](double &d){ return d*d; });
  // double squared_sum = std::accumulate(squared_weights.begin(),
  //                                      squared_weights.end(),
  //                                      0.0);
  // if (0.3 > 1.0 / squared_sum / static_cast<double>(num_particles))
  //     low_variance_resampling();
}

void
FastSlam::sample_odom_motion(Particle& p, const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Skip first odom message
  if (!prev_odom) {
    prev_odom = msg;
    return;
  }

  std::normal_distribution<double> x_dist(0, 0.05);
  std::normal_distribution<double> y_dist(0, 0.05);
  std::normal_distribution<double> theta_dist(0, 0.01);

  p.pose[0] +=
    msg->pose.pose.position.x - prev_odom->pose.pose.position.x + x_dist(gen);
  p.pose[1] +=
    msg->pose.pose.position.y - prev_odom->pose.pose.position.y + y_dist(gen);
  p.pose[2] +=
    tf2::getYaw(msg->pose.pose.orientation) -
    tf2::getYaw(prev_odom->pose.pose.orientation) + theta_dist(gen);

  prev_odom = msg;
}

void
FastSlam::sample_motion_model(Particle& p, const std::vector<double>& ut)
{
  double rot1 = atan2(ut[1], ut[0]);
  double trans = sqrt(pow(ut[0], 2) + pow(ut[1], 2));
  double rot2 = ut[2] - rot1;

  // Reversing creates large rot values and inflated noise
  if (ut[0] < 0) {
    rot1 -= M_PI;
    rot2 += M_PI;
  }

  std::normal_distribution<double> rot1_dist(rot1, sqrt(alpha0 * pow(rot1, 2) +
                                                        alpha1 * pow(trans, 2)));
  std::normal_distribution<double> trans_dist(trans, sqrt(alpha2 * pow(trans, 2) +
                                                          alpha3 * pow(rot1, 2) +
                                                          alpha3 * pow(rot2, 2)));
  std::normal_distribution<double> rot2_dist(rot2, sqrt(alpha0 * pow(rot2, 2) +
                                                        alpha1 * pow(trans, 2)));

  double rot1hat = rot1_dist(gen);
  double transhat = trans_dist(gen);
  double rot2hat = rot2_dist(gen);

  // Correct actions above to accept reversing
  if (ut[0] < 0) {
    rot1 += M_PI;
    rot2 -= M_PI;
    transhat *= -1;
  }

  p.pose[0] += transhat * cos(p.pose[2] + rot1hat);
  p.pose[1] += transhat * sin(p.pose[2] + rot1hat);
  p.pose[2] += rot1hat + rot2hat;
}

void
FastSlam::measurement_model_map(const sensor_msgs::msg::LaserScan::SharedPtr zt,
                                const Eigen::Vector3d& xt,
                                OccupancyGrid& m_prev,
                                double& weight)
{
  weight = likelihood_field_range_finder_model(zt, xt, m_prev);
}

double
FastSlam::likelihood_field_range_finder_model(const sensor_msgs::msg::LaserScan::SharedPtr zt,
                                              const Eigen::Vector3d& xt,
                                              OccupancyGrid& m_prev)
{
  m_prev.update_distance_field();

  double q = 1.0;
  for (size_t k = 0; k < zt->ranges.size(); ++k) {

    // Ignore negative information
    if (zt->ranges[k] > zt->range_max)
      continue;

    double beam_heading = xt[2] + zt->angle_min + k * zt->angle_increment;

    double xzt = xt[0] +
      lidar_offset[0] * cos(xt[2]) -
      lidar_offset[1] * sin(xt[2]) +
      zt->ranges[k] * cos(beam_heading);

    double yzt = xt[1] +
      lidar_offset[1] * cos(xt[2]) +
      lidar_offset[0] * sin(xt[2]) +
      zt->ranges[k] * sin(beam_heading);

    double distance = sqrt(m_prev.occupied_squared_distance_lookup(xzt, yzt));

    q *= zhit * norm_pdf(distance, sigma) + zrand / zmax;
  }

  return q;
}

nav_msgs::msg::OccupancyGrid
FastSlam::ros_og_msg()
{
  auto og_msg = nav_msgs::msg::OccupancyGrid();

  size_t i = std::distance(weights.begin(),
                           std::max_element(weights.begin(), weights.end()));

  og_msg.info = particles[i].map.ros_map_meta_data_msg();
  // Not setting og_msg.info.map_load_time

  og_msg.data.reserve(particles[i].map.size());
  for (size_t j = 0; j < particles[i].map.size(); ++j) {
    double prob = 1 - 1 / (1 + exp(particles[i].map[j].log_odds));
    if (prob == 0.5)
      og_msg.data.push_back(-1);
    else
      og_msg.data.push_back(prob * 100);
  }

  return og_msg;
}

tf2::Transform
FastSlam::ros_transform_data() const
{
  tf2::Transform transform;

  size_t i = std::distance(weights.begin(),
                           std::max_element(weights.begin(), weights.end()));

  transform.setOrigin(tf2::Vector3(particles[i].pose[0],
                                   particles[i].pose[1],
                                   0));
  tf2::Quaternion q;
  q.setRPY(0, 0, particles[i].pose[2]);
  transform.setRotation(q);

  return transform;
}

void
FastSlam::update_occupancy_grid(const sensor_msgs::msg::LaserScan::SharedPtr zt,
                                const Eigen::Vector3d& xt,
                                OccupancyGrid& m_prev)
{
  for (size_t k = 0; k < zt->ranges.size(); ++k) {

    double range = zt->ranges[k];

    // Use negative information (i.e. capture free space for
    // measurements that don't return)
    if (range > zt->range_max)
      range = zt->range_max;

    double beam_heading = xt[2] + zt->angle_min + k * zt->angle_increment;

    double xzt = xt[0] +
      lidar_offset[0] * cos(xt[2]) -
      lidar_offset[1] * sin(xt[2]) +
      range * cos(beam_heading);

    double yzt = xt[1] +
      lidar_offset[1] * cos(xt[2]) +
      lidar_offset[0] * sin(xt[2]) +
      range * sin(beam_heading);

    auto ray_path = bresenham(m_prev.indices_by_pose(xt[0] + lidar_offset[0],
                                                     xt[1] + lidar_offset[1]),
                              m_prev.indices_by_pose(xzt, yzt));

    // Update beliefs for free space
    for (size_t i = 0; i < ray_path.size()-1; ++i) {
      m_prev.cell_by_indices(ray_path[i]).log_odds += l_free - l_0;
    }

    // Update belief for last cell (obstacle if not max range)
    if (zt->ranges[k] > zt->range_max)
      m_prev.cell_by_indices(ray_path.back()).log_odds += l_free - l_0;
    else
      m_prev.cell_by_indices(ray_path.back()).log_odds += l_occ - l_0;
  }
}

void
FastSlam::resampling()
{
  std::vector<Particle> samples;

  std::discrete_distribution<> dist(weights.begin(), weights.end());

  for (size_t m = 0; m < num_particles; ++m) {
    size_t i = dist(gen);
    samples.push_back(particles[i]);
  }

  particles = samples;
}

void
FastSlam::low_variance_resampling()
{
  std::vector<Particle> samples;

  std::uniform_real_distribution<double> dist(0.0, 1.0/num_particles);

  double r = 0.0;
  while (r == 0.0)
    r = dist(gen); // reroll if 0.0

  double c = weights[0];
  size_t i = 0;

  for (size_t m = 0; m < num_particles; ++m) {
    double U = r + static_cast<double>(m) / static_cast<double>(num_particles);
    while (U > c) {
      i++;
      c += weights[i];
    }
    samples.push_back(particles[i]);
  }

  particles = samples;
}

void
FastSlam::normalize_weights()
{
  double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
  std::transform(weights.begin(), weights.end(), weights.begin(),
                 [&sum](double &w){ return w/sum; });
}

}  // namespace slam_2d
