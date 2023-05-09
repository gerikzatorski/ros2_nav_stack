// slam_node.hpp

#ifndef SLAM_NODE_HPP
#define SLAM_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_broadcaster.h"

#include "slam_2d/fastslam.hpp"

namespace slam_2d {

class SlamNode : public rclcpp::Node
{
public:
  SlamNode();

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void og_timer_callback();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub;

  // Timer used to periodically publish occupancy grid message
  rclcpp::TimerBase::SharedPtr timer;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  tf2::Stamped<tf2::Transform> t_base_to_laser {};

  FastSlam algorithm;
};

}  // namespace slam_2d

#endif  // SLAM_NODE_HPP
