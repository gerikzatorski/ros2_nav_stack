// slam_node.cpp

#include <chrono>
#include <functional>

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" // tf::convert
#include "tf2/LinearMath/Quaternion.h"

#include "slam_2d/slam_node.hpp"

namespace slam_2d {

using namespace std::chrono_literals;
using std::placeholders::_1;

SlamNode::SlamNode()
  : Node("slam_node")
{
  // Declare ROS params
  this->declare_parameter("number_of_particles");
  this->declare_parameter("alpha0");
  this->declare_parameter("alpha1");
  this->declare_parameter("alpha2");
  this->declare_parameter("alpha3");
  this->declare_parameter("sigma");
  this->declare_parameter("zhit");
  this->declare_parameter("zmax");
  this->declare_parameter("zrand");

  // Setup ROS publishers and subscribers
  scan_sub =
    this->create_subscription<sensor_msgs::msg::LaserScan>
    ("scan", 10, std::bind(&SlamNode::scan_callback, this, _1));
  odom_sub =
    this->create_subscription<nav_msgs::msg::Odometry>
    ("odom", 10, std::bind(&SlamNode::odom_callback, this, _1));
  grid_pub =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
  timer =
    this->create_wall_timer(500ms,
                            std::bind(&SlamNode::og_timer_callback, this));

  // Setup ROS tf objects
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Setup algorithm for node
  algorithm = FastSlam(this->get_parameter("number_of_particles").as_int(),
                       this->get_parameter("alpha0").as_double(),
                       this->get_parameter("alpha1").as_double(),
                       this->get_parameter("alpha2").as_double(),
                       this->get_parameter("alpha3").as_double(),
                       this->get_parameter("sigma").as_double(),
                       this->get_parameter("zhit").as_double(),
                       this->get_parameter("zmax").as_double(),
                       this->get_parameter("zrand").as_double());
}

void
SlamNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  // Listen for base-->laser msg
  geometry_msgs::msg::TransformStamped base_to_laser_msg;
  try {
    base_to_laser_msg = tf_buffer->lookupTransform("base_link", "laser",
                                                   tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(),
                "Could not transform %s to %s: %s", "base_link", "laser", ex.what());
    return;
  }

  algorithm.scan_update(scan_msg, base_to_laser_msg);
}

void
SlamNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  algorithm.odom_update(msg);
}

void
SlamNode::og_timer_callback()
{
  // Listen for base-->odom msg
  geometry_msgs::msg::TransformStamped base_to_odom_msg;
  try {
    base_to_odom_msg = tf_buffer->lookupTransform("base_link", "odom",
                                                  tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(),
                "Could not transform %s to %s: %s", "base_link", "odom", ex.what());
    return;
  }

  // Convert to base-->odom tf
  tf2::Stamped<tf2::Transform> t_base_to_odom;
  tf2::fromMsg(base_to_odom_msg, t_base_to_odom);

  // Retrieve map-->base_link tf (from slam algorithm)
  tf2::Stamped<tf2::Transform> t_map_to_base;
  t_map_to_base.setData(algorithm.ros_transform_data());

  // Calculate map-->odom tf
  tf2::Stamped<tf2::Transform> t_map_to_odom;
  t_map_to_odom.setData(t_map_to_base * t_base_to_odom);

  // Convert to map-->odom msg
  geometry_msgs::msg::TransformStamped map_to_odom_msg;
  map_to_odom_msg = tf2::toMsg(t_map_to_odom);

  // Complete and broadcast map-->odom msg
  map_to_odom_msg.header.stamp = base_to_odom_msg.header.stamp;
  map_to_odom_msg.header.frame_id = "map";
  map_to_odom_msg.child_frame_id = "odom";
  tf_broadcaster->sendTransform(map_to_odom_msg);

  // Publish occupancy grid message
  nav_msgs::msg::OccupancyGrid og_msg = algorithm.ros_og_msg();
  og_msg.header.stamp = base_to_odom_msg.header.stamp;
  og_msg.header.frame_id = "map";

  grid_pub->publish(og_msg);
}

}  // namespace slam_2d

int
main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_2d::SlamNode>());
  rclcpp::shutdown();
  return 0;
}
