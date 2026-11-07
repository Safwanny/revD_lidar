// Copyright 2026 Safwan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Sits between twist_mux and the controller: /cmd_vel_mux ->
// /mobile_base_controller/reference, limited by governor.hpp. The points it
// reacts to are the ToF rows of all four faces, kept for a few seconds in
// the odom frame (near the hull corners no face sees anything, each covers
// only +-30 deg), and the latest LiDAR scan. tof_only mode drops the LiDAR
// and snaps commands to axis_only; it is on while the tof_only parameter is
// set or /lidar/ok last said false.

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "revd_lidar_safety/governor.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace revd_lidar_safety
{

namespace
{

constexpr std::array<const char *, 4> kFaces{"front", "rear", "left", "right"};
// Room for the ToF memory and one scan, so the callbacks do not allocate:
// 4 faces x 24 kept zones x 15 Hz x 3 s, and the D500's 500 beams.
constexpr std::size_t kMemoryCapacity = 8192;
constexpr std::size_t kScanCapacity = 1024;

struct Stamped
{
  double seconds;
  Point odom;
};

}  // namespace

class SafetyGovernor : public rclcpp::Node
{
public:
  explicit SafetyGovernor(const rclcpp::NodeOptions & options)
  : Node("safety_governor", options), tf_buffer_(get_clock())
  {
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, this);
    // Out-of-range values throw here, so the node never starts on them.
    p_.half_length = declare("half_length", p_.half_length, 0.05, 0.5);
    p_.half_width = declare("half_width", p_.half_width, 0.05, 0.5);
    p_.max_speed = declare("max_speed", p_.max_speed, 0.0, 1.0);
    p_.max_angular = declare("max_angular", p_.max_angular, 0.0, 5.0);
    p_.brake_decel = declare("brake_decel", p_.brake_decel, 0.1, 5.0);
    p_.latency = declare("latency", p_.latency, 0.0, 1.0);
    p_.min_gap = declare("min_gap", p_.min_gap, 0.0, 0.2);
    p_.side_min_gap = declare("side_min_gap", p_.side_min_gap, 0.0, 0.2);
    p_.creep_speed = declare("creep_speed", p_.creep_speed, 0.0, 0.1);
    p_.side_gap = declare("side_gap", p_.side_gap, 0.0, 0.5);
    p_.side_factor = declare("side_factor", p_.side_factor, 0.0, 1.0);
    p_.slow_after = declare("slow_after", p_.slow_after, 0.0, 5.0);
    p_.stale_after = declare("stale_after", p_.stale_after, 0.0, 5.0);
    p_.trim_angular = declare("trim_angular", p_.trim_angular, 0.0, 5.0);
    p_.spin_margin = declare("spin_margin", p_.spin_margin, 0.0, 0.2);
    min_height_ = declare("min_height", 0.02, -1.0, 1.0);
    max_height_ = declare("max_height", 0.20, -1.0, 2.0);
    memory_ = declare("memory", 3.0, 0.0, 10.0);
    force_tof_only_ = declare_parameter("tof_only", false, read_only("force tof_only mode"));
    base_frame_ = declare_parameter("base_frame", "base_footprint", read_only("robot frame"));
    odom_frame_ = declare_parameter("odom_frame", "odom", read_only("frame points are kept in"));
    if (p_.stale_after < p_.slow_after || max_height_ <= min_height_) {
      throw std::invalid_argument("stale_after < slow_after or max_height <= min_height");
    }

    memory_points_.reserve(kMemoryCapacity);
    scan_points_.reserve(kScanCapacity);
    points_.reserve(kMemoryCapacity + kScanCapacity);
    seen_.fill(-std::numeric_limits<double>::infinity());
    scan_seen_ = -std::numeric_limits<double>::infinity();

    const auto sensor_qos = rclcpp::SensorDataQoS();
    for (std::size_t face = 0; face < kFaces.size(); ++face) {
      tof_subscriptions_.push_back(
        create_subscription<sensor_msgs::msg::PointCloud2>(
          std::string("/tof/") + kFaces[face] + "/rows", sensor_qos,
          [this, face](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
            on_cloud(face, *message);
          }));
    }
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", sensor_qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {on_scan(*message);});
    // Latched: the health monitor states it once and on every change.
    lidar_ok_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/lidar/ok", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        if (lidar_ok_ != message->data) {
          RCLCPP_WARN(get_logger(), "LiDAR %s: %s mode", message->data ? "ok" : "lost",
          message->data ? "normal" : "tof_only");
        }
        lidar_ok_ = message->data;
      });
    output_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mobile_base_controller/reference", 10);
    command_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_mux", 10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr message) {on_command(*message);});
  }

private:
  static rcl_interfaces::msg::ParameterDescriptor read_only(const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
  }

  double declare(const std::string & name, double value, double low, double high)
  {
    auto descriptor = read_only("see revd_lidar_bringup/config/safety.yaml");
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = low;
    range.to_value = high;
    descriptor.floating_point_range.push_back(range);
    return declare_parameter(name, value, descriptor);
  }

  double now() {return get_clock()->now().seconds();}

  bool tof_only() const {return force_tof_only_ || !lidar_ok_;}

  // Transform to odom at the message stamp, else the latest one.
  bool to_odom(const std_msgs::msg::Header & header, tf2::Transform & transform)
  {
    try {
      tf2::fromMsg(
        tf_buffer_.lookupTransform(odom_frame_, header.frame_id, header.stamp).transform,
        transform);
      return true;
    } catch (const tf2::TransformException &) {
    }
    try {
      tf2::fromMsg(
        tf_buffer_.lookupTransform(odom_frame_, header.frame_id, tf2::TimePointZero).transform,
        transform);
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
      return false;
    }
  }

  void on_cloud(std::size_t face, const sensor_msgs::msg::PointCloud2 & message)
  {
    const double stamp = now();
    seen_[face] = stamp;
    tf2::Transform transform;
    if (!to_odom(message.header, transform)) {
      return;
    }
    sensor_msgs::PointCloud2ConstIterator<float> x(message, "x"), y(message, "y"),
    z(message, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }
      const tf2::Vector3 q = transform * tf2::Vector3(*x, *y, *z);
      // odom z is height above the floor (the EKF runs in 2D).
      if (q.z() >= min_height_ && q.z() <= max_height_) {
        memory_points_.push_back({stamp, {q.x(), q.y()}});
      }
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan & message)
  {
    scan_seen_ = now();
    tf2::Transform transform;
    if (!to_odom(message.header, transform)) {
      return;
    }
    // Only the latest scan: it sees all round, over the hull. The scan
    // plane is above the ToF height window, so no height filter.
    scan_points_.clear();
    for (std::size_t i = 0; i < message.ranges.size(); ++i) {
      const double r = message.ranges[i];
      if (!std::isfinite(r) || r < message.range_min || r > message.range_max) {
        continue;
      }
      const double a = message.angle_min + i * message.angle_increment;
      const tf2::Vector3 q = transform * tf2::Vector3(r * std::cos(a), r * std::sin(a), 0.0);
      scan_points_.push_back({q.x(), q.y()});
    }
  }

  void on_command(const geometry_msgs::msg::TwistStamped & message)
  {
    geometry_msgs::msg::TwistStamped out;
    out.header.frame_id = message.header.frame_id;
    // Our own clock, not the input's: the controller times a command out by
    // its stamp, and a sender on another clock (wall time against sim time)
    // would make every command look fresh forever.
    out.header.stamp = get_clock()->now();
    geometry_msgs::msg::TransformStamped pose;
    try {
      pose = tf_buffer_.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      // No pose, no memory: publish zero rather than drive blind.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
      output_->publish(out);
      return;
    }
    const double stamp = now();
    std::size_t old = 0;
    while (old < memory_points_.size() && stamp - memory_points_[old].seconds > memory_) {
      ++old;
    }
    memory_points_.erase(memory_points_.begin(), memory_points_.begin() + old);

    const bool axis_mode = tof_only();
    const double x = pose.transform.translation.x;
    const double y = pose.transform.translation.y;
    const double yaw = tf2::getYaw(pose.transform.rotation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    auto to_base = [&](const Point & q) {
        const double dx = q.x - x;
        const double dy = q.y - y;
        return Point{dx * c + dy * s, -dx * s + dy * c};
      };
    points_.clear();
    for (const auto & kept : memory_points_) {
      points_.push_back(to_base(kept.odom));
    }
    Ages ages;
    for (std::size_t face = 0; face < kFaces.size(); ++face) {
      ages.faces[face] = stamp - seen_[face];
    }
    if (!axis_mode) {
      for (const auto & q : scan_points_) {
        points_.push_back(to_base(q));
      }
      ages.lidar = stamp - scan_seen_;
    }
    Command command{message.twist.linear.x, message.twist.linear.y, message.twist.angular.z};
    if (axis_mode) {
      command = axis_only(command, points_, p_);
    }
    const Command safe = govern(command, points_, ages, p_);
    out.twist.linear.x = safe.vx;
    out.twist.linear.y = safe.vy;
    out.twist.angular.z = safe.wz;
    output_->publish(out);
  }

  Params p_;
  double min_height_;
  double max_height_;
  double memory_;
  bool force_tof_only_;
  bool lidar_ok_ = true;
  std::string base_frame_;
  std::string odom_frame_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<Stamped> memory_points_;
  std::vector<Point> scan_points_;
  std::vector<Point> points_;
  std::array<double, 4> seen_;
  double scan_seen_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> tof_subscriptions_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_ok_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_;
};

}  // namespace revd_lidar_safety

RCLCPP_COMPONENTS_REGISTER_NODE(revd_lidar_safety::SafetyGovernor)
