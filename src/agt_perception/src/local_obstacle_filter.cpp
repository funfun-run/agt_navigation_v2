#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LocalObstacleFilter : public rclcpp::Node
{
public:
  LocalObstacleFilter()
  : Node("agt_local_obstacle_filter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto input = declare_parameter<std::string>(
      "input_topic", "/agt/mapping/registered_points_lidar");
    const auto output = declare_parameter<std::string>(
      "output_topic", "/agt/perception/obstacle_cloud");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    min_height_ = declare_parameter<double>("min_height", 0.08);
    max_height_ = declare_parameter<double>("max_height", 1.8);
    min_range_ = declare_parameter<double>("min_range", 0.35);
    max_range_ = declare_parameter<double>("max_range", 5.0);
    robot_min_x_ = declare_parameter<double>("robot_min_x", -0.5915);
    robot_max_x_ = declare_parameter<double>("robot_max_x", 0.5915);
    robot_min_y_ = declare_parameter<double>("robot_min_y", -0.469);
    robot_max_y_ = declare_parameter<double>("robot_max_y", 0.469);
    transform_timeout_ = declare_parameter<double>("transform_timeout", 0.1);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS(),
      std::bind(&LocalObstacleFilter::filter, this, std::placeholders::_1));
  }

private:
  void filter(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    geometry_msgs::msg::TransformStamped transform_msg;
    try {
      transform_msg = tf_buffer_.lookupTransform(
        target_frame_, cloud->header.frame_id, cloud->header.stamp,
        tf2::durationFromSec(transform_timeout_));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "obstacle cloud transform unavailable: %s",
        error.what());
      return;
    }

    const auto & rotation = transform_msg.transform.rotation;
    const auto & translation = transform_msg.transform.translation;
    const tf2::Transform transform(
      tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
      tf2::Vector3(translation.x, translation.y, translation.z));

    sensor_msgs::msg::PointCloud2 output;
    output.header = cloud->header;
    output.header.frame_id = target_frame_;
    output.height = 1;
    output.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(static_cast<size_t>(cloud->width) * cloud->height);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> input_x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> input_y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> input_z(*cloud, "z");
      sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
      sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
      sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");
      size_t accepted = 0;
      const double min_range_squared = min_range_ * min_range_;
      const double max_range_squared = max_range_ * max_range_;

      for (; input_x != input_x.end(); ++input_x, ++input_y, ++input_z) {
        if (!std::isfinite(*input_x) || !std::isfinite(*input_y) || !std::isfinite(*input_z)) {
          continue;
        }
        const tf2::Vector3 point = transform * tf2::Vector3(*input_x, *input_y, *input_z);
        const double range_squared = point.x() * point.x() + point.y() * point.y();
        const bool inside_robot = point.x() >= robot_min_x_ && point.x() <= robot_max_x_ &&
          point.y() >= robot_min_y_ && point.y() <= robot_max_y_;
        if (point.z() < min_height_ || point.z() > max_height_ ||
          range_squared < min_range_squared || range_squared > max_range_squared || inside_robot)
        {
          continue;
        }
        *output_x = static_cast<float>(point.x());
        *output_y = static_cast<float>(point.y());
        *output_z = static_cast<float>(point.z());
        ++output_x;
        ++output_y;
        ++output_z;
        ++accepted;
      }
      modifier.resize(accepted);
      publisher_->publish(output);
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "invalid PointCloud2 layout: %s", error.what());
    }
  }

  std::string target_frame_;
  double min_height_;
  double max_height_;
  double min_range_;
  double max_range_;
  double robot_min_x_;
  double robot_max_x_;
  double robot_min_y_;
  double robot_max_y_;
  double transform_timeout_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalObstacleFilter>());
  rclcpp::shutdown();
  return 0;
}
