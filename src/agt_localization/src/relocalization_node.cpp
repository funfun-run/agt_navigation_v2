#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#ifdef _OPENMP
#include <omp.h>
#endif

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "relocalization_core/relocalizer.hpp"
#include "agt_localization/ros_conversions.hpp"

namespace
{

using relocalization_core::CloudT;
using relocalization_core::PointT;

int ompMaxThreads()
{
#ifdef _OPENMP
  return omp_get_max_threads();
#else
  return 1;
#endif
}

std::string statusText(
  const relocalization_core::RelocalizationResult & result,
  const relocalization_core::RelocalizationDebugInfo & debug_info)
{
  return
    "status=" + relocalization_core::toString(result.status_code) +
    " backend=" + relocalization_core::toString(result.backend) +
    " converged=" + std::string(result.has_converged ? "true" : "false") +
    " fitness=" + std::to_string(result.fitness_score) +
    " iterations=" + std::to_string(result.iterations) +
    " raw_scan=" + std::to_string(debug_info.raw_scan_size) +
    " cropped_scan=" + std::to_string(debug_info.cropped_scan_size) +
    " runtime_ms=" + std::to_string(debug_info.backend_runtime_ms) +
    " message=" + result.status_message;
}

}  // namespace

class RelocalizationNode : public rclcpp::Node
{
public:
  RelocalizationNode()
  : Node("relocalization_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    global_map_pcd_ = declare_parameter<std::string>("global_map_pcd", "");
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    tracking_frame_ = declare_parameter<std::string>("tracking_frame", "lidar_link");
    cloud_topic_ = declare_parameter<std::string>(
      "cloud_topic", "/agt/mapping/registered_points_lidar");
    initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", "/initialpose");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_aligned_cloud_ = declare_parameter<bool>("publish_aligned_cloud", true);

    relocalization_core::RelocalizerConfig config;
    config.backend = parseBackend(
      declare_parameter<std::string>("backend", "ndt"));
    config.map_voxel_leaf_size = declare_parameter<double>("map_voxel_leaf_size", 0.25);
    config.scan_voxel_leaf_size = declare_parameter<double>("scan_voxel_leaf_size", 0.25);
    config.min_scan_points = declare_parameter<int>("min_scan_points", 200);
    config.fitness_score_threshold = declare_parameter<double>("fitness_score_threshold", 2.0);
    config.max_iterations = declare_parameter<int>("max_iterations", 100);
    config.transform_epsilon = declare_parameter<double>("transform_epsilon", 1e-6);
    config.euclidean_fitness_epsilon =
      declare_parameter<double>("euclidean_fitness_epsilon", 1e-6);
    config.max_correspondence_distance =
      declare_parameter<double>("max_correspondence_distance", 3.0);
    config.crop_box.enabled = declare_parameter<bool>("crop_box_enabled", true);
    config.crop_box.frame_mode = parseCropMode(
      declare_parameter<std::string>("crop_box_frame_mode", "scan_local"));
    config.crop_box.x_min = declare_parameter<double>("crop_x_min", 0.0);
    config.crop_box.x_max = declare_parameter<double>("crop_x_max", 30.0);
    config.crop_box.y_min = declare_parameter<double>("crop_y_min", -15.0);
    config.crop_box.y_max = declare_parameter<double>("crop_y_max", 15.0);
    config.crop_box.z_min = declare_parameter<double>("crop_z_min", -2.0);
    config.crop_box.z_max = declare_parameter<double>("crop_z_max", 2.0);
    config.ndt.resolution = declare_parameter<double>("ndt_resolution", 1.0);
    config.ndt.step_size = declare_parameter<double>("ndt_step_size", 0.1);
    config.ndt.num_threads = declare_parameter<int>("ndt_num_threads", ompMaxThreads());
    config.ndt.search_method = parseNdtSearchMethod(
      declare_parameter<std::string>("ndt_search_method", "DIRECT7"));

    relocalizer_.setConfig(config);
    if (!global_map_pcd_.empty()) {
      if (!relocalizer_.setGlobalMapFromPcd(global_map_pcd_, global_frame_)) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to load global_map_pcd=%s at startup",
          global_map_pcd_.c_str());
      }
    }

    status_pub_ = create_publisher<std_msgs::msg::String>("/agt/localization/status", 10);
    aligned_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
      "/agt/localization/aligned_points", rclcpp::SensorDataQoS());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&RelocalizationNode::publishLatestTf, this));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RelocalizationNode::cloudCallback, this, std::placeholders::_1));
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, 10,
      std::bind(&RelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Relocalization node ready. backend=%s cloud_topic=%s initialpose_topic=%s global_map_pcd=%s",
      relocalization_core::toString(relocalizer_.config().backend).c_str(),
      cloud_topic_.c_str(), initialpose_topic_.c_str(), global_map_pcd_.c_str());
  }

private:
  relocalization_core::RegistrationBackendType parseBackend(const std::string & backend) const
  {
    if (backend == "icp") {
      return relocalization_core::RegistrationBackendType::kIcp;
    }
    return relocalization_core::RegistrationBackendType::kNdt;
  }

  relocalization_core::CropBoxFrameMode parseCropMode(const std::string & mode) const
  {
    if (mode == "disabled") {
      return relocalization_core::CropBoxFrameMode::kDisabled;
    }
    return relocalization_core::CropBoxFrameMode::kScanLocal;
  }

  int parseNdtSearchMethod(const std::string & method) const
  {
    if (method == "KDTREE") {
      return 0;
    }
    if (method == "DIRECT26") {
      return 1;
    }
    if (method == "DIRECT1") {
      return 3;
    }
    return 2;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_msg_ = msg;
  }

  relocalization_core::CloudPtr cloudFromMsgInTrackingFrame(
    const sensor_msgs::msg::PointCloud2 & msg)
  {
    relocalization_core::CloudPtr cloud(new CloudT());
    pcl::fromROSMsg(msg, *cloud);

    if (msg.header.frame_id.empty() || msg.header.frame_id == tracking_frame_) {
      return cloud;
    }

    const auto tracking_from_cloud_msg = tf_buffer_.lookupTransform(
      tracking_frame_, msg.header.frame_id, rclcpp::Time(msg.header.stamp),
      std::chrono::milliseconds(200));
    const Eigen::Matrix4f tracking_from_cloud =
      agt_localization::transformMsgToEigen(tracking_from_cloud_msg);
    relocalization_core::CloudPtr transformed(new CloudT());
    pcl::transformPointCloud(*cloud, *transformed, tracking_from_cloud);
    return transformed;
  }

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud_msg = latest_cloud_msg_;
    }

    if (!cloud_msg) {
      publishStatus("status=scan_too_small message=no latest cloud");
      return;
    }

    if (!relocalizer_.hasMap() && !global_map_pcd_.empty()) {
      if (!relocalizer_.setGlobalMapFromPcd(global_map_pcd_, global_frame_)) {
        publishStatus("status=map_not_ready message=failed to load map");
        return;
      }
    }

    relocalization_core::CloudPtr scan_cloud;
    try {
      scan_cloud = cloudFromMsgInTrackingFrame(*cloud_msg);
    } catch (const tf2::TransformException & ex) {
      publishStatus("status=backend_failed message=cloud transform lookup failed");
      RCLCPP_WARN(get_logger(), "Failed to transform scan to tracking frame: %s", ex.what());
      return;
    }

    relocalization_core::RelocalizationRequest request;
    request.source_cloud = scan_cloud;
    request.source_frame_id = tracking_frame_;
    request.target_frame_id = global_frame_;
    Eigen::Matrix4f base_from_tracking = Eigen::Matrix4f::Identity();
    try {
      const auto base_from_tracking_msg = tf_buffer_.lookupTransform(
        base_frame_, tracking_frame_, tf2::TimePointZero, std::chrono::milliseconds(200));
      base_from_tracking = agt_localization::transformMsgToEigen(base_from_tracking_msg);
    } catch (const tf2::TransformException & ex) {
      publishStatus("status=backend_failed message=base to tracking lookup failed");
      RCLCPP_WARN(get_logger(), "Failed to lookup base->tracking: %s", ex.what());
      return;
    }

    // RViz initialpose is map_from_base; registration requires map_from_tracking.
    request.initial_guess =
      agt_localization::poseMsgToEigen(msg->pose.pose) * base_from_tracking;
    request.request_time_sec = now().seconds();
    request.enable_debug_outputs = publish_aligned_cloud_;

    const auto result = relocalizer_.relocalize(request);
    const auto debug_info = relocalizer_.latestDebugInfo();
    publishStatus(statusText(result, debug_info));

    if (!result.success) {
      RCLCPP_WARN(
        get_logger(), "Relocalization failed. status=%s fitness=%.4f message=%s",
        relocalization_core::toString(result.status_code).c_str(), result.fitness_score,
        result.status_message.c_str());
      return;
    }

    const rclcpp::Time stamp = now();
    Eigen::Matrix4f tracking_in_odom = Eigen::Matrix4f::Identity();
    try {
      const auto odom_to_tracking_msg = tf_buffer_.lookupTransform(
        odom_frame_, tracking_frame_, tf2::TimePointZero, std::chrono::milliseconds(200));
      tracking_in_odom = agt_localization::transformMsgToEigen(odom_to_tracking_msg).inverse();
    } catch (const tf2::TransformException & ex) {
      publishStatus("status=backend_failed message=odom to tracking lookup failed");
      RCLCPP_WARN(get_logger(), "Failed to lookup odom->tracking: %s", ex.what());
      return;
    }

    const Eigen::Matrix4f map_to_odom = result.estimated_pose * tracking_in_odom;
    {
      std::lock_guard<std::mutex> lock(tf_mutex_);
      latest_map_to_odom_ = map_to_odom;
      has_latest_tf_ = true;
    }

    if (publish_tf_) {
      tf_broadcaster_->sendTransform(
        agt_localization::eigenToTransformMsg(map_to_odom, stamp, global_frame_, odom_frame_));
    }

    if (publish_aligned_cloud_ && result.aligned_cloud) {
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*result.aligned_cloud, aligned_msg);
      aligned_msg.header.stamp = stamp;
      aligned_msg.header.frame_id = global_frame_;
      aligned_cloud_pub_->publish(aligned_msg);
    }

    RCLCPP_INFO(
      get_logger(), "Relocalization succeeded. backend=%s fitness=%.4f iterations=%d",
      relocalization_core::toString(result.backend).c_str(), result.fitness_score,
      result.iterations);
  }

  void publishLatestTf()
  {
    if (!publish_tf_) {
      return;
    }

    Eigen::Matrix4f map_to_odom = Eigen::Matrix4f::Identity();
    {
      std::lock_guard<std::mutex> lock(tf_mutex_);
      if (!has_latest_tf_) {
        return;
      }
      map_to_odom = latest_map_to_odom_;
    }

    tf_broadcaster_->sendTransform(
      agt_localization::eigenToTransformMsg(map_to_odom, now(), global_frame_, odom_frame_));
  }

  void publishStatus(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  relocalization_core::Relocalizer relocalizer_;
  std::string global_map_pcd_;
  std::string global_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string tracking_frame_;
  std::string cloud_topic_;
  std::string initialpose_topic_;
  bool publish_tf_{true};
  bool publish_aligned_cloud_{true};

  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_msg_;
  std::mutex cloud_mutex_;

  Eigen::Matrix4f latest_map_to_odom_{Eigen::Matrix4f::Identity()};
  bool has_latest_tf_{false};
  std::mutex tf_mutex_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
