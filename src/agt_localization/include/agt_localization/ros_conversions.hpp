#ifndef AGT_LOCALIZATION__ROS_CONVERSIONS_HPP_
#define AGT_LOCALIZATION__ROS_CONVERSIONS_HPP_

#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace agt_localization
{

inline Eigen::Matrix4f poseMsgToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaternionf quat(
    static_cast<float>(pose.orientation.w),
    static_cast<float>(pose.orientation.x),
    static_cast<float>(pose.orientation.y),
    static_cast<float>(pose.orientation.z));
  quat.normalize();

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = quat.toRotationMatrix();
  transform(0, 3) = static_cast<float>(pose.position.x);
  transform(1, 3) = static_cast<float>(pose.position.y);
  transform(2, 3) = static_cast<float>(pose.position.z);
  return transform;
}

inline Eigen::Matrix4f transformMsgToEigen(const geometry_msgs::msg::TransformStamped & msg)
{
  Eigen::Quaternionf quat(
    static_cast<float>(msg.transform.rotation.w),
    static_cast<float>(msg.transform.rotation.x),
    static_cast<float>(msg.transform.rotation.y),
    static_cast<float>(msg.transform.rotation.z));
  quat.normalize();

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = quat.toRotationMatrix();
  transform(0, 3) = static_cast<float>(msg.transform.translation.x);
  transform(1, 3) = static_cast<float>(msg.transform.translation.y);
  transform(2, 3) = static_cast<float>(msg.transform.translation.z);
  return transform;
}

inline geometry_msgs::msg::TransformStamped eigenToTransformMsg(
  const Eigen::Matrix4f & transform,
  const rclcpp::Time & stamp,
  const std::string & parent_frame,
  const std::string & child_frame)
{
  const Eigen::Quaternionf quat(transform.block<3, 3>(0, 0));

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent_frame;
  msg.child_frame_id = child_frame;
  msg.transform.translation.x = transform(0, 3);
  msg.transform.translation.y = transform(1, 3);
  msg.transform.translation.z = transform(2, 3);
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  msg.transform.rotation.w = quat.w();
  return msg;
}

}  // namespace agt_localization

#endif  // AGT_LOCALIZATION__ROS_CONVERSIONS_HPP_
