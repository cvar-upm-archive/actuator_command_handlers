
#include "as2_control_command_handlers/position_control.hpp"

namespace as2
{
namespace controlCommandsHandlers
{
PositionControl::PositionControl(as2::Node * node_ptr) : BasicControlCommandsHandler(node_ptr){};

bool PositionControl::sendPositionCommandWithYawAngle(
  const float & x, const float & y, const float & z, const float & yaw_angle)
{
  return sendPositionCommandWithYawAngle(
    x, y, z, tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_angle)));
}
bool PositionControl::sendPositionCommandWithYawAngle(
  const float & x, const float & y, const float & z, const geometry_msgs::msg::Quaternion & q)
{
  yaw_mode_ = YAW_ANGLE;
  this->command_pose_msg_.pose.position.x = x;
  this->command_pose_msg_.pose.position.y = y;
  this->command_pose_msg_.pose.position.z = z;
  this->command_pose_msg_.pose.orientation = q;

  return this->sendCommand();
};

bool PositionControl::sendPositionCommandWithYawSpeed(
  const float & x, const float & y, const float & z, const float & yaw_speed)
{
  yaw_mode_ = YAW_RATE;
  this->command_pose_msg_.pose.position.x = x;
  this->command_pose_msg_.pose.position.y = y;
  this->command_pose_msg_.pose.position.z = z;
  this->command_twist_msg_.twist.angular.z = yaw_speed;

  return this->sendCommand();
};

as2_msgs::msg::PlatformControlMode PositionControl::ownSetPlatformControlMode()
{
  as2_msgs::msg::PlatformControlMode platform_control_mode_msg;

  platform_control_mode_msg.control_mode = as2_msgs::msg::PlatformControlMode::POSITION_MODE;
  if (yaw_mode_ == YAW_ANGLE) {
    platform_control_mode_msg.yaw_mode = as2_msgs::msg::PlatformControlMode::YAW_ANGLE;
  } else {
    platform_control_mode_msg.yaw_mode = as2_msgs::msg::PlatformControlMode::YAW_SPEED;
  }
  platform_control_mode_msg.reference_frame = as2_msgs::msg::PlatformControlMode::LOCAL_ENU_FRAME;

  return platform_control_mode_msg;
};

}  // namespace controlCommandsHandlers
}  // namespace as2
