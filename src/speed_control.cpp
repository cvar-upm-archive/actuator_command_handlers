
#include "as2_control_command_handlers/speed_control.hpp"

namespace as2
{
namespace controlCommandsHandlers
{
SpeedControl::SpeedControl(as2::Node * node_ptr) : BasicControlCommandsHandler(node_ptr){};

bool SpeedControl::sendSpeedCommandWithYawAngle(
  const float & vx, const float & vy, const float & vz, const float & yaw_angle)
{
  return sendSpeedCommandWithYawAngle(
    vx, vy, vz, tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_angle)));
}

bool SpeedControl::sendSpeedCommandWithYawAngle(
  const float & vx, const float & vy, const float & vz, const geometry_msgs::msg::Quaternion & q)
{
  yaw_mode_ = YAW_ANGLE;
  this->command_twist_msg_.twist.linear.x = vx;
  this->command_twist_msg_.twist.linear.y = vy;
  this->command_twist_msg_.twist.linear.z = vz;
  this->command_pose_msg_.pose.orientation = q;

  return this->sendCommand();
};

bool SpeedControl::sendSpeedCommandWithYawSpeed(
  const float & vx, const float & vy, const float & vz, const float & yaw_speed)
{
  yaw_mode_ = YAW_RATE;
  this->command_twist_msg_.twist.linear.x = vx;
  this->command_twist_msg_.twist.linear.y = vy;
  this->command_twist_msg_.twist.linear.z = vz;
  this->command_twist_msg_.twist.angular.z = yaw_speed;

  return this->sendCommand();
};

as2_msgs::msg::PlatformControlMode SpeedControl::ownSetPlatformControlMode()
{
  as2_msgs::msg::PlatformControlMode platform_control_mode_msg;

  platform_control_mode_msg.control_mode = as2_msgs::msg::PlatformControlMode::SPEED_MODE;
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
