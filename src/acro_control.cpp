
#include "as2_control_command_handlers/acro_control.hpp"

namespace as2
{
namespace controlCommandsHandlers
{
AcroControl::AcroControl(as2::Node * node_ptr) : BasicControlCommandsHandler(node_ptr){};

bool AcroControl::sendAngleRatesWithThrust(
  const float & x, const float & y, const float & z, const float & thrust)
{
  this->command_twist_msg_.twist.angular.x = x;
  this->command_twist_msg_.twist.angular.y = y;
  this->command_twist_msg_.twist.angular.z = z;
  this->command_thrust_msg_.thrust = thrust;
  return this->sendCommand();
};
bool AcroControl::sendAngleRatesWithNormalizedThrust(
  const float & x, const float & y, const float & z, const float & thrust,
  const float & normalized_thrust)
{
  this->command_twist_msg_.twist.angular.x = x;
  this->command_twist_msg_.twist.angular.y = y;
  this->command_twist_msg_.twist.angular.z = z;
  this->command_thrust_msg_.thrust = thrust;
  this->command_thrust_msg_.thrust_normalized = normalized_thrust;
  return this->sendCommand();
};

as2_msgs::msg::PlatformControlMode AcroControl::setPlatformControlMode()
{
  as2_msgs::msg::PlatformControlMode platform_control_mode_msg;

  platform_control_mode_msg.control_mode = as2_msgs::msg::PlatformControlMode::ACRO_MODE;
  platform_control_mode_msg.yaw_mode = as2_msgs::msg::PlatformControlMode::YAW_SPEED;
  platform_control_mode_msg.reference_frame = as2_msgs::msg::PlatformControlMode::BODY_FLU_FRAME;

  return platform_control_mode_msg;
};


}  // namespace controlCommandsHandlers
}  // namespace as2
