#ifndef SPEED_CONTROL_COMMANDS_HPP
#define SPEED_CONTROL_COMMANDS_HPP

#include <as2_msgs/action/take_off.hpp>
#include <as2_msgs/msg/thrust.hpp>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <thread>

#include "as2_core/node.hpp"
#include "basic_control_commands.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace as2
{
namespace controlCommandsHandlers
{
class SpeedControl : public as2::controlCommandsHandlers::BasicControlCommandsHandler
{
public:
  SpeedControl(as2::Node * node_ptr);
  enum YawMode { NONE, YAW_ANGLE, YAW_RATE };

  bool sendSpeedCommandWithYawAngle(
    const float & vx, const float & vy, const float & vz, const float & yaw_angle);
  bool sendSpeedCommandWithYawAngle(
    const float & vx, const float & vy, const float & vz, const geometry_msgs::msg::Quaternion & q);
  bool sendSpeedCommandWithYawSpeed(
    const float & vx, const float & vy, const float & vz, const float & yaw_speed);

private:
  YawMode yaw_mode_;

  as2_msgs::msg::PlatformControlMode ownSetPlatformControlMode();
};

}  // namespace controlCommandsHandlers
}  // namespace as2

#endif  // SPEED_CONTROL_COMMANDS_HPP