#ifndef SPEED_CONTROL_COMMANDS_HPP
#define SPEED_CONTROL_COMMANDS_HPP

#include <memory>
#include <functional>
#include <thread>

#include <aerostack2_msgs/action/take_off.hpp>
#include <aerostack2_msgs/msg/thrust.hpp>

#include "aerostack2_core/node.hpp"

#include "basic_control_commands.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace aerostack2
{
  namespace controlCommandsHandlers
  {

    class SpeedControl : public aerostack2::controlCommandsHandlers::BasicControlCommandsHandler
    {
    public:
      SpeedControl(aerostack2::Node *node_ptr);
      enum YawMode
      {
        NONE,
        YAW_ANGLE,
        YAW_RATE
      };

      bool sendSpeedCommandWithYawAngle(const float& vx, const float& vy, const float& vz, const float& yaw_angle);
      bool sendSpeedCommandWithYawAngle(const float& vx, const float& vy, const float& vz, const geometry_msgs::msg::Quaternion& q);
      bool sendSpeedCommandWithYawSpeed(const float& vx, const float& vy, const float& vz, const float& yaw_speed);

    private:
      YawMode yaw_mode_;

      aerostack2_msgs::msg::PlatformControlMode setPlatformControlMode();
      bool OwnSetCommands();
    };

  } // namespace controlCommandsHandlers
} // namespace aerostack2

#endif // SPEED_CONTROL_COMMANDS_HPP