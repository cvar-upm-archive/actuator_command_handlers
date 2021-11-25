#ifndef ACRO_CONTROL_COMMANDS_HPP
#define ACRO_CONTROL_COMMANDS_HPP

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
class AcroControl : public as2::controlCommandsHandlers::BasicControlCommandsHandler
{
public:
  AcroControl(as2::Node * node_ptr);
  enum YawMode { NONE, YAW_ANGLE, YAW_RATE };

  bool sendAngleRatesWithThrust(
    const float & x, const float & y, const float & z, const float & thrust);
  bool sendAngleRatesWithNormalizedThrust(
    const float & x, const float & y, const float & z, const float & thrust,
    const float & normalized_thrust);

private:
  as2_msgs::msg::PlatformControlMode setPlatformControlMode();
};

}  // namespace controlCommandsHandlers
}  // namespace as2

#endif  // BASIC_CONTROL_COMMANDS_HPP