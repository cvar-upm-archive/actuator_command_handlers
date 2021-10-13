#ifndef ACRO_CONTROL_COMMANDS_HPP
#define ACRO_CONTROL_COMMANDS_HPP

#include <memory>
#include <functional>
#include <thread>


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

    class AcroControl : public aerostack2::controlCommandsHandlers::BasicControlCommandsHandler
    {
    public:
      AcroControl(aerostack2::Node *node_ptr);
      enum YawMode
      {
        NONE,
        YAW_ANGLE,
        YAW_RATE
      };

      bool sendAngleRatesWithThrust(const float& x, const float& y, const float& z, const float& thrust);
      bool sendAngleRatesWithNormalizedThrust(const float& x, const float& y, const float& z, const float& thrust, const float& normalized_thrust);

    private:
      aerostack2_msgs::msg::PlatformControlMode setPlatformControlMode();
      bool OwnSetCommands();
    };

  } // namespace controlCommandsHandlers
} // namespace aerostack2

#endif // BASIC_CONTROL_COMMANDS_HPP