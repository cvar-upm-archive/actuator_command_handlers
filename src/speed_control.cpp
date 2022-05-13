/*!*******************************************************************************************
 *  \file       speed_control.cpp
 *  \brief      speed_control implementation file
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "actuator_command_handlers/speed_control.hpp"

namespace as2
{
  namespace actuatorCommandsHandlers
  {
    SpeedControl::SpeedControl(as2::Node *node_ptr) : BasicActuatorCommandsHandler(node_ptr){};

    bool SpeedControl::sendSpeedCommandWithYawAngle(
        const float &vx, const float &vy, const float &vz, const float &yaw_angle)
    {
      return sendSpeedCommandWithYawAngle(
          vx, vy, vz, tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_angle)));
    }

    bool SpeedControl::sendSpeedCommandWithYawAngle(
        const float &vx, const float &vy, const float &vz, const geometry_msgs::msg::Quaternion &q)
    {
      yaw_mode_ = YAW_ANGLE;
      this->command_twist_msg_.twist.linear.x = vx;
      this->command_twist_msg_.twist.linear.y = vy;
      this->command_twist_msg_.twist.linear.z = vz;
      this->command_pose_msg_.pose.orientation = q;

      return this->sendCommand();
    };

    bool SpeedControl::sendSpeedCommandWithYawSpeed(
        const float &vx, const float &vy, const float &vz, const float &yaw_speed)
    {
      yaw_mode_ = YAW_RATE;
      this->command_twist_msg_.twist.linear.x = vx;
      this->command_twist_msg_.twist.linear.y = vy;
      this->command_twist_msg_.twist.linear.z = vz;
      this->command_twist_msg_.twist.angular.z = yaw_speed;

      return this->sendCommand();
    };

    as2_msgs::msg::ControlMode SpeedControl::ownSetControlMode()
    {
      as2_msgs::msg::ControlMode platform_control_mode_msg;

      platform_control_mode_msg.control_mode = as2_msgs::msg::ControlMode::SPEED;
      if (yaw_mode_ == YAW_ANGLE)
      {
        platform_control_mode_msg.yaw_mode = as2_msgs::msg::ControlMode::YAW_ANGLE;
      }
      else
      {
        platform_control_mode_msg.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
      }
      platform_control_mode_msg.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;

      return platform_control_mode_msg;
    };

  } // namespace actuatorCommandsHandlers
} // namespace as2
