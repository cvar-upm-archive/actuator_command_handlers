/*!*******************************************************************************************
 *  \file       basic_actuator_commands.cpp
 *  \brief      basic_actuator_commands implementation file
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

#include "actuator_command_handlers/basic_actuator_commands.hpp"

namespace as2
{
  namespace actuatorCommandsHandlers
  {
    BasicActuatorCommandsHandler::BasicActuatorCommandsHandler(as2::Node *as2_ptr) : node_ptr_(as2_ptr)
    {
      // if (set_mode_client_ptr_.get() == nullptr) {
      set_mode_client_ptr_ =
          std::make_shared<as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>>(
              as2_names::services::platform::set_platform_control_mode);
      // aux_node_ptr_ = std::make_shared<rclcpp::Node>("command_handler_aux_node");
      // set_mode_client_ = aux_node_ptr_->create_client<as2_msgs::srv::SetControlMode>(
      //     node_ptr_->generate_global_name("set_platform_control_mode"));
      // }

      number_of_instances_++;
      RCLCPP_INFO(node_ptr_->get_logger(),
                  "There are %d instances of BasicActuatorCommandsHandler created",
                  number_of_instances_);

      command_pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(
          as2_names::topics::actuator_command::pose, as2_names::topics::actuator_command::qos);
      command_twist_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
          as2_names::topics::actuator_command::twist, as2_names::topics::actuator_command::qos);
      command_thrust_pub_ = node_ptr_->create_publisher<as2_msgs::msg::Thrust>(
          as2_names::topics::actuator_command::thrust, as2_names::topics::actuator_command::qos);
      platform_info_sub_ = node_ptr_->create_subscription<as2_msgs::msg::PlatformInfo>(
          as2_names::topics::platform::info, as2_names::topics::platform::qos,
          [](const as2_msgs::msg::PlatformInfo::SharedPtr msg)
          {
            BasicActuatorCommandsHandler::current_mode_ = msg->current_control_mode;
          });
    };

    BasicActuatorCommandsHandler::~BasicActuatorCommandsHandler()
    {
      number_of_instances_--;
      if (number_of_instances_ == 0)
      {
        set_mode_client_.reset();
        platform_info_sub_.reset();
        set_mode_client_ptr_.reset();
      }
    };

    bool BasicActuatorCommandsHandler::sendCommand()
    {
      static auto last_time = this->node_ptr_->now();

      setControlMode();

      if (this->node_ptr_->now() - last_time > rclcpp::Duration::from_seconds(1.0f / AUX_NODE_SPIN_RATE))
      {
        // rclcpp::spin_some(this->aux_node_ptr_);
        last_time = this->node_ptr_->now();
      }

      if (this->current_mode_ != desired_control_mode_)
      {
        if (!setMode(desired_control_mode_))
        {
          RCLCPP_ERROR(node_ptr_->get_logger(), "Cannot set control mode");
          return false;
        }
      }

      publishCommands();
      return true;
    };

    void BasicActuatorCommandsHandler::publishCommands()
    {
      rclcpp::Time stamp = node_ptr_->now();
      // TODO: Use drone_id_ in odometry_frame_id
      command_pose_msg_.header.stamp = stamp;
      command_pose_msg_.header.frame_id = "odom";

      command_twist_msg_.header.stamp = stamp;
      command_twist_msg_.header.frame_id = "odom";

      command_thrust_msg_.header.stamp = stamp;
      command_thrust_msg_.header.frame_id = "base_link";

      command_pose_pub_->publish(command_pose_msg_);
      command_twist_pub_->publish(command_twist_msg_);
      command_thrust_pub_->publish(command_thrust_msg_);
    }

    bool BasicActuatorCommandsHandler::setMode(const as2_msgs::msg::ControlMode &mode)
    {
      RCLCPP_INFO(node_ptr_->get_logger(), "Setting control mode to %d", mode.control_mode);
      auto request = as2_msgs::srv::SetControlMode::Request();
      auto response = as2_msgs::srv::SetControlMode::Response();
      request.control_mode = mode;
      bool out = set_mode_client_ptr_->sendRequest(request, response);
      if (out && response.success)
      {
        current_mode_ = mode;
        return response.success;
      }
      return false;
    };

    int BasicActuatorCommandsHandler::number_of_instances_ = 0;
    // std::shared_ptr<rclcpp::Node> BasicActuatorCommandsHandler::aux_node_ptr_ = nullptr;
    rclcpp::Client<as2_msgs::srv::SetControlMode>::SharedPtr
        BasicActuatorCommandsHandler::set_mode_client_ = nullptr;
    rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr
        BasicActuatorCommandsHandler::platform_info_sub_ = nullptr;
    as2_msgs::msg::ControlMode BasicActuatorCommandsHandler::current_mode_ =
        as2_msgs::msg::ControlMode();
    // as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>::SharedPtr
    //     BasicActuatorCommandsHandler::set_mode_client_ptr_ = nullptr;

  } // namespace actuatorCommandsHandlers
} // namespace as2
