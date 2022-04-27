#include "as2_control_command_handlers/basic_control_commands.hpp"

#include <as2_core/names/services.hpp>
#include <as2_core/synchronous_service_client.hpp>

namespace as2 {
namespace controlCommandsHandlers {
BasicControlCommandsHandler::BasicControlCommandsHandler(as2::Node *as2_ptr) : node_ptr_(as2_ptr) {
  number_of_instances_++;
  if (number_of_instances_ == 0) {
    set_mode_client_ptr_ =
        std::make_shared<as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>>(
            as2_names::services::platform::set_platform_control_mode);
    // aux_node_ptr_ = std::make_shared<rclcpp::Node>("command_handler_aux_node");
    // set_mode_client_ = aux_node_ptr_->create_client<as2_msgs::srv::SetControlMode>(
    //     node_ptr_->generate_global_name("set_platform_control_mode"));
  }

  RCLCPP_INFO(node_ptr_->get_logger(),
              "There are %d instances of BasicControlCommandsHandler created",
              number_of_instances_);

  command_pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::actuator_command::pose, as2_names::topics::actuator_command::qos);
  command_twist_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::actuator_command::twist, as2_names::topics::actuator_command::qos);
  command_thrust_pub_ = node_ptr_->create_publisher<as2_msgs::msg::Thrust>(
      as2_names::topics::actuator_command::thrust, as2_names::topics::actuator_command::qos);
  platform_info_sub_ = node_ptr_->create_subscription<as2_msgs::msg::PlatformInfo>(
      as2_names::topics::platform::info, as2_names::topics::platform::qos,
      [](const as2_msgs::msg::PlatformInfo::SharedPtr msg) {
        BasicControlCommandsHandler::current_mode_ = msg->current_control_mode;
      });
};

BasicControlCommandsHandler::~BasicControlCommandsHandler() {
  number_of_instances_--;
  if (number_of_instances_ == 0) {
    set_mode_client_.reset();
    platform_info_sub_.reset();
    set_mode_client_ptr_.reset();
  }
};

bool BasicControlCommandsHandler::sendCommand() {
  static auto last_time = this->node_ptr_->now();

  setControlMode();

  if (this->node_ptr_->now() - last_time > rclcpp::Duration(1.0f / AUX_NODE_SPIN_RATE)) {
    // rclcpp::spin_some(this->aux_node_ptr_);
    last_time = this->node_ptr_->now();
  }

  if (this->current_mode_ != desired_control_mode_) {
    if (!setMode(desired_control_mode_)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Cannot set control mode");
      return false;
    }
  }

  publishCommands();
  return true;
};

void BasicControlCommandsHandler::publishCommands() {
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

bool BasicControlCommandsHandler::setMode(const as2_msgs::msg::ControlMode &mode) {
  RCLCPP_INFO(node_ptr_->get_logger(), "Setting control mode to %d", mode.control_mode);
  auto request = std::make_shared<as2_msgs::srv::SetControlMode::Request>();
  request->control_mode = mode;
  auto response = set_mode_client_ptr_->sendRequest(request);
  if (response.success) {
    current_mode_ = mode;
  }
  return response.success;
};

int BasicControlCommandsHandler::number_of_instances_ = 0;
// std::shared_ptr<rclcpp::Node> BasicControlCommandsHandler::aux_node_ptr_ = nullptr;
rclcpp::Client<as2_msgs::srv::SetControlMode>::SharedPtr
    BasicControlCommandsHandler::set_mode_client_ = nullptr;
rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr
    BasicControlCommandsHandler::platform_info_sub_ = nullptr;
as2_msgs::msg::ControlMode BasicControlCommandsHandler::current_mode_ =
    as2_msgs::msg::ControlMode();
as2::SynchronousServiceClient<as2_msgs::srv::SetControlMode>::SharedPtr
    BasicControlCommandsHandler::set_mode_client_ptr_ = nullptr;

}  // namespace controlCommandsHandlers
}  // namespace as2
