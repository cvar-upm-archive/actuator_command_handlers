#include "as2_control_command_handlers/basic_control_commands.hpp"

namespace as2
{
namespace controlCommandsHandlers
{
BasicControlCommandsHandler::BasicControlCommandsHandler(as2::Node * as2_ptr) : node_ptr_(as2_ptr)
{
  if (number_of_instances_ == 0) {
    command_pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(
      node_ptr_->generate_global_name("actuator_command/pose"), 10);
    command_twist_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
      node_ptr_->generate_global_name("actuator_command/twist"), 10);
    command_thrust_pub_ = node_ptr_->create_publisher<as2_msgs::msg::Thrust>(
      node_ptr_->generate_global_name("actuator_command/thrust"), 10);

    set_mode_client_ = node_ptr_->create_client<as2_msgs::srv::SetPlatformControlMode>(
      node_ptr_->generate_global_name("set_platform_control_mode"));
    platform_info_sub_ = node_ptr_->create_subscription<as2_msgs::msg::PlatformInfo>(
      node_ptr_->generate_global_name("platform/info"), 10,
      [](const as2_msgs::msg::PlatformInfo::SharedPtr msg) {
        BasicControlCommandsHandler::current_mode_ = msg->current_control_mode;
      });
  }

  number_of_instances_++;
  RCLCPP_INFO(
    node_ptr_->get_logger(), "There are %d instances of BasicControlCommandsHandler created",
    number_of_instances_);
};

BasicControlCommandsHandler::~BasicControlCommandsHandler()
{
  number_of_instances_--;
  if (number_of_instances_ == 0 && node_ptr_ != nullptr) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Deleting node_ptr_");
    set_mode_client_.reset();
    platform_info_sub_.reset();
  }
};

bool BasicControlCommandsHandler::sendCommand()
{
  static auto last_time = this->node_ptr_->now();

  setPlatformControlMode();

  if (this->current_mode_ != desired_control_mode_) {
    if (!setMode(desired_control_mode_)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Cannot set control mode");
      return false;
    }
  }

  publishCommands();
  return true;
};

void BasicControlCommandsHandler::publishCommands()
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

bool BasicControlCommandsHandler::setMode(const as2_msgs::msg::PlatformControlMode & mode)
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Setting control mode to %d", mode.control_mode);
  auto request = std::make_shared<as2_msgs::srv::SetPlatformControlMode::Request>();
  request->control_mode = mode;

  // RCLCPP_INFO(node_ptr_->get_logger(), "waiting for_service");
  // RCLCPP_INFO(node_ptr_->get_logger(), "ptr address=  %x", set_mode_client_.get());
  while (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
    // RCLCPP_INFO(node_ptr_->get_logger(), "waiting for service ok");
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "service not available, waiting again...");
  }
  // RCLCPP_INFO(node_ptr_->get_logger(), "service available");

  auto result = set_mode_client_->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(  node_ptr_->get_node_base_interface()
    , result, std::chrono::seconds(1)) ==
    rclcpp::executor::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Platform Control Mode changed sucessfully");
    current_mode_ =
      mode;  // set current_mode_ to the new mode to avoid issues while current_mode_ is not updated
  } else {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), " Platform Control Mode was not able to be settled sucessfully");
    return false;
  }
  // RCLCPP_INFO(node_ptr_->get_logger(), "service called correctly");
  return true;
};

int BasicControlCommandsHandler::number_of_instances_ = 0;
rclcpp::Client<as2_msgs::srv::SetPlatformControlMode>::SharedPtr
  BasicControlCommandsHandler::set_mode_client_ = nullptr;
rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr
  BasicControlCommandsHandler::platform_info_sub_ = nullptr;
as2_msgs::msg::PlatformControlMode BasicControlCommandsHandler::current_mode_ =
  as2_msgs::msg::PlatformControlMode();

}  // namespace controlCommandsHandlers
}  // namespace as2
