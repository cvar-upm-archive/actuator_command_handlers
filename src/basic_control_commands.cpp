#include "as2_control_command_handlers/basic_control_commands.hpp"

namespace aerostack2
{
  namespace controlCommandsHandlers
  {
    //TODO: fix this bool initializer, so ugly IMPORTANT!!
    BasicControlCommandsHandler::BasicControlCommandsHandler(aerostack2::Node *aerostack2_ptr, bool thrust_is_normalized )
        : node_ptr_(aerostack2_ptr)
    {
      number_of_instances_ ++;
      std::cout<< number_of_instances_ << " instances of BasicControlCommandsHandler created" << std::endl;
      if (rclcpp_node_ptr_ == nullptr){
        std::cout << "Creating rclcpp_node_ptr_" << std::endl;
        rclcpp_node_ptr_ = std::make_shared<rclcpp::Node>("command_handler_service_node");
        set_mode_client_ = rclcpp_node_ptr_->create_client<aerostack2_msgs::srv::SetPlatformControlMode>(node_ptr_->generate_topic_name("set_platform_control_mode"));
      }
      // TODO: STANDARIZE TOPICS NAMES
      command_pose_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(node_ptr_->generate_topic_name("actuator_command/pose"), 10);
      command_twist_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(node_ptr_->generate_topic_name("actuator_command/twist"), 10);
      if (thrust_is_normalized){
        command_thrust_pub_ = node_ptr_->create_publisher<aerostack2_msgs::msg::Thrust>(node_ptr_->generate_topic_name("actuator_command/thrust"), 10);
      }
      else{
        command_thrust_pub_ = node_ptr_->create_publisher<aerostack2_msgs::msg::Thrust>(node_ptr_->generate_topic_name("actuator_command/unnormalized_thrust"), 10);
      }

    };

    BasicControlCommandsHandler::~BasicControlCommandsHandler()
    {
      number_of_instances_--;
      if (number_of_instances_ == 0 && rclcpp_node_ptr_ != nullptr){
        std::cout << "Deleting rclcpp_node_ptr_" << std::endl;
        rclcpp_node_ptr_.reset();
        set_mode_client_.reset();
        }
      // resetting actual_controller_ptr
      if (actual_controller_ptr_ == this)
      {
        actual_controller_ptr_ = nullptr;
      }
    };

    bool BasicControlCommandsHandler::sendCommand()
    {
      if (actual_controller_ptr_ != this)
      {
        std::cout << "BasicControlCommandsHandler::sendCommand() - actual_controller_ptr_ != this" << std::endl;
        if (setMode(setPlatformControlMode()))
        {
          actual_controller_ptr_ = this;

        } 
        else
        {
          RCLCPP_ERROR(node_ptr_->get_logger(), "Cannot set control mode");
          return false;
        }
      }

      if (OwnSetCommands())
      {
        auto stamp = node_ptr_->now();
        command_pose_msg_.header.stamp = stamp;
        command_pose_msg_.header.frame_id = "odom";

        command_twist_msg_.header.stamp = stamp;
        command_twist_msg_.header.frame_id = "odom";

        command_thrust_msg_.header.stamp = stamp;
        command_thrust_msg_.header.frame_id = "base_link";

        command_pose_pub_->publish(command_pose_msg_);
        command_twist_pub_->publish(command_twist_msg_);
        command_thrust_pub_->publish(command_thrust_msg_);
        return true;
      }
      else
      {
        return false;
      }
    };

    void BasicControlCommandsHandler::forceUpdatePlatformControlMode()
    {
      actual_controller_ptr_ = nullptr;
    }

    bool BasicControlCommandsHandler::setMode(const aerostack2_msgs::msg::PlatformControlMode &mode)
    {
      RCLCPP_INFO(node_ptr_->get_logger(), "Setting control mode to %d", mode.control_mode);
      auto request = std::make_shared<aerostack2_msgs::srv::SetPlatformControlMode::Request>();
      request->control_mode = mode;

      RCLCPP_INFO(node_ptr_->get_logger(), "waiting for_service");
      RCLCPP_INFO(node_ptr_->get_logger(), "ptr address=  %x",set_mode_client_.get());
      while (!set_mode_client_->wait_for_service(std::chrono::seconds(1)))
      {
        RCLCPP_INFO(node_ptr_->get_logger(), "waiting for service ok" );
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(rclcpp_node_ptr_->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return false;
        }
        RCLCPP_INFO(rclcpp_node_ptr_->get_logger(), "service not available, waiting again...");
      }
      RCLCPP_INFO(rclcpp_node_ptr_->get_logger(), "service available");

      auto result = set_mode_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(rclcpp_node_ptr_, result,std::chrono::seconds(1)) == rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(rclcpp_node_ptr_->get_logger(), "Platform Control Mode changed sucessfully");
      }
      else
      {
        RCLCPP_ERROR(rclcpp_node_ptr_->get_logger(), " Platform Control Mode was not able to be settled sucessfully");
        return false;
      }
      RCLCPP_INFO(rclcpp_node_ptr_->get_logger(), "service called correctly");
      return true;
    };
    BasicControlCommandsHandler *BasicControlCommandsHandler::actual_controller_ptr_ = nullptr;
    int BasicControlCommandsHandler::number_of_instances_ = 0;
    std::shared_ptr<rclcpp::Node> BasicControlCommandsHandler::rclcpp_node_ptr_ = nullptr;
    rclcpp::Client<aerostack2_msgs::srv::SetPlatformControlMode>::SharedPtr BasicControlCommandsHandler::set_mode_client_ = nullptr;

  } // namespace controlCommandsHandlers
} // namespace aerostack2
