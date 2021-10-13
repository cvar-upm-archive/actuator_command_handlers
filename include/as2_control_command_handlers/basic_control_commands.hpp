#ifndef BASIC_CONTROL_COMMANDS_HPP 
#define BASIC_CONTROL_COMMANDS_HPP

#include <memory>
#include <functional>
#include <thread>

#include "aerostack2_core/node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <aerostack2_msgs/msg/thrust.hpp>
#include <aerostack2_msgs/srv/set_platform_control_mode.hpp>


namespace aerostack2{

namespace controlCommandsHandlers{

class BasicControlCommandsHandler{

public:
    BasicControlCommandsHandler(aerostack2::Node* aerostack2_ptr);
    ~BasicControlCommandsHandler();
protected:

  virtual aerostack2_msgs::msg::PlatformControlMode setPlatformControlMode() = 0;
  virtual bool OwnSetCommands()=0;
  
  bool sendCommand();
  void forceUpdatePlatformControlMode();

  geometry_msgs::msg::PoseStamped   command_pose_msg_;
  geometry_msgs::msg::TwistStamped  command_twist_msg_;
  aerostack2_msgs::msg::Thrust      command_thrust_msg_;
    
  aerostack2::Node* node_ptr_;

private:
  static BasicControlCommandsHandler* actual_controller_ptr_;
  static int number_of_instances_;
  static std::shared_ptr<rclcpp::Node> rclcpp_node_ptr_;
  static rclcpp::Client<aerostack2_msgs::srv::SetPlatformControlMode>::SharedPtr set_mode_client_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr command_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_twist_pub_;
  rclcpp::Publisher<aerostack2_msgs::msg::Thrust>::SharedPtr command_thrust_pub_;
  

  bool setMode(const aerostack2_msgs::msg::PlatformControlMode& mode);
  
};


} // namespace controlCommandsHandlers
} // namespace aerostack2

#endif // BASIC_CONTROL_COMMANDS_HPP