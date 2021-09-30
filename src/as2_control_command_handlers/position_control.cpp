
#include "position_control.hpp"

namespace aerostack2
{
  namespace controlCommandsHandlers
  {

    PositionControl::PositionControl(aerostack2::Node *node_ptr)
        : BasicControlCommandsHandler(node_ptr){};

    bool PositionControl::sendPositionCommandWithYawAngle(const float& x, const float& y, const float& z, const float& yaw_angle){
      return sendPositionCommandWithYawAngle(x,y,z,tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_angle)));
    }
    bool PositionControl::sendPositionCommandWithYawAngle(const float& x, const float& y, const float& z, const geometry_msgs::msg::Quaternion& q)
    {
      yaw_mode_ = YAW_ANGLE;
      this->command_pose_msg_.pose.position.x = x;
      this->command_pose_msg_.pose.position.y = y;
      this->command_pose_msg_.pose.position.z = z;
      this->command_pose_msg_.pose.orientation = q;

      return this->sendCommand();
    };

    bool PositionControl::sendPositionCommandWithYawSpeed(const float& x, const float& y, const float& z, const float& yaw_speed)
    {
      yaw_mode_ = YAW_RATE;
      this->command_pose_msg_.pose.position.x = x;
      this->command_pose_msg_.pose.position.y = y;  
      this->command_pose_msg_.pose.position.z = z;
      this->command_twist_msg_.twist.angular.z = yaw_speed;

      return this->sendCommand();
    };

    aerostack2_msgs::msg::PlatformControlMode PositionControl::setPlatformControlMode()
    {
      aerostack2_msgs::msg::PlatformControlMode platform_control_mode_msg;

      platform_control_mode_msg.control_mode = aerostack2_msgs::msg::PlatformControlMode::POSITION_MODE;
      if (yaw_mode_ == YAW_ANGLE)
      {
        platform_control_mode_msg.yaw_mode = aerostack2_msgs::msg::PlatformControlMode::YAW_ANGLE;
      }
      else
      {
        platform_control_mode_msg.yaw_mode = aerostack2_msgs::msg::PlatformControlMode::YAW_SPEED;
      }
      platform_control_mode_msg.reference_frame = aerostack2_msgs::msg::PlatformControlMode::LOCAL_ENU_FRAME;

      return platform_control_mode_msg;
    };

    bool PositionControl::OwnSetCommands()
    {
      static YawMode lastYaWMode = YawMode::NONE;
      if (lastYaWMode != yaw_mode_)
      {
        RCLCPP_INFO(node_ptr_->get_logger(), "Yaw mode changed to %d", yaw_mode_);
        this->forceUpdatePlatformControlMode();
      }
      lastYaWMode = yaw_mode_;
      return true;
    };

  } // namespace controlCommandsHandlers
} // namespace aerostack2
