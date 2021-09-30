
#include "speed_control.hpp"

namespace aerostack2
{
  namespace controlCommandsHandlers
  {

    SpeedControl::SpeedControl(aerostack2::Node *node_ptr)
        : BasicControlCommandsHandler(node_ptr){};

    
    bool SpeedControl::sendSpeedCommandWithYawAngle(const float& vx, const float& vy, const float& vz, const float& yaw_angle){
      return sendSpeedCommandWithYawAngle(vx, vy, vz, tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_angle)));
    }

    bool SpeedControl::sendSpeedCommandWithYawAngle(const float& vx, const float& vy, const float& vz, const geometry_msgs::msg::Quaternion& q)
    {
      yaw_mode_ = YAW_ANGLE;
      this->command_twist_msg_.twist.linear.x = vx;
      this->command_twist_msg_.twist.linear.y = vy;
      this->command_twist_msg_.twist.linear.z = vz;
      this->command_pose_msg_.pose.orientation = q;

      return this->sendCommand();
    };

    bool SpeedControl::sendSpeedCommandWithYawSpeed(const float& vx, const float& vy, const float& vz, const float& yaw_speed)
    {
      yaw_mode_ = YAW_RATE;
      this->command_twist_msg_.twist.linear.x = vx;
      this->command_twist_msg_.twist.linear.y = vy;
      this->command_twist_msg_.twist.linear.z = vz;
      this->command_twist_msg_.twist.angular.z = yaw_speed;
             
      return this->sendCommand();
    };

    aerostack2_msgs::msg::PlatformControlMode SpeedControl::setPlatformControlMode()
    {
      aerostack2_msgs::msg::PlatformControlMode platform_control_mode_msg;

      platform_control_mode_msg.control_mode = aerostack2_msgs::msg::PlatformControlMode::SPEED_MODE;
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

    bool SpeedControl::OwnSetCommands()
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
