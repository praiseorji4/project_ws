#ifndef MY_ROBOT_HARDWARE_HPP
#define MY_ROBOT_HARDWARE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pigpiod_if2.h> // pigpio library header
#include "pluginlib/class_list_macros.hpp"

namespace my_robot
{
  class MyRobotHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MyRobotHardware);

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  private:
    // Member variables to hold the state and command values
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_commands_;
    std::vector<std::string> joint_names_;
    size_t number_of_joints_;

    // Motor control pins
    int pi_;
    int L_IN1, L_IN2, L_PWM1, L_IN3, L_IN4, L_PWM2;
    int R_IN1, R_IN2, R_PWM1, R_IN3, R_IN4, R_PWM2;
  };
} // namespace my_robot

#endif // MY_ROBOT_HARDWARE_HPP_
