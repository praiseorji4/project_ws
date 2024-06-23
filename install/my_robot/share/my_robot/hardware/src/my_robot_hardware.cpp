#include "my_robot/my_robot_hardware.hpp"

namespace my_robot
{
    // Define the hardware interface constants if they are not defined
    const std::string HW_IF_POSITION = "position";
    const std::string HW_IF_VELOCITY = "velocity";

    hardware_interface::CallbackReturn MyRobotHardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        number_of_joints_ = info.joints.size();
        hw_positions_.resize(number_of_joints_, 0.0);
        hw_velocities_.resize(number_of_joints_, 0.0);
        hw_commands_.resize(number_of_joints_, 0.0);
        joint_names_.resize(number_of_joints_);

        for (size_t i = 0; i < number_of_joints_; ++i)
        {
            joint_names_[i] = info.joints[i].name;
        }

        // Initialize pigpio
        pi_ = pigpio_start(nullptr, nullptr);
        if (pi_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"), "Failed to initialize pigpio");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Define motor control pins
        L_IN1 = 20;
        L_IN2 = 21;
        L_PWM1 = 0;
        L_IN3 = 22;
        L_IN4 = 23;
        L_PWM2 = 1;
        R_IN1 = 24;
        R_IN2 = 25;
        R_PWM1 = 12;
        R_IN3 = 26;
        R_IN4 = 27;
        R_PWM2 = 13;

        // Set up motor control pins
        set_mode(pi_, L_IN1, PI_OUTPUT);
        set_mode(pi_, L_IN2, PI_OUTPUT);
        set_mode(pi_, L_PWM1, PI_OUTPUT);
        set_mode(pi_, L_IN3, PI_OUTPUT);
        set_mode(pi_, L_IN4, PI_OUTPUT);
        set_mode(pi_, L_PWM2, PI_OUTPUT);
        set_mode(pi_, R_IN1, PI_OUTPUT);
        set_mode(pi_, R_IN2, PI_OUTPUT);
        set_mode(pi_, R_PWM1, PI_OUTPUT);
        set_mode(pi_, R_IN3, PI_OUTPUT);
        set_mode(pi_, R_IN4, PI_OUTPUT);
        set_mode(pi_, R_PWM2, PI_OUTPUT);

        RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface initialized successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MyRobotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
        RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface activated successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MyRobotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface deactivated successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type MyRobotHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)

    {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    for (std::size_t i = 0; i < hw_velocities_.size(); i++)
    {
        // Simulate DiffBot wheels's movement as a first-order system
        // Update the joint status: this is a revolute joint without any limit.
        // Simply integrates
        hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

        RCLCPP_INFO(
            rclcpp::get_logger("MyRobotHardware"),
            "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
            hw_velocities_[i], joint_names_[i].c_str());
    }
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MyRobotHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Write commands to hardware here using pigpio
        int left_front_speed = static_cast<int>(hw_commands_[0] * 255);  // Scale command to PWM range
        int left_rear_speed = static_cast<int>(hw_commands_[1] * 255);   // Scale command to PWM range
        int right_front_speed = static_cast<int>(hw_commands_[2] * 255); // Scale command to PWM range
        int right_rear_speed = static_cast<int>(hw_commands_[3] * 255);  // Scale command to PWM range

        // Left front motor control
        if (left_front_speed >= 0)
        {
            gpio_write(pi_, L_IN1, 0);
            gpio_write(pi_, L_IN2, 1);
        }
        else
        {
            gpio_write(pi_, L_IN1, 1);
            gpio_write(pi_, L_IN2, 0);
        }
        set_PWM_dutycycle(pi_, L_PWM1, abs(left_front_speed));

        // Left rear motor control
        if (left_rear_speed >= 0)
        {
            gpio_write(pi_, L_IN3, 0);
            gpio_write(pi_, L_IN4, 1);
        }
        else
        {
            gpio_write(pi_, L_IN3, 1);
            gpio_write(pi_, L_IN4, 0);
        }
        set_PWM_dutycycle(pi_, L_PWM2, abs(left_rear_speed));

        // Right front motor control
        if (right_front_speed >= 0)
        {
            gpio_write(pi_, R_IN1, 1);
            gpio_write(pi_, R_IN2, 0);
        }
        else
        {
            gpio_write(pi_, R_IN1, 0);
            gpio_write(pi_, R_IN2, 1);
        }
        set_PWM_dutycycle(pi_, R_PWM1, abs(right_front_speed));

        // Right rear motor control
        if (right_rear_speed >= 0)
        {
            gpio_write(pi_, R_IN3, 1);
            gpio_write(pi_, R_IN4, 0);
        }
        else
        {
            gpio_write(pi_, R_IN3, 0);
            gpio_write(pi_, R_IN4, 1);
        }
        set_PWM_dutycycle(pi_, R_PWM2, abs(right_rear_speed));

        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> MyRobotHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[0], HW_IF_POSITION, &hw_positions_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[1], HW_IF_POSITION, &hw_positions_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[2], HW_IF_POSITION, &hw_positions_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[3], HW_IF_POSITION, &hw_positions_[3]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[0], HW_IF_VELOCITY, &hw_velocities_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[1], HW_IF_VELOCITY, &hw_velocities_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[2], HW_IF_VELOCITY, &hw_velocities_[2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[3], HW_IF_VELOCITY, &hw_velocities_[3]));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MyRobotHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[0], HW_IF_VELOCITY, &hw_commands_[0]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[1], HW_IF_VELOCITY, &hw_commands_[1]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[2], HW_IF_VELOCITY, &hw_commands_[2]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[3], HW_IF_VELOCITY, &hw_commands_[3]));

        return command_interfaces;
    }

} // namespace my_robot

// Register the hardware interface as a plugin
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(my_robot::MyRobotHardware, hardware_interface::SystemInterface)
