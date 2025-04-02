#ifndef STEPPER_CONTROLLER_CPP__MOTOR_CONTROLLER_NODE_HPP_
#define STEPPER_CONTROLLER_CPP__MOTOR_CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "stepper_controller_cpp/gpiod_stepper_motor.hpp"
#include <gpiod.hpp> // C++ header
#include <string>
#include <vector>
#include <map>
#include <memory>

namespace stepper_controller_cpp
{

using GpiodChipPtr = std::unique_ptr<::gpiod::chip>;

class MotorControllerNode : public rclcpp::Node
{
public:
    explicit MotorControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~MotorControllerNode() override = default;

private:
    void initialize_gpio_and_comms();
    void parse_motor_parameters();
    void command_callback(const std_msgs::msg::String::SharedPtr msg);

    std::string gpio_chip_device_path_;
    GpiodChipPtr gpio_chip_;

    std::map<std::string, std::unique_ptr<stepper_controller_cpp::GpiodStepperMotor>> motors_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;
};

} // namespace stepper_controller_cpp

#endif // STEPPER_CONTROLLER_CPP__MOTOR_CONTROLLER_NODE_HPP_