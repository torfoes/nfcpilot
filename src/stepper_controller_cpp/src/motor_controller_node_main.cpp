// ... (main function unchanged) ...
#include "rclcpp/rclcpp.hpp"
#include "stepper_controller_cpp/motor_controller_node.hpp"
#include <memory>
#include <exception>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  std::shared_ptr<stepper_controller_cpp::MotorControllerNode> node = nullptr;
  int return_code = 0;

  try {
      node = std::make_shared<stepper_controller_cpp::MotorControllerNode>(options);
      RCLCPP_INFO(rclcpp::get_logger("motor_controller_main"), "Starting motor controller node spin.");
      rclcpp::spin(node);
  } catch (const std::exception & e) {
      RCLCPP_FATAL(rclcpp::get_logger("motor_controller_main"), "Caught exception during node initialization or spin: %s", e.what());
      return_code = 1;
  } catch (...) {
       RCLCPP_FATAL(rclcpp::get_logger("motor_controller_main"), "Caught unknown exception during node initialization or spin.");
       return_code = 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("motor_controller_main"), "Shutting down ROS.");
  rclcpp::shutdown();
  return return_code;
}