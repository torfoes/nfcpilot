#include "stepper_controller_cpp/motor_controller_node.hpp"

#include "rclcpp/parameter.hpp"
#include "rclcpp/exceptions.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

// Use forced include path if regular one still fails (uncomment ONE line)
// #include "/usr/local/include/gpiod.hpp"
#include <gpiod.hpp>

#include <sstream>
#include <stdexcept>
#include <chrono>
#include <vector>
#include <string>
#include <memory>
#include <utility>
#include <map>
#include <set>

namespace stepper_controller_cpp
{

// --- Constructor ---
MotorControllerNode::MotorControllerNode(const rclcpp::NodeOptions & options)
    : Node("motor_controller_node", options), gpio_chip_(nullptr)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Motor Controller Node using libgpiod C++ API...");
    try {
        initialize_gpio_and_comms();
    } catch (const std::exception & e) {
        RCLCPP_FATAL(this->get_logger(), "CRITICAL FAILURE during node initialization: %s", e.what());
        gpio_chip_.reset(); throw;
    }
}

// --- Initialization ---
void MotorControllerNode::initialize_gpio_and_comms()
{
    RCLCPP_INFO(this->get_logger(), "Declaring parameters and opening GPIO chip...");

    auto chip_desc = rcl_interfaces::msg::ParameterDescriptor{};
    chip_desc.description = "Path to the GPIO chip device (e.g., /dev/gpiochip0)";
    this->declare_parameter<std::string>("gpio_chip_device", "/dev/gpiochip0", chip_desc);

//    // *** Correct Parameter Declaration: Declare namespace only ***
//    rcl_interfaces::msg::ParameterDescriptor motors_desc;
//    motors_desc.description = "Container for individual motor configurations.";
//    this->declare_parameter("motors", rclcpp::ParameterType::PARAMETER_NOT_SET, motors_desc);

    try {
        gpio_chip_device_path_ = this->get_parameter("gpio_chip_device").as_string();
    } catch (const std::exception &e) {
         RCLCPP_ERROR(this->get_logger(), "Failed to get 'gpio_chip_device' parameter: %s", e.what()); throw;
    }
    RCLCPP_INFO(this->get_logger(), "Target GPIO chip device: %s", gpio_chip_device_path_.c_str());

    try {
        // *** Correction: Use constructor directly, no OPEN_BY_PATH enum ***
        gpio_chip_ = std::make_unique<::gpiod::chip>(gpio_chip_device_path_);

        if (!gpio_chip_ || !(*gpio_chip_)) {
             std::string err_msg = "GPIO chip object is null or invalid after opening "; err_msg += gpio_chip_device_path_;
             throw std::runtime_error(err_msg);
        }

        auto info = gpio_chip_->get_info();
         RCLCPP_INFO(this->get_logger(), "Opened GPIO chip: Name='%s', Label='%s', Lines=%zu", // Use %zu for size_t
             info.name().c_str(),
             info.label().c_str(),
             info.num_lines());
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to open GPIO chip '" + gpio_chip_device_path_ + "': " + e.what());
    }

    parse_motor_parameters(); // Parse after chip is confirmed open

    command_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "motor_command", 10, std::bind(&MotorControllerNode::command_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Command subscriber created.");
}

// --- Motor Parameter Parsing ---
void MotorControllerNode::parse_motor_parameters()
{
     RCLCPP_INFO(this->get_logger(), "Parsing motor parameters...");
     if (!gpio_chip_ || !(*gpio_chip_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot parse motor parameters: GPIO chip is not valid."); return;
     }

    std::set<std::string> motor_name_set;
    try{
        auto param_prefixes = this->list_parameters({"motors"}, 1).prefixes;
        std::string motor_prefix = "motors.";
        for (const auto & prefix : param_prefixes) {
            if (prefix.length() > motor_prefix.length() && prefix.substr(0, motor_prefix.length()) == motor_prefix) {
                 std::string name = prefix.substr(motor_prefix.length());
                 if (!name.empty() && name.back() == '.') name.pop_back();
                 if (!name.empty()) motor_name_set.insert(name);
            }
        }
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& ) {
        RCLCPP_WARN(this->get_logger(), "'motors' parameter namespace not found or empty."); return;
    }

    if (motor_name_set.empty()) {
         RCLCPP_WARN(this->get_logger(), "No motor configurations found under 'motors' parameter namespace."); return;
    }

    std::vector<std::string> motor_names(motor_name_set.begin(), motor_name_set.end());
    RCLCPP_INFO(this->get_logger(), "Found %zu potential motor configurations:", motor_names.size());
    for(const auto& name : motor_names) { RCLCPP_INFO(this->get_logger(), "- %s", name.c_str()); }

    // *** Correction: Get num_lines via chip_info ***
    unsigned int num_chip_lines = gpio_chip_->get_info().num_lines();
    size_t successful_motors = 0;

    for (const auto& motor_name : motor_names) {
         RCLCPP_INFO(this->get_logger(), "--- Configuring motor: '%s' ---", motor_name.c_str());
         std::string param_base = "motors." + motor_name + ".";
         try {
            rcl_interfaces::msg::ParameterDescriptor pin_desc; pin_desc.description = "GPIO line offset";
            rcl_interfaces::msg::ParameterDescriptor spr_desc; spr_desc.description = "Steps per revolution";
            rcl_interfaces::msg::ParameterDescriptor freq_desc; freq_desc.description = "Max step frequency (Hz)";

            // Declare params with defaults
            this->declare_parameter<int>(param_base + "step_pin", -1, pin_desc);
            this->declare_parameter<int>(param_base + "dir_pin", -1, pin_desc);
            this->declare_parameter<int>(param_base + "enable_pin", -1, pin_desc);
            this->declare_parameter<int>(param_base + "steps_per_revolution", 200, spr_desc);
            this->declare_parameter<double>(param_base + "max_step_frequency_hz", 1000.0, freq_desc); // Use double

            // Get values
            int step_pin = this->get_parameter(param_base + "step_pin").as_int();
            int dir_pin = this->get_parameter(param_base + "dir_pin").as_int();
            int enable_pin = this->get_parameter(param_base + "enable_pin").as_int();
            int steps_per_rev = this->get_parameter(param_base + "steps_per_revolution").as_int();
            double max_freq = this->get_parameter(param_base + "max_step_frequency_hz").as_double();

            // Validation
            if (step_pin < 0 || dir_pin < 0 ) throw std::runtime_error("step/dir pins required.");
            if (static_cast<unsigned int>(step_pin) >= num_chip_lines || static_cast<unsigned int>(dir_pin) >= num_chip_lines || (enable_pin >= 0 && static_cast<unsigned int>(enable_pin) >= num_chip_lines)) {
                 throw std::runtime_error("Pin offset out of bounds.");
            }
            if (steps_per_rev <= 0) throw std::runtime_error("Steps must be positive.");
            if (max_freq <= 0.0) { max_freq = 1.0; /* Warning in motor class */ }

            RCLCPP_INFO(this->get_logger(), "  Params validated: Step=%d, Dir=%d, En=%d, MaxFreq=%.2f Hz",
                step_pin, dir_pin, enable_pin, max_freq);

            // Create Motor Object
            std::string consumer = std::string(this->get_name()) + "_" + motor_name;
            auto motor = std::make_unique<GpiodStepperMotor>(
                *gpio_chip_, step_pin, dir_pin, enable_pin, steps_per_rev,
                max_freq, consumer
            );

            if (!motor->is_valid()) throw std::runtime_error("GPIO request failed internally.");

            motors_[motor_name] = std::move(motor);
            RCLCPP_INFO(this->get_logger(), "Successfully configured motor '%s'", motor_name.c_str());
            successful_motors++;

         } catch (const std::exception &e) {
             RCLCPP_ERROR(this->get_logger(), "Failed to configure motor '%s': %s. Skipping.", motor_name.c_str(), e.what());
         }
    }

    RCLCPP_INFO(this->get_logger(), "Finished motor config: %zu/%zu successful.", successful_motors, motor_names.size());
    if (successful_motors == 0 && !motor_names.empty()) {
         RCLCPP_ERROR(this->get_logger(), "Failed to initialize ANY motors!");
    }
}

// --- Command Callback ---
void MotorControllerNode::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // ... (Callback unchanged) ...
    RCLCPP_DEBUG(this->get_logger(), "Received command: '%s'", msg->data.c_str());
    std::string command_str = msg->data;
    std::stringstream ss(command_str);
    std::string motor_name, steps_str, freq_str;
    double frequency = 0.0;

    if (!std::getline(ss, motor_name, ':') || !std::getline(ss, steps_str)) {
         RCLCPP_ERROR(this->get_logger(), "Invalid command format: '%s'. Expected 'motor_name:steps[:frequency]'.", command_str.c_str()); return;
    }
    if (std::getline(ss, freq_str, ':')) {
        try { frequency = std::stod(freq_str); }
        catch (const std::exception& e) {
             RCLCPP_ERROR(this->get_logger(), "Invalid frequency value '%s': %s", freq_str.c_str(), e.what()); return;
        }
    }

    int steps = 0;
    try { steps = std::stoi(steps_str); }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid steps value '%s': %s", steps_str.c_str(), e.what()); return;
    }

    auto it = motors_.find(motor_name);
    if (it == motors_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Unknown motor: '%s'", motor_name.c_str()); return;
    }

    GpiodStepperMotor* motor_ptr = it->second.get();
    if (!motor_ptr || !motor_ptr->is_valid()) {
         RCLCPP_ERROR(this->get_logger(), "Motor '%s' not valid.", motor_name.c_str()); return;
    }

    try {
        if (frequency > 0.0) { RCLCPP_INFO(this->get_logger(), "Executing: Motor '%s', Steps %d @ %.2f Hz", motor_name.c_str(), steps, frequency); }
        else { RCLCPP_INFO(this->get_logger(), "Executing: Motor '%s', Steps %d @ Max Speed", motor_name.c_str(), steps); }
        motor_ptr->step(steps, frequency);
        RCLCPP_INFO(this->get_logger(), "Finished: Motor '%s', Steps %d", motor_name.c_str(), steps);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error stepping motor '%s': %s", motor_name.c_str(), e.what());
        try { if (motor_ptr->is_enabled()) motor_ptr->disable(); } catch (...) {}
    }
}

} // namespace stepper_controller_cpp