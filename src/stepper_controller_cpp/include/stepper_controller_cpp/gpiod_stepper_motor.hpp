#ifndef STEPPER_CONTROLLER_CPP__GPIOD_STEPPER_MOTOR_HPP_
#define STEPPER_CONTROLLER_CPP__GPIOD_STEPPER_MOTOR_HPP_

#include <gpiod.hpp>
#include <string>
#include <chrono>
#include <stdexcept>
#include <optional> // For optional line_request

namespace stepper_controller_cpp
{

class GpiodStepperMotor
{
public:
    GpiodStepperMotor(
        ::gpiod::chip& chip,
        int step_pin_offset_int,
        int dir_pin_offset_int,
        int enable_pin_offset_int,
        int steps_per_rev,
        double max_frequency_hz,
        const std::string& consumer_name);

    ~GpiodStepperMotor() = default;

    GpiodStepperMotor(const GpiodStepperMotor&) = delete;
    GpiodStepperMotor& operator=(const GpiodStepperMotor&) = delete;
    GpiodStepperMotor(GpiodStepperMotor&&) noexcept = default;
    GpiodStepperMotor& operator=(GpiodStepperMotor&&) noexcept = default;

    void step(int steps, double frequency_hz = 0.0);
    void enable();
    void disable();
    void set_max_frequency(double frequency_hz);
    bool is_enabled() const;
    bool is_valid() const;

private:
    // Initialization Order Corrected
    bool has_enable_pin_;
    int steps_per_revolution_;
    std::chrono::microseconds min_pulse_delay_us_;
    bool currently_enabled_;

    // Offsets (initialized in constructor)
    ::gpiod::line::offset step_pin_;
    ::gpiod::line::offset dir_pin_;
    ::gpiod::line::offset enable_pin_;

    // *** Use std::optional to avoid default construction ***
    std::optional<::gpiod::line_request> motor_request_opt_;

    // Helper
    std::chrono::microseconds frequency_to_delay(double frequency_hz) const;
};

} // namespace stepper_controller_cpp

#endif // STEPPER_CONTROLLER_CPP__GPIOD_STEPPER_MOTOR_HPP_