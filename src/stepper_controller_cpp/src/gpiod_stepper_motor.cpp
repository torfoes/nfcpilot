#include "stepper_controller_cpp/gpiod_stepper_motor.hpp"

#include <vector>
#include <cmath>
#include <thread>
#include <stdexcept>
#include <string>
#include <system_error>
#include <iostream>
#include <limits>
#include <optional> // Include optional

#include <gpiod.hpp>

namespace stepper_controller_cpp
{
const std::chrono::microseconds MIN_ALLOWABLE_DELAY_US(10);
const double MAX_ALLOWABLE_FREQUENCY = 1.0e6 / MIN_ALLOWABLE_DELAY_US.count();

// --- Helper ---
std::chrono::microseconds GpiodStepperMotor::frequency_to_delay(double frequency_hz) const {
    if (frequency_hz <= 0.0) {
        return std::chrono::microseconds(std::numeric_limits<long long>::max());
    }
    if (frequency_hz > MAX_ALLOWABLE_FREQUENCY) {
        frequency_hz = MAX_ALLOWABLE_FREQUENCY; // Clamp
    }
    long long delay_us = static_cast<long long>((1.0e6 / frequency_hz) + 0.5);
    return std::chrono::microseconds(std::max((long long)MIN_ALLOWABLE_DELAY_US.count(), delay_us));
}

// --- Constructor ---
GpiodStepperMotor::GpiodStepperMotor(
    ::gpiod::chip& chip,
    int step_pin_offset_int,
    int dir_pin_offset_int,
    int enable_pin_offset_int,
    int steps_per_rev,
    double max_frequency_hz,
    const std::string& consumer_name)
    : has_enable_pin_(enable_pin_offset_int >= 0),
      steps_per_revolution_(steps_per_rev),
      min_pulse_delay_us_(frequency_to_delay(max_frequency_hz)),
      currently_enabled_(!has_enable_pin_),
      // Initialize offsets safely
      step_pin_(step_pin_offset_int >= 0 ? step_pin_offset_int : 0),
      dir_pin_(dir_pin_offset_int >= 0 ? dir_pin_offset_int : 0),
      enable_pin_(has_enable_pin_ && enable_pin_offset_int >= 0 ? enable_pin_offset_int : 0),
      motor_request_opt_(std::nullopt) // Initialize optional as empty
{
    if (steps_per_revolution_ <= 0) throw std::invalid_argument("Steps must be positive");
    if (step_pin_offset_int < 0 || dir_pin_offset_int < 0) throw std::invalid_argument("Pin offsets must be non-negative");

    try {
        auto builder = chip.prepare_request();
        builder.set_consumer(consumer_name);

        builder.add_line_settings(
            step_pin_,
            ::gpiod::line_settings()
                .set_direction(::gpiod::line::direction::OUTPUT)
                .set_output_value(::gpiod::line::value::INACTIVE));
        builder.add_line_settings(
            dir_pin_,
            ::gpiod::line_settings()
                .set_direction(::gpiod::line::direction::OUTPUT)
                .set_output_value(::gpiod::line::value::INACTIVE));
        if (has_enable_pin_) {
            builder.add_line_settings(
                enable_pin_,
                ::gpiod::line_settings()
                    .set_direction(::gpiod::line::direction::OUTPUT)
                    .set_output_value(::gpiod::line::value::ACTIVE));
        }

        // *** Emplace the request object into the optional ***
        motor_request_opt_.emplace(builder.do_request());

        // Check validity after emplacing
        if (!motor_request_opt_ || !(*motor_request_opt_)) {
             motor_request_opt_.reset(); // Ensure it's empty on failure
             throw std::runtime_error("GPIO line request failed.");
        }

    } catch (const std::exception& e) {
        motor_request_opt_.reset(); // Ensure empty on any exception
        throw std::runtime_error("GPIO setup failed for motor (" + consumer_name + "): " + e.what());
    }
}


// --- Validity Check ---
bool GpiodStepperMotor::is_valid() const {
    // Check if optional holds a value AND that value is valid
    return motor_request_opt_.has_value() && static_cast<bool>(*motor_request_opt_);
}

// --- Public Methods ---
void GpiodStepperMotor::set_max_frequency(double frequency_hz) {
    min_pulse_delay_us_ = frequency_to_delay(frequency_hz);
}

void GpiodStepperMotor::enable() {
    if (!is_valid()) throw std::logic_error("Motor invalid.");
    // Access request via optional's operator* or value()
    if (has_enable_pin_ && !currently_enabled_) {
        try {
            motor_request_opt_->set_value(enable_pin_, ::gpiod::line::value::INACTIVE);
            currently_enabled_ = true;
        } catch (const std::exception &e) { throw std::runtime_error("Enable failed: " + std::string(e.what())); }
    }
}

void GpiodStepperMotor::disable() {
     if (!is_valid()) { std::cerr << "Warning: Disable called on invalid motor." << std::endl; return; }
     if (has_enable_pin_ && currently_enabled_) {
        try {
            motor_request_opt_->set_value(enable_pin_, ::gpiod::line::value::ACTIVE);
            currently_enabled_ = false;
        } catch (const std::exception &e) { throw std::runtime_error("Disable failed: " + std::string(e.what())); }
    }
}

bool GpiodStepperMotor::is_enabled() const {
    return currently_enabled_;
}

void GpiodStepperMotor::step(int steps, double frequency_hz) {
    if (steps == 0) return;
    if (!is_valid()) throw std::logic_error("Motor invalid.");

    std::chrono::microseconds step_delay_us = (frequency_hz > 0.0)
        ? frequency_to_delay(frequency_hz) : min_pulse_delay_us_;
    if (step_delay_us < min_pulse_delay_us_) step_delay_us = min_pulse_delay_us_;
    if (step_delay_us < MIN_ALLOWABLE_DELAY_US) step_delay_us = MIN_ALLOWABLE_DELAY_US;

    ::gpiod::line::value direction_value = (steps > 0) ? ::gpiod::line::value::ACTIVE : ::gpiod::line::value::INACTIVE;
    try {
        motor_request_opt_->set_value(dir_pin_, direction_value);
    } catch (const std::exception &e) { throw std::runtime_error("Set direction failed: " + std::string(e.what())); }

    enable();

    int num_steps = std::abs(steps);
    std::chrono::microseconds half_delay = step_delay_us / 2;
    if (half_delay.count() < 1) half_delay = std::chrono::microseconds(1);

    // Get reference to request object for loop
    auto& request = *motor_request_opt_;

    try {
        for (int i = 0; i < num_steps; ++i) {
            request.set_value(step_pin_, ::gpiod::line::value::ACTIVE);
            std::this_thread::sleep_for(half_delay);
            request.set_value(step_pin_, ::gpiod::line::value::INACTIVE);
            std::this_thread::sleep_for(half_delay);
        }
    } catch (const std::exception &e) {
        try { request.set_value(step_pin_, ::gpiod::line::value::INACTIVE); } catch (...) {}
        throw std::runtime_error("Step sequence failed: " + std::string(e.what()));
    }
}

} // namespace stepper_controller_cpp