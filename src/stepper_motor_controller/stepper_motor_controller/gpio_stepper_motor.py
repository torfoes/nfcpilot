import gpiod
import threading
import time
import os
from .hardware_interface import StepperMotorInterface


class GPIOStepperMotor(StepperMotorInterface):
    def __init__(self, logger):
        self.motors = {}
        self.logger = logger
        self.logger.info("GPIOStepperMotor (libgpiod) initialized.")

        # Determine which GPIO chip to use:
        chip_device = "/dev/gpiochip0"  # default for older Pis
        if os.path.exists("/dev/gpiochip-rpi"):
            chip_device = "/dev/gpiochip-rpi"
        else:
            try:
                with open("/proc/device-tree/model", "r") as model_file:
                    model = model_file.read()
                if "Raspberry Pi 5" in model:
                    chip_device = "/dev/gpiochip4"
            except Exception as e:
                self.logger.warning(f"Unable to determine Pi model: {e}")

        self.logger.info(f"Using GPIO chip: {chip_device}")
        try:
            self.chip = gpiod.Chip(chip_device)
        except Exception as e:
            self.logger.error(f"Failed to open GPIO chip {chip_device}: {e}")
            raise

    def initialize_motors(self, motor_configs):
        """
        motor_configs: dict mapping motor IDs to configuration dictionaries.
        Each configuration should include:
          - step_pin: int, GPIO offset for step signal
          - dir_pin: int, GPIO offset for direction signal
          - enable_pin: int (or -1 if unused)
          - initial_speed: float
        """
        for motor_id, config in motor_configs.items():
            try:
                step_pin = config['step_pin']
                dir_pin = config['dir_pin']
                enable_pin = config.get('enable_pin', -1)

                # Get the STEP line (using get_lines() returns a list)
                step_line = self.chip.get_lines([step_pin])[0]
                step_line.request(
                    consumer="GPIOStepperMotor-step",
                    type=gpiod.LINE_REQ_DIR_OUT,
                    default_vals=[0]
                )

                # Get the DIR line
                dir_line = self.chip.get_lines([dir_pin])[0]
                dir_line.request(
                    consumer="GPIOStepperMotor-dir",
                    type=gpiod.LINE_REQ_DIR_OUT,
                    default_vals=[0]
                )

                # Get the ENABLE line if provided
                if enable_pin is not None and enable_pin >= 0:
                    enable_line = self.chip.get_lines([enable_pin])[0]
                    enable_line.request(
                        consumer="GPIOStepperMotor-enable",
                        type=gpiod.LINE_REQ_DIR_OUT,
                        default_vals=[0]
                    )
                else:
                    enable_line = None

                self.motors[motor_id] = {
                    'step_line': step_line,
                    'dir_line': dir_line,
                    'enable_line': enable_line,
                    'speed': config.get('initial_speed', 0.0),
                    # Additional fields can be added for state management.
                }
                self.logger.info(f"Motor {motor_id} initialized (step: {step_pin}, dir: {dir_pin}).")
            except Exception as e:
                self.logger.error(f"Error initializing motor {motor_id}: {e}")
                raise

    def set_speed(self, motor_id, speed):
        # Dummy implementation for continuous speed control.
        if motor_id not in self.motors:
            self.logger.error(f"Motor {motor_id} not found.")
            return
        self.motors[motor_id]['speed'] = speed
        self.logger.info(f"Set motor {motor_id} speed to {speed}")
        # Here you would start a thread that continuously toggles the step_line
        # according to the speed. (Left as an exercise.)

    def step_motor(self, motor_id, steps, speed):
        # Dummy implementation for stepping a fixed number of steps.
        if motor_id not in self.motors:
            self.logger.error(f"Motor {motor_id} not found.")
            return

        motor = self.motors[motor_id]
        step_line = motor['step_line']
        dir_line = motor['dir_line']

        # Set direction based on the sign of steps.
        if steps < 0:
            steps = abs(steps)
            dir_line.set_value(0)
        else:
            dir_line.set_value(1)

        # Calculate delay based on speed.
        delay = 0.1 if speed == 0 else 1.0 / (abs(speed) * 200 * 2)

        for i in range(steps):
            step_line.set_value(1)
            time.sleep(delay)
            step_line.set_value(0)
            time.sleep(delay)
        self.logger.info(f"Stepped motor {motor_id} for {steps} steps at speed {speed}")

    def cleanup(self):
        for motor_id, motor in self.motors.items():
            try:
                motor['step_line'].release()
            except Exception as e:
                self.logger.warning(f"Error releasing step_line for motor {motor_id}: {e}")
            try:
                motor['dir_line'].release()
            except Exception as e:
                self.logger.warning(f"Error releasing dir_line for motor {motor_id}: {e}")
            if motor['enable_line'] is not None:
                try:
                    motor['enable_line'].release()
                except Exception as e:
                    self.logger.warning(f"Error releasing enable_line for motor {motor_id}: {e}")
        try:
            self.chip.close()
        except Exception as e:
            self.logger.warning(f"Error closing GPIO chip: {e}")
        self.logger.info("GPIO cleanup completed.")