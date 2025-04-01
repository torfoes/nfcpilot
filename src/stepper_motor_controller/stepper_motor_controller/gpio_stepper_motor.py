import os
import gpiod
import threading
import time
from gpiod.line import Direction, Value, LineSettings
from .hardware_interface import StepperMotorInterface

class GPIOStepperMotor(StepperMotorInterface):
    def __init__(self, logger):
        self.motors = {}
        self.logger = logger
        self.logger.info("GPIOStepperMotor (libgpiod) initialized.")

        # Determine which GPIO chip to use:
        chip_device = "/dev/gpiochip0"  # default for older Pis

        # If the RPi-specific chip exists, use its full path:
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
        Initialize motors based on the provided configurations.
        Each motor config should have 'step_pin', 'dir_pin', and optionally 'enable_pin'.
        If 'enable_pin' is set to a negative value, it is considered unused.
        """
        for motor_id, config in motor_configs.items():
            step_pin = config['step_pin']
            dir_pin = config['dir_pin']
            enable_pin = config.get('enable_pin', None)

            # Request the STEP line.
            try:
                step_line = self.chip.get_line(step_pin)
                step_line.request(
                    consumer="GPIOStepperMotor-step",
                    type=gpiod.LINE_REQ_DIR_OUT,
                    default_vals=[Value.INACTIVE]
                )
            except OSError as e:
                self.logger.error(f"Error requesting step_line on pin {step_pin} for motor {motor_id}: {e}")
                continue

            # Request the DIR line.
            try:
                dir_line = self.chip.get_line(dir_pin)
                dir_line.request(
                    consumer="GPIOStepperMotor-dir",
                    type=gpiod.LINE_REQ_DIR_OUT,
                    default_vals=[Value.INACTIVE]
                )
            except OSError as e:
                self.logger.error(f"Error requesting dir_line on pin {dir_pin} for motor {motor_id}: {e}")
                try:
                    step_line.release()
                except Exception:
                    pass
                continue

            # Request the ENABLE line, if provided and valid.
            if enable_pin is not None and enable_pin >= 0:
                try:
                    enable_line = self.chip.get_line(enable_pin)
                    enable_line.request(
                        consumer="GPIOStepperMotor-enable",
                        type=gpiod.LINE_REQ_DIR_OUT,
                        default_vals=[Value.INACTIVE]
                    )
                except OSError as e:
                    self.logger.error(f"Error requesting enable_line on pin {enable_pin} for motor {motor_id}: {e}")
                    enable_line = None
            else:
                enable_line = None

            motor = {
                'step_line': step_line,
                'dir_line': dir_line,
                'enable_line': enable_line,
                'speed': config.get('initial_speed', 0.0),  # speed is a float between -1 and 1
                'running': False,
                'thread': None,
                'lock': threading.Lock(),
                'stop_event': threading.Event(),
                'step_pin': step_pin,
                'dir_pin': dir_pin,
                'enable_pin': enable_pin,
            }
            self.motors[motor_id] = motor
            self.logger.info(f"Motor {motor_id} initialized with config: {config}")

    def _run_motor(self, motor_id: int):
        """Internal thread function for continuous motion via set_speed()."""
        motor = self.motors[motor_id]
        step_line = motor['step_line']
        dir_line = motor['dir_line']
        stop_event = motor['stop_event']
        step_pin = motor['step_pin']
        dir_pin = motor['dir_pin']

        try:
            while not stop_event.is_set():
                with motor['lock']:
                    current_speed = motor['speed']
                # Set direction based on sign.
                dir_line.set_value(dir_pin, Value.ACTIVE if current_speed >= 0 else Value.INACTIVE)
                current_speed = abs(current_speed)
                delay = 0.1 if current_speed == 0 else 1.0 / (current_speed * 200 * 2)
                step_line.set_value(step_pin, Value.ACTIVE)
                time.sleep(delay)
                step_line.set_value(step_pin, Value.INACTIVE)
                time.sleep(delay)
        except Exception as e:
            self.logger.error(f"Error running motor {motor_id}: {e}")
        finally:
            motor['running'] = False

    def _step_motor_thread(self, motor_id: int, steps: int, speed: float):
        """Internal thread function to step the motor a fixed number of steps."""
        motor = self.motors[motor_id]
        step_line = motor['step_line']
        dir_line = motor['dir_line']
        step_pin = motor['step_pin']
        # Set direction based on the sign of steps.
        if steps < 0:
            steps = abs(steps)
            dir_line.set_value(motor['dir_pin'], Value.INACTIVE)
        else:
            dir_line.set_value(motor['dir_pin'], Value.ACTIVE)
        current_speed = abs(speed)
        delay = 0.1 if current_speed == 0 else 1.0 / (current_speed * 200 * 2)
        for _ in range(steps):
            if motor['stop_event'].is_set():
                break
            step_line.set_value(step_pin, Value.ACTIVE)
            time.sleep(delay)
            step_line.set_value(step_pin, Value.INACTIVE)
            time.sleep(delay)
        motor['running'] = False

    def set_speed(self, motor_id: int, speed: float):
        """
        Set the motor speed continuously.
        Speed is a float between -1 and 1 (fraction of maximum speed).
        Setting speed to 0 stops the motor.
        """
        if motor_id not in self.motors:
            self.logger.error(f"Error: Motor {motor_id} not initialized.")
            return

        motor = self.motors[motor_id]
        with motor['lock']:
            if speed == 0:
                if motor['running']:
                    motor['stop_event'].set()
                    if motor['thread'] is not None:
                        motor['thread'].join()
                        motor['thread'] = None
                    motor['running'] = False
                    self.logger.info(f"Stopped motor {motor_id} by setting speed to 0")
                else:
                    self.logger.info(f"Motor {motor_id} is already stopped.")
                motor['speed'] = 0
            else:
                motor['speed'] = speed
                motor['dir_line'].set_value(motor['dir_pin'], Value.ACTIVE if speed >= 0 else Value.INACTIVE)
                if not motor['running']:
                    motor['stop_event'].clear()
                    motor['running'] = True
                    motor['thread'] = threading.Thread(target=self._run_motor, args=(motor_id,))
                    motor['thread'].daemon = True
                    motor['thread'].start()
                    self.logger.info(f"Started motor {motor_id} at speed {speed}")
                else:
                    self.logger.info(f"Updated motor {motor_id} speed to {speed}")

    def step_motor(self, motor_id: int, steps: int, speed: float):
        """
        Step the motor a fixed number of steps at the given speed.
        Negative steps indicate reverse direction.
        """
        if motor_id not in self.motors:
            self.logger.error(f"Error: Motor {motor_id} not initialized.")
            return

        motor = self.motors[motor_id]
        with motor['lock']:
            if motor['running']:
                self.logger.warning(f"Motor {motor_id} is currently running. Cannot execute step command.")
                return
            motor['stop_event'].clear()
            motor['running'] = True
            motor['thread'] = threading.Thread(target=self._step_motor_thread, args=(motor_id, steps, speed))
            motor['thread'].daemon = True
            motor['thread'].start()
            self.logger.info(f"Stepping motor {motor_id} for {steps} steps at speed {speed}")

    def cleanup(self):
        """
        Cleanup all motors by stopping any running motion and releasing GPIO resources.
        """
        for motor_id, motor in self.motors.items():
            # Stop the motor by setting its speed to 0.
            self.set_speed(motor_id, 0)
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
