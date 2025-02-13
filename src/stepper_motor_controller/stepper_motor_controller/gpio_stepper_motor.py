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
        chip_device = "gpiochip0"  # default for older Pis
        if os.path.exists("/dev/gpiochip-rpi"):
            chip_device = "gpiochip-rpi"
        else:
            try:
                with open("/proc/device-tree/model", "r") as model_file:
                    model = model_file.read()
                if "Raspberry Pi 5" in model:
                    chip_device = "gpiochip4"
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
        Each motor config should have 'step_pin', 'dir_pin' and optionally 'enable_pin'.
        If 'enable_pin' is set to a negative value (e.g. -1), it is considered unused.
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
                    default_vals=[0]
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
                    default_vals=[0]
                )
            except OSError as e:
                self.logger.error(f"Error requesting dir_line on pin {dir_pin} for motor {motor_id}: {e}")
                try:
                    step_line.release()
                except Exception:
                    pass
                continue

            # Request the ENABLE line, if provided and valid (>= 0).
            if enable_pin is not None and enable_pin >= 0:
                try:
                    enable_line = self.chip.get_line(enable_pin)
                    enable_line.request(
                        consumer="GPIOStepperMotor-enable",
                        type=gpiod.LINE_REQ_DIR_OUT,
                        default_vals=[0]
                    )
                except OSError as e:
                    self.logger.error(f"Error requesting enable_line on pin {enable_pin} for motor {motor_id}: {e}")
                    enable_line = None  # Or decide if you want to abort motor initialization
            else:
                enable_line = None

            motor = {
                'step_line': step_line,
                'dir_line': dir_line,
                'enable_line': enable_line,
                'speed': config.get('initial_speed', 0.0),  # speed is a float (rps)
                'running': False,
                'thread': None,
                'lock': threading.Lock(),
                'stop_event': threading.Event(),
            }
            self.motors[motor_id] = motor
            self.logger.info(f"Motor {motor_id} initialized with config: {config}")

    def start_motor(self, motor_id: int, speed: float):
        if motor_id in self.motors:
            motor = self.motors[motor_id]
            with motor['lock']:
                if motor['running']:
                    # If already running, update speed instead.
                    motor['speed'] = speed
                    self.logger.info(f"Motor {motor_id} is already running; updated speed to {speed}")
                    return
                motor['speed'] = speed
                motor['running'] = True
                motor['stop_event'].clear()
                motor['thread'] = threading.Thread(target=self._run_motor, args=(motor_id,))
                motor['thread'].daemon = True
                motor['thread'].start()
                self.logger.info(f"Started motor {motor_id} at speed {speed}")
        else:
            self.logger.error(f"Error: Motor {motor_id} not initialized.")

    def _run_motor(self, motor_id: int):
        motor = self.motors[motor_id]
        step_line = motor['step_line']
        dir_line = motor['dir_line']
        stop_event = motor['stop_event']

        try:
            while not stop_event.is_set():
                # Check the current speed (under lock) on every iteration.
                with motor['lock']:
                    current_speed = motor['speed']
                # Update direction based on the sign of the current speed.
                dir_line.set_value(1 if current_speed >= 0 else 0)
                current_speed = abs(current_speed)
                # Calculate delay based on current speed (assuming 200 steps per revolution).
                delay = 0.1 if current_speed == 0 else 1.0 / (current_speed * 200 * 2)
                # Generate the pulse (50% duty cycle).
                step_line.set_value(1)
                time.sleep(delay)
                step_line.set_value(0)
                time.sleep(delay)
        except Exception as e:
            self.logger.error(f"Error running motor {motor_id}: {e}")
        finally:
            motor['running'] = False

    def set_speed(self, motor_id: int, speed: float):
        """
        Unified interface to update the motor speed. If the motor is not running,
        it will be started with the new speed. If it is running, the speed is updated
        on the fly.
        """
        if motor_id in self.motors:
            motor = self.motors[motor_id]
            with motor['lock']:
                was_running = motor['running']
                motor['speed'] = speed
                motor['dir_line'].set_value(1 if speed >= 0 else 0)
            if not was_running:
                self.start_motor(motor_id, speed)
            else:
                self.logger.info(f"Updated motor {motor_id} speed to {speed}")
        else:
            self.logger.error(f"Error: Motor {motor_id} not initialized.")

    def stop_motor(self, motor_id: int):
        if motor_id in self.motors:
            motor = self.motors[motor_id]
            with motor['lock']:
                if not motor['running']:
                    self.logger.warning(f"Motor {motor_id} is not running.")
                    return
                motor['running'] = False
                motor['stop_event'].set()
            if motor['thread'] is not None:
                motor['thread'].join()
                motor['thread'] = None
            self.logger.info(f"Stopped motor {motor_id}")
        else:
            self.logger.error(f"Error: Motor {motor_id} not initialized.")

    def cleanup(self):
        # Stop all motors and release the GPIO lines and chip.
        for motor_id, motor in self.motors.items():
            self.stop_motor(motor_id)
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
