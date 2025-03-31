import os
import gpiod
import threading
import time
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
        Initialize motors based on the provided configurations.
        Each motor config should have 'step_pin', 'dir_pin', and optionally 'enable_pin'.
        If 'enable_pin' is set to a negative value, it is considered unused.
        """
        for motor_id, config in motor_configs.items():
            step_pin = config['step_pin']
            dir_pin = config['dir_pin']
            enable_pin = config.get('enable_pin', None)

            # Request the STEP line using the new API.
            try:
                step_req = gpiod.request_lines(
                    self.chip.path,
                    consumer="GPIOStepperMotor-step",
                    config={ step_pin: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT,
                                                           output_value=gpiod.line.Value.INACTIVE) }
                )
            except OSError as e:
                self.logger.error(f"Error requesting step line on pin {step_pin} for motor {motor_id}: {e}")
                continue

            # Request the DIR line.
            try:
                dir_req = gpiod.request_lines(
                    self.chip.path,
                    consumer="GPIOStepperMotor-dir",
                    config={ dir_pin: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT,
                                                          output_value=gpiod.line.Value.INACTIVE) }
                )
            except OSError as e:
                self.logger.error(f"Error requesting dir line on pin {dir_pin} for motor {motor_id}: {e}")
                try:
                    step_req.release()
                except Exception:
                    pass
                continue

            # Request the ENABLE line, if provided and valid.
            if enable_pin is not None and enable_pin >= 0:
                try:
                    enable_req = gpiod.request_lines(
                        self.chip.path,
                        consumer="GPIOStepperMotor-enable",
                        config={ enable_pin: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT,
                                                                 output_value=gpiod.line.Value.INACTIVE) }
                    )
                except OSError as e:
                    self.logger.error(f"Error requesting enable line on pin {enable_pin} for motor {motor_id}: {e}")
                    enable_req = None
            else:
                enable_req = None

            self.motors[motor_id] = {
                'step_req': step_req,
                'dir_req': dir_req,
                'enable_req': enable_req,
                'speed': config.get('initial_speed', 0.0),
                'running': False,
                'thread': None,
                'lock': threading.Lock(),
                'stop_event': threading.Event(),
                'step_pin': step_pin,
                'dir_pin': dir_pin,
                'enable_pin': enable_pin,
            }
            self.logger.info(f"Motor {motor_id} initialized with config: {config}")

    def _run_motor(self, motor_id: int):
        """Internal thread function for continuous motion via set_speed()."""
        motor = self.motors[motor_id]
        step_req = motor['step_req']
        dir_req = motor['dir_req']
        stop_event = motor['stop_event']
        step_pin = motor['step_pin']
        dir_pin = motor['dir_pin']

        try:
            while not stop_event.is_set():
                with motor['lock']:
                    current_speed = motor['speed']
                # Set direction based on sign.
                dir_req.set_value(dir_pin, 1 if current_speed >= 0 else 0)
                current_speed = abs(current_speed)
                delay = 0.1 if current_speed == 0 else 1.0 / (current_speed * 200 * 2)
                step_req.set_value(step_pin, 1)
                time.sleep(delay)
                step_req.set_value(step_pin, 0)
                time.sleep(delay)
        except Exception as e:
            self.logger.error(f"Error running motor {motor_id}: {e}")
        finally:
            motor['running'] = False

    def _step_motor_thread(self, motor_id: int, steps: int, speed: float):
        """Internal thread function to step the motor a fixed number of steps."""
        motor = self.motors[motor_id]
        step_req = motor['step_req']
        dir_req = motor['dir_req']
        step_pin = motor['step_pin']
        # Set direction based on the sign of steps.
        if steps < 0:
            steps = abs(steps)
            dir_req.set_value(motor['dir_pin'], 0)
        else:
            dir_req.set_value(motor['dir_pin'], 1)
        current_speed = abs(speed)
        delay = 0.1 if current_speed == 0 else 1.0 / (current_speed * 200 * 2)
        for _ in range(steps):
            if motor['stop_event'].is_set():
                break
            step_req.set_value(step_pin, 1)
            time.sleep(delay)
            step_req.set_value(step_pin, 0)
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
                motor['dir_req'].set_value(motor['dir_pin'], 1 if speed >= 0 else 0)
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
            self.set_speed(motor_id, 0)
            try:
                motor['step_req'].release()
            except Exception as e:
                self.logger.warning(f"Error releasing step_req for motor {motor_id}: {e}")
            try:
                motor['dir_req'].release()
            except Exception as e:
                self.logger.warning(f"Error releasing dir_req for motor {motor_id}: {e}")
            if motor['enable_req'] is not None:
                try:
                    motor['enable_req'].release()
                except Exception as e:
                    self.logger.warning(f"Error releasing enable_req for motor {motor_id}: {e}")
        try:
            self.chip.close()
        except Exception as e:
            self.logger.warning(f"Error closing GPIO chip: {e}")
        self.logger.info("GPIO cleanup completed.")
