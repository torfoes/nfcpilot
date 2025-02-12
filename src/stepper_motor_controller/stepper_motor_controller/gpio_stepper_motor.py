import gpiod
import threading
import time
from .hardware_interface import StepperMotorInterface


class GPIOStepperMotor(StepperMotorInterface):
    def __init__(self, logger):
        self.motors = {}
        self.logger = logger
        self.logger.info("GPIOStepperMotor (libgpiod) initialized.")

        # Open the GPIO chip (typically "gpiochip0" on the Pi)
        self.chip = gpiod.Chip("gpiochip4")

    def initialize_motors(self, motor_configs):
        """
        Initialize motors based on the provided configurations.
        Each motor config should have 'step_pin', 'dir_pin' and optionally 'enable_pin'.
        """
        for motor_id, config in motor_configs.items():
            step_pin = config['step_pin']
            dir_pin = config['dir_pin']
            enable_pin = config.get('enable_pin', None)

            # Request the output lines for each pin using libgpiod
            step_line = self.chip.get_line(step_pin)
            step_line.request(
                consumer="GPIOStepperMotor-step",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0]
            )

            dir_line = self.chip.get_line(dir_pin)
            dir_line.request(
                consumer="GPIOStepperMotor-dir",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0]
            )

            if enable_pin is not None:
                enable_line = self.chip.get_line(enable_pin)
                enable_line.request(
                    consumer="GPIOStepperMotor-enable",
                    type=gpiod.LINE_REQ_DIR_OUT,
                    default_vals=[0]
                )
            else:
                enable_line = None

            motor = {
                'step_line': step_line,
                'dir_line': dir_line,
                'enable_line': enable_line,
                'speed': config.get('initial_speed', 0),
                'running': False,
                'thread': None,
                'lock': threading.Lock(),
                'stop_event': threading.Event(),
            }
            self.motors[motor_id] = motor
            self.logger.info(f"Motor {motor_id} initialized with config: {config}")

    def start_motor(self, motor_id: int, speed: int):
        if motor_id in self.motors:
            motor = self.motors[motor_id]
            with motor['lock']:
                if motor['running']:
                    self.logger.warning(f"Motor {motor_id} is already running.")
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
        speed = motor['speed']
        stop_event = motor['stop_event']

        # Set the direction based on the sign of speed.
        # A value of 1 corresponds to a HIGH output.
        dir_line.set_value(1 if speed >= 0 else 0)
        speed = abs(speed)

        # Calculate delay based on speed (using 200 steps per revolution as an example).
        delay = 0.1 if speed == 0 else 1.0 / (speed * 200 * 2)

        try:
            while not stop_event.is_set():
                # Pulse the STEP line HIGH then LOW.
                step_line.set_value(1)
                time.sleep(delay)
                step_line.set_value(0)
                time.sleep(delay)
        except Exception as e:
            self.logger.error(f"Error running motor {motor_id}: {e}")
        finally:
            motor['running'] = False

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

    def set_speed(self, motor_id: int, speed: int):
        if motor_id in self.motors:
            motor = self.motors[motor_id]
            with motor['lock']:
                was_running = motor['running']
                if was_running:
                    # Stop the motor if it's currently running so that the new speed takes effect.
                    self.stop_motor(motor_id)
                motor['speed'] = speed
                motor['dir_line'].set_value(1 if speed >= 0 else 0)
            # If the motor was running, restart it with the updated speed.
            if was_running:
                self.start_motor(motor_id, speed)
            self.logger.info(f"Set motor {motor_id} speed to {speed}")
        else:
            self.logger.error(f"Error: Motor {motor_id} not initialized.")

    def cleanup(self):
        # Stop all motors and release the GPIO chip.
        for motor_id in list(self.motors.keys()):
            self.stop_motor(motor_id)
        self.chip.close()
        self.logger.info("GPIO cleanup completed.")
