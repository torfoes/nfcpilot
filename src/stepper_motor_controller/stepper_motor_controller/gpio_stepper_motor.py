import RPi.GPIO as GPIO
import threading
import time
from .hardware_interface import StepperMotorInterface


class GPIOStepperMotor(StepperMotorInterface):
    def __init__(self, logger):
        self.motors = {}
        self.logger = logger
        self.logger.info("GPIOStepperMotor initialized.")

        # Initialize GPIO settings
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

    def initialize_motors(self, motor_configs):
        """
        Initialize motors based on the provided configurations.

        :param motor_configs: A dictionary with motor_id as key and configuration as value.
        """
        for motor_id, config in motor_configs.items():
            step_pin = config['step_pin']
            dir_pin = config['dir_pin']
            enable_pin = config.get('enable_pin', None)

            # Set up GPIO pins
            GPIO.setup(step_pin, GPIO.OUT)
            GPIO.setup(dir_pin, GPIO.OUT)
            if enable_pin is not None:
                GPIO.setup(enable_pin, GPIO.OUT)
                GPIO.output(enable_pin, GPIO.LOW)

            # Initialize motor state
            motor = {
                'step_pin': step_pin,
                'dir_pin': dir_pin,
                'enable_pin': enable_pin,
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
                    self.logger.warn(f"Motor {motor_id} is already running.")
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
        step_pin = motor['step_pin']
        dir_pin = motor['dir_pin']
        speed = motor['speed']
        stop_event = motor['stop_event']

        # Set the direction
        GPIO.output(dir_pin, GPIO.HIGH if speed >= 0 else GPIO.LOW)
        speed = abs(speed)

        if speed == 0:
            delay = 0.1  # Default delay for zero speed to prevent division by zero
        else:
            delay = 1.0 / (speed * 200 * 2)  # Adjust delay based on speed and steps per revolution

        try:
            while not stop_event.is_set():
                GPIO.output(step_pin, GPIO.HIGH)
                time.sleep(delay)
                GPIO.output(step_pin, GPIO.LOW)
                time.sleep(delay)
        except Exception as e:
            self.logger.error(f"Error running motor {motor_id}: {e}")
            motor['running'] = False

    def stop_motor(self, motor_id: int):
        if motor_id in self.motors:
            motor = self.motors[motor_id]
            with motor['lock']:
                if not motor['running']:
                    self.logger.warn(f"Motor {motor_id} is not running.")
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
                motor['speed'] = speed
                GPIO.output(motor['dir_pin'], GPIO.HIGH if speed >= 0 else GPIO.LOW)
                speed = abs(speed)
                if speed == 0:
                    delay = 0.1
                else:
                    delay = 1.0 / (speed * 200 * 2)
                motor['delay'] = delay
                motor['stop_event'].set()
                motor['running'] = False

            # Restart the motor with new speed if it was running
            if motor['running']:
                self.start_motor(motor_id, speed)
            self.logger.info(f"Set motor {motor_id} speed to {speed}")
        else:
            self.logger.error(f"Error: Motor {motor_id} not initialized.")

    def cleanup(self):
        for motor_id in list(self.motors.keys()):
            self.stop_motor(motor_id)
        GPIO.cleanup()
        self.logger.info("GPIO cleanup completed.")
