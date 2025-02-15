from .hardware_interface import StepperMotorInterface

class MockStepperMotor(StepperMotorInterface):
    def __init__(self, logger):
        self.motors = {}
        self.logger = logger
        self.logger.info("MockStepperMotor initialized.")

    def initialize_motors(self, motor_configs):
        self.motors = motor_configs
        self.logger.info(f"Mock motors initialized with configs: {self.motors}")

    def set_speed(self, motor_id: int, speed: float):
        if motor_id in self.motors:
            if speed == 0:
                self.logger.info(f"Mock: Stopped motor {motor_id} by setting speed to 0")
            else:
                self.logger.info(f"Mock: Set motor {motor_id} speed to {speed}")
        else:
            self.logger.error(f"Mock error: Motor {motor_id} not initialized.")

    def step_motor(self, motor_id: int, steps: int, speed: float):
        if motor_id in self.motors:
            self.logger.info(f"Mock: Step motor {motor_id}: {steps} steps at speed {speed}")
        else:
            self.logger.error(f"Mock error: Motor {motor_id} not initialized.")

    def cleanup(self):
        self.logger.info("Mock cleanup called.")
