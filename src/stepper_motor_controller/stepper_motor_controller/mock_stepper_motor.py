from hardware_interface import StepperMotorInterface


class MockStepperMotor(StepperMotorInterface):
    def __init__(self, logger):
        self.motors = {}
        self.logger = logger
        self.logger.info("MockStepperMotor initialized.")

    def initialize_motors(self, motor_configs):
        self.motors = motor_configs
        self.logger.info(f"Mock motors initialized with configs: {self.motors}")

    def start_motor(self, motor_id: int, speed: int):
        if motor_id in self.motors:
            self.logger.info(f"Mock start motor {motor_id} at speed {speed}")
        else:
            self.logger.error(f"Mock error: Motor {motor_id} not initialized.")

    def stop_motor(self, motor_id: int):
        if motor_id in self.motors:
            self.logger.info(f"Mock stop motor {motor_id}")
        else:
            self.logger.error(f"Mock error: Motor {motor_id} not initialized.")

    def set_speed(self, motor_id: int, speed: int):
        if motor_id in self.motors:
            self.logger.info(f"Mock set motor {motor_id} speed to {speed}")
        else:
            self.logger.error(f"Mock error: Motor {motor_id} not initialized.")

    def cleanup(self):
        self.logger.info("Mock cleanup called.")
