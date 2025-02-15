from abc import ABC, abstractmethod


class StepperMotorInterface(ABC):
    @abstractmethod
    def initialize_motors(self, motor_configs):
        """
        Initialize motors based on the provided configurations.

        :param motor_configs: A dictionary with motor_id as key and configuration as value.
        """
        pass

    @abstractmethod
    def set_speed(self, motor_id: int, speed: float):
        """
        Set the motor speed continuously.
        Speed is a float between -1 and 1, where the magnitude is the fraction of maximum speed.
        Setting speed to 0 stops the motor.
        """
        pass

    @abstractmethod
    def step_motor(self, motor_id: int, steps: int, speed: float):
        """
        Step the motor a fixed number of steps at the given speed.
        Negative steps indicate reverse direction.
        """
        pass

    @abstractmethod
    def cleanup(self):
        """
        Cleanup any resources (e.g. GPIO lines).
        """
        pass
