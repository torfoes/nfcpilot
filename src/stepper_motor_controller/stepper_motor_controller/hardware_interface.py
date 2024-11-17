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
    def start_motor(self, motor_id: int, speed: int):
        pass

    @abstractmethod
    def stop_motor(self, motor_id: int):
        pass

    @abstractmethod
    def set_speed(self, motor_id: int, speed: int):
        pass

    @abstractmethod
    def cleanup(self):
        pass
