"""The OI (Operator Input) module contains `Action Sets` and `Control Schemes`.

An `Action Set` defines the inputs a controller should have. For example, a driver's controller needs to have an input
for driving forwards and backwards. These inputs can be either functions that return a value such as a float or bool,
or they can be Commands buttons. A button can be bound to run a command when pressed, held, etc.

A `Control Scheme` implements an `Action Set` and defines which physical buttons or joysticks on a controller perform
each action. For example, on an Xbox controller, the left joystick might be bound to forwards and backwards movement.
Any special logic (e.g. inverting the Y axis on a joystick) is also defined in a `Control Scheme`.
"""

from typing import Protocol
from abc import abstractmethod

import wpilib
from commands2 import Trigger
from commands2.button import CommandXboxController, JoystickButton


# Action Sets


class DriverActionSet(Protocol):
    @abstractmethod
    def forward(self) -> float:
        """Movement along the X axis, from -1 to 1"""
        raise NotImplementedError

    @abstractmethod
    def strafe(self) -> float:
        """Movement along the Y axis, from -1 to 1"""
        raise NotImplementedError

    @abstractmethod
    def turn(self) -> float:
        """Rotation around the Z axis, from -1 (clockwise) to 1 (counter-clockwise)"""
        raise NotImplementedError

    @property
    @abstractmethod
    def balance(self) -> Trigger:
        """Button that activates the robot's auto-balance"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reset_gyro(self) -> Trigger:
        raise NotImplementedError


class OperatorActionSet(Protocol):
    pass


# Control schemes


class XboxDriver(DriverActionSet):
    """Drive the robot with an Xbox controller"""

    def __init__(self, port: int):
        """Construct an XboxDriver

        :param port: The port that the joystick is plugged into. Reported on the Driver Station
        """
        self.stick = CommandXboxController(port)

    def forward(self) -> float:
        """The robot's movement along the X axis, controlled by moving the left joystick up and down. From -1 to 1"""
        return -self.stick.getLeftY() * 1

    def strafe(self) -> float:
        """The robot's movement along the Y axis, controlled by moving the left joystick left and right. From -1 to 1"""
        return -self.stick.getLeftX() * 1

    def turn(self) -> float:
        """The robot's movement around the Z axis, controlled by moving the right joystick left and right.
        From -1 to 1, CCW+
        """
        return -self.stick.getRightX() * .5

    @property
    def balance(self) -> Trigger:
        return self.stick.X()

    @property
    def reset_gyro(self) -> Trigger:
        return self.stick.Y()


class XboxOperator(OperatorActionSet):
    pass
