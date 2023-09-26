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

import commands2
from commands2 import Trigger
from commands2.button import CommandXboxController


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

    @property
    @abstractmethod
    def align(self) -> Trigger:
        """Align with the nearest scoring field element"""
        raise NotImplementedError

    @property
    @abstractmethod
    def toggle_field_relative(self) -> Trigger:
        """Toggle field-relative control on or off"""
        raise NotImplementedError

    @property
    @abstractmethod
    def ski_stop(self) -> Trigger:
        """Turn the wheels to an 'X' shape"""
        raise NotImplementedError


class OperatorActionSet(Protocol):
    @abstractmethod
    def pivot(self) -> float:
        """The power to the pivot motors, from -1 to 1"""
        raise NotImplementedError

    @abstractmethod
    def extend(self) -> float:
        """The power to the winch motor, from -1 to 1"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reach_high(self) -> Trigger:
        """Button to move the arm to the high goal"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reach_mid(self) -> Trigger:
        """Button to move the arm to the middle goal"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reach_low(self) -> Trigger:
        """Button to move the arm to the low goal"""
        raise NotImplementedError

    @property
    @abstractmethod
    def stow(self) -> Trigger:
        "Button to rotate the arm upward and extend it to its minimum length"
        raise NotImplementedError

    @property
    @abstractmethod
    def intake(self) -> Trigger:
        """Hold button to intake"""
        raise NotImplementedError

    @property
    @abstractmethod
    def outtake(self) -> Trigger:
        """Hold button to outtake"""
        raise NotImplementedError

    @property
    @abstractmethod
    def cycle_next_height(self) -> Trigger:
        """Choose the next arm height"""
        raise NotImplementedError

    @property
    @abstractmethod
    def cycle_previous_height(self) -> Trigger:
        """Choose the previous arm height"""
        raise NotImplementedError

    @property
    @abstractmethod
    def toggle_brake(self) -> Trigger:
        """Toggle the friction brake that holds the arm in place"""
        raise NotImplementedError

    @property
    @abstractmethod
    def reset_arm_angle(self) -> Trigger:
        raise NotImplementedError

    @property
    @abstractmethod
    def reset_winch_extension(self) -> Trigger:
        raise NotImplementedError


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
        return deadband(-self.stick.getLeftY(), 0.08)

    def strafe(self) -> float:
        """The robot's movement along the Y axis, controlled by moving the left joystick left and right. From -1 to 1"""
        return deadband(-self.stick.getLeftX(), 0.08)

    def turn(self) -> float:
        """The robot's movement around the Z axis, controlled by moving the right joystick left and right.
        From -1 to 1, CCW+
        """
        return deadband(-self.stick.getRightX(), 0.08) * 0.6

    @property
    def balance(self) -> Trigger:
        return Trigger(lambda: False)

    @property
    def reset_gyro(self) -> Trigger:
        return self.stick.start()

    @property
    def align(self) -> Trigger:
        return Trigger(lambda: False)

    @property
    def toggle_field_relative(self) -> Trigger:
        return self.stick.back()    # PS4 Button 9

    @property
    def ski_stop(self) -> Trigger:
        return self.stick.Y()


class XboxOperator(OperatorActionSet):
    """Operate the arm with an Xbox controller"""

    def __init__(self, port: int):
        """Construct an XboxOperator

        :param port: The port that the joystick is plugged into. Reported on the Driver Station
        """
        self.stick = CommandXboxController(port)
        self.loop = commands2.CommandScheduler.getInstance().getDefaultButtonLoop()

    def pivot(self) -> float:
        return deadband(-self.stick.getRightY(), 0.01)

    def extend(self) -> float:
        return deadband(-self.stick.getLeftY(), 0.01)

    @property
    def reach_high(self) -> Trigger:
        return Trigger()

    @property
    def reach_mid(self) -> Trigger:
        return Trigger()

    @property
    def reach_low(self) -> Trigger:
        return Trigger()

    @property
    def stow(self) -> Trigger:
        return self.stick.X()

    @property
    def intake(self) -> Trigger:
        return self.stick.leftBumper()

    @property
    def outtake(self) -> Trigger:
        return self.stick.leftTrigger(0.2)

    @property
    def cycle_next_height(self) -> Trigger:
        return Trigger(self.stick.POVUp(self.loop).getAsBoolean)

    @property
    def cycle_previous_height(self) -> Trigger:
        return Trigger(self.stick.POVDown(self.loop).getAsBoolean)

    @property
    def toggle_brake(self) -> Trigger:
        return self.stick.Y()

    @property
    def reset_arm_angle(self) -> Trigger:
        return self.stick.start()

    @property
    def reset_winch_extension(self) -> Trigger:
        return self.stick.back()


def deadband(value, band):
    return value if abs(value) > band else 0
