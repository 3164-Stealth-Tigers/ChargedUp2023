"""Put a technical description of the telescoping arm and how it's controlled through code here."""
import math
from collections.abc import Callable

import commands2
import rev
from wpimath.geometry import Rotation2d, Translation3d

from map import ArmJointConstants, WinchConstants, RobotDimensions

"""
Possible ideas for counteracting gravity:
    * Have min, mid, and max extension values that we switch between
    * 
"""


class ArmPivot:
    """The pivot component of the arm"""

    def __init__(self):
        self.leader = rev.CANSparkMax(ArmJointConstants.LEADER_MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)
        self.follower = rev.CANSparkMax(ArmJointConstants.FOLLOWER_MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)
        self.controller = self.leader.getPIDController()
        self.encoder = self.leader.getEncoder()

    def _config_motors(self):
        self.follower.follow(self.leader)

        # Configure smart motion
        pass

    def rotate_to(self, angle: Rotation2d):
        self.controller.setReference(angle.degrees(), rev.CANSparkMax.ControlType.kSmartMotion)

    def set_power(self, power: float):
        self.leader.set(power)

    def reset_angle(self, reference: float = 0):
        self.encoder.setPosition(reference)

    @property
    def angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.encoder.getPosition())


class ArmWinch:
    """The winch component of the arm"""

    def __init__(self):
        self.motor = rev.CANSparkMax(WinchConstants.MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)
        self.controller = self.motor.getPIDController()

        self._config_motors()

    def _config_motors(self):
        pass

    def extend(self, distance: float):
        self.controller.setReference(distance, rev.CANSparkMax.ControlType.kPosition)

    def set_power(self, power: float):
        self.motor.set(power)


class ArmStructure(commands2.SubsystemBase):
    """
    A structure that combines the winch and joint components. Instantiate this instead of the components seperately.
    """

    def __init__(self):
        commands2.SubsystemBase.__init__(self)

        self.pivot = ArmPivot()
        self.winch = ArmWinch()

    def extend(self, distance: float):
        # Limit the arm's extension to avoid fouls
        limited_extension = min(maximum_extension(self.pivot.angle), distance)

        self.winch.extend(limited_extension)

    def rotate_to(self, angle: Rotation2d):
        self.pivot.rotate_to(angle)

    def manual_command(self, pivot: Callable[[], float], extend: Callable[[], float]):
        def run():
            self.pivot.set_power(pivot())
            self.winch.set_power(extend())

        return commands2.RunCommand(run, [self])


def angle_to(target: Translation3d, robot_translation: Translation3d) -> Rotation2d:
    global_arm_translation = robot_translation + ArmJointConstants.RELATIVE_POSITION
    xy_distance = global_arm_translation.toTranslation2d().distance(target.toTranslation2d())
    z_distance = abs(target.z - global_arm_translation.z)
    angle = math.atan(z_distance / xy_distance)
    return Rotation2d(angle)


def extension_to(target: Translation3d, angle: Rotation2d, robot_translation: Translation3d) -> float:
    global_arm_translation = robot_translation + ArmJointConstants.RELATIVE_POSITION
    z_distance = abs(target.z - global_arm_translation.z)
    hyp = z_distance / angle.sin()
    return hyp


def maximum_extension(angle: Rotation2d) -> float:
    horizontal = abs(RobotDimensions.PIVOT_TO_MAX_HORIZONTAL_EXTENSION / angle.cos())
    to_ceiling = abs(RobotDimensions.PIVOT_TO_MAX_VERTICAL_EXTENSION / (angle - Rotation2d.fromDegrees(90)).cos())
    to_floor = abs(RobotDimensions.PIVOT_TO_FLOOR / (Rotation2d.fromDegrees(360) - angle).sin())
    return min(horizontal, to_ceiling, to_floor, RobotDimensions.MAX_EXTENSION_FROM_PIVOT)
