"""Put a technical description of the telescoping arm and how it's controlled through code here."""
import math
from collections.abc import Callable

import commands2
import rev
import wpilib
from wpimath.geometry import Rotation2d, Translation3d

from map import PivotConstants, WinchConstants, RobotDimensions

"""
Possible ideas for counteracting gravity:
    * Have min, mid, and max extension values that we switch between
    * 
"""

# TODO: Figure out structure for arm code. I.e., how should I split up the pivot and winch?
# TODO: Make the arm stow itself


class ArmPivot(commands2.SubsystemBase):
    """The pivot component of the arm"""

    def __init__(self):
        commands2.SubsystemBase.__init__(self)

        self._setpoint = None

        self.leader = rev.CANSparkMax(PivotConstants.LEADER_MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)
        self.controller = self.leader.getPIDController()
        self.encoder = self.leader.getEncoder()
        self.follower = rev.CANSparkMax(PivotConstants.FOLLOWER_MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)

        self._config_motors()

    def _config_motors(self):
        self.leader.restoreFactoryDefaults()

        self.controller.setP(0)
        self.controller.setI(0)
        self.controller.setD(0)
        self.controller.setFF(0)

        self.leader.setSmartCurrentLimit(20)
        self.leader.setSmartCurrentLimit(40)

        self.leader.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

        self.encoder.setPositionConversionFactor(1)
        self.encoder.setPosition(0)

        self.follower.follow(self.leader)

    def rotate_to(self, angle: Rotation2d):
        self._setpoint = angle.degrees()
        self.controller.setReference(self._setpoint, rev.CANSparkMax.ControlType.kPosition)

    def set_power(self, power: float):
        self.leader.set(power)

    def reset_angle(self, reference: float = 0):
        self.encoder.setPosition(reference)

    def at_setpoint(self) -> bool:
        if self._setpoint is None:
            raise Exception("Setpoint isn't set! Call rotate_to() first.")

        return at_setpoint(self.encoder.getPosition(), self._setpoint, PivotConstants.PID_TOLERANCE)

    @property
    def angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.encoder.getPosition())


class ArmWinch(commands2.SubsystemBase):
    """The winch component of the arm"""

    def __init__(self):
        commands2.SubsystemBase.__init__(self)

        self._setpoint = None

        self.motor = rev.CANSparkMax(WinchConstants.MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)
        self.controller = self.motor.getPIDController()
        self.encoder = self.motor.getEncoder()

        self._config_motors()

    def _config_motors(self):
        self.motor.restoreFactoryDefaults()

        self.controller.setP(0)
        self.controller.setI(0)
        self.controller.setD(0)
        self.controller.setFF(0)

        self.motor.setSmartCurrentLimit(20)

        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

        self.encoder.setPositionConversionFactor(1)
        self.encoder.setPosition(0)

    def extend_distance(self, distance: float):
        self._setpoint = distance
        self.controller.setReference(self._setpoint, rev.CANSparkMax.ControlType.kPosition)

    def set_power(self, power: float):
        self.motor.set(power)

    def at_setpoint(self) -> bool:
        if self._setpoint is None:
            raise Exception("Setpoint isn't set! Call rotate_to() first.")

        return at_setpoint(self.encoder.getPosition(), self._setpoint, WinchConstants.PID_TOLERANCE)


class ArmStructure(commands2.SubsystemBase):
    """
    A structure that combines the winch and joint components. Instantiate this instead of the components seperately.
    """

    def __init__(self):
        commands2.SubsystemBase.__init__(self)

        self.pivot = ArmPivot()
        self.winch = ArmWinch()

    # TODO: Make the arm not kill itself by ramming into the floor if we command the angle to change but not the extension.
    # We could constantly run the maximum_extension function and adjust in periodic()

    def extend_distance(self, distance: float):
        # Limit the arm's extension to avoid fouls
        limited_extension = min(maximum_extension(self.pivot.angle), distance)

        self.winch.extend_distance(limited_extension)

    def stow(self):
        """Command the arm to rotate upright and extend as far in as possible."""

        self.extend_distance(0)

        # 90 degrees is upright
        self.pivot.rotate_to(Rotation2d.fromDegrees(90))

    def manual_winch_command(self, power: Callable[[], float]):
        return commands2.RunCommand(lambda: self.winch.set_power(power()), self.winch).alongWith(
            commands2.RunCommand(lambda: wpilib.SmartDashboard.putNumber("Winch Power", power()))  # type: ignore
        )

    def manual_pivot_command(self, power: Callable[[], float]):
        return commands2.RunCommand(lambda: self.pivot.set_power(power()), self.pivot).alongWith(
            commands2.RunCommand(lambda: wpilib.SmartDashboard.putNumber("Pivot Power", power()))  # type: ignore
        )


def angle_to(target: Translation3d, robot_translation: Translation3d) -> Rotation2d:
    global_arm_translation = robot_translation + PivotConstants.RELATIVE_POSITION
    xy_distance = global_arm_translation.toTranslation2d().distance(target.toTranslation2d())
    z_distance = abs(target.z - global_arm_translation.z)
    angle = math.atan(z_distance / xy_distance)
    return Rotation2d(angle)


def extension_to(target: Translation3d, angle: Rotation2d, robot_translation: Translation3d) -> float:
    global_arm_translation = robot_translation + PivotConstants.RELATIVE_POSITION
    z_distance = abs(target.z - global_arm_translation.z)
    hyp = z_distance / angle.sin()
    return hyp


def maximum_extension(angle: Rotation2d) -> float:
    horizontal = abs(RobotDimensions.PIVOT_TO_MAX_HORIZONTAL_EXTENSION / angle.cos())
    to_ceiling = abs(RobotDimensions.PIVOT_TO_MAX_VERTICAL_EXTENSION / (angle - Rotation2d.fromDegrees(90)).cos())
    to_floor = abs(RobotDimensions.PIVOT_TO_FLOOR / (Rotation2d.fromDegrees(360) - angle).sin())
    return min(horizontal, to_ceiling, to_floor, RobotDimensions.MAX_EXTENSION_FROM_PIVOT)


def at_setpoint(reference: float, setpoint: float, tolerance: float) -> bool:
    return setpoint - tolerance <= reference <= setpoint + tolerance
