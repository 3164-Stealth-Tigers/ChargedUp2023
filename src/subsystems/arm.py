"""Put a technical description of the telescoping arm and how it's controlled through code here."""
import enum
import math
from collections.abc import Callable

import commands2
import rev
import wpilib
import wpimath.controller
import wpimath.trajectory
from wpimath.geometry import Rotation2d, Translation3d

from map import PivotConstants, WinchConstants, RobotDimensions

# TODO: Figure out structure for arm code. I.e., how should I split up the pivot and winch?
# TODO: Make the arm stow itself

DELTA_TIME = 0.02


class MechanismMode(enum.Enum):
    DIRECT = enum.auto()
    POSITION = enum.auto()


class ArmPivot(commands2.SubsystemBase):
    """The pivot component of the arm"""

    def __init__(self):
        commands2.SubsystemBase.__init__(self)

        self.mode = MechanismMode.DIRECT
        self.setpoint = wpimath.trajectory.TrapezoidProfile.State()
        self.goal = wpimath.trajectory.TrapezoidProfile.State()

        self.constraints = wpimath.trajectory.TrapezoidProfile.Constraints(
            PivotConstants.MAX_VELOCITY, PivotConstants.MAX_ACCELERATION
        )

        self.leader = rev.CANSparkMax(PivotConstants.LEADER_MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)
        self.controller = self.leader.getPIDController()
        self.encoder = self.leader.getEncoder()
        self.follower = rev.CANSparkMax(PivotConstants.FOLLOWER_MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)

        self.feedforward = wpimath.controller.ArmFeedforward(
            PivotConstants.kS, PivotConstants.kG, PivotConstants.kV, PivotConstants.kA
        )

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

    def periodic(self) -> None:
        wpilib.SmartDashboard.putString("Pivot Mode", self.mode.name)

        if self.mode is MechanismMode.POSITION:
            profile = wpimath.trajectory.TrapezoidProfile(self.constraints, self.goal, self.setpoint)

            self.setpoint = profile.calculate(DELTA_TIME)

            wpilib.SmartDashboard.putNumber("Pivot Desired Position (deg)", self.setpoint.position)
            wpilib.SmartDashboard.putNumber("Pivot Desired Velocity (deg/s)", self.setpoint.velocity)

            self.controller.setReference(
                self.setpoint.position,
                rev.CANSparkMax.ControlType.kPosition,
                arbFeedforward=self.feedforward.calculate(self.setpoint.position, self.setpoint.velocity),
            )

    def rotate_to(self, angle: Rotation2d):
        """
        Make the arm rotate to a specified angle

        :param angle: The angle setpoint
        """
        self.mode = MechanismMode.POSITION
        self.goal = wpimath.trajectory.TrapezoidProfile.State(angle.degrees())

    def set_power(self, power: float):
        self.mode = MechanismMode.DIRECT
        self.leader.set(power)

    def reset_angle(self, reference: float = 0):
        self.encoder.setPosition(reference)

    def at_goal(self) -> bool:
        if self.mode is not MechanismMode.POSITION:
            raise Exception("Goal not set! Call rotate_to() first.")

        return within_tolerance(self.encoder.getPosition(), self.goal.position, PivotConstants.PID_TOLERANCE)

    @property
    def angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.encoder.getPosition())


class ArmWinch(commands2.SubsystemBase):
    """The winch component of the arm"""

    def __init__(self):
        commands2.SubsystemBase.__init__(self)

        self.setpoint = 0

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

    def periodic(self) -> None:
        wpilib.SmartDashboard.putNumber("Winch Desired Position (m)", self.setpoint)

    def extend_distance(self, distance: float):
        self.setpoint = distance
        self.controller.setReference(self.setpoint, rev.CANSparkMax.ControlType.kPosition)

    def set_power(self, power: float):
        self.motor.set(power)

    def at_setpoint(self) -> bool:
        return within_tolerance(self.encoder.getPosition(), self.setpoint, WinchConstants.PID_TOLERANCE)


class ArmStructure(commands2.SubsystemBase):
    """
    A structure that combines the winch and joint components. Instantiate this instead of the components seperately.
    """

    def __init__(self):
        commands2.SubsystemBase.__init__(self)

        self.pivot = ArmPivot()
        self.winch = ArmWinch()

        # A Spike relay that controls the friction brake
        self.brake = wpilib.Relay(PivotConstants.BRAKE_PORT, wpilib.Relay.Direction.kForwardOnly)

    # TODO: Make the arm not kill itself by ramming into the floor if we command the angle to change but not the extension.
    # We could constantly run the maximum_extension function and adjust in periodic()

    def periodic(self) -> None:
        if self.pivot.mode is MechanismMode.POSITION:
            brake_value = (
                wpilib.Relay.Value.kOn
                if within_tolerance(self.pivot.setpoint.velocity, 0, 0.01)
                else wpilib.Relay.Value.kOff
            )
            self.brake.set(brake_value)

        wpilib.SmartDashboard.putBoolean("Brake Engaged", self.brake.get() is wpilib.Relay.Value.kOn)

    def extend_distance(self, distance: float):
        # Limit the arm's extension to avoid fouls
        limited_extension = min(maximum_extension(self.pivot.angle), distance)

        self.winch.extend_distance(limited_extension)

    def toggle_brake(self):
        value = wpilib.Relay.Value.kOn if self.brake.get() == wpilib.Relay.Value.kOff else wpilib.Relay.Value.kOff
        self.brake.set(value)

    def manual_winch_power_command(self, power: Callable[[], float]):
        return commands2.RunCommand(lambda: self.winch.set_power(power()), self.winch).alongWith(
            commands2.RunCommand(lambda: wpilib.SmartDashboard.putNumber("Winch Power", power()))  # type: ignore
        )

    def manual_pivot_power_command(self, power: Callable[[], float]):
        return commands2.RunCommand(lambda: self.pivot.set_power(power()), self.pivot).alongWith(
            commands2.RunCommand(lambda: wpilib.SmartDashboard.putNumber("Pivot Power", power()))  # type: ignore
        )

    def manual_pivot_position_command(self, delta_position: Callable[[], float]):
        def inner():
            self.pivot.goal.position += delta_position()

        return commands2.RunCommand(inner, self.pivot).beforeStarting(
            lambda: setattr(self.pivot, "mode", MechanismMode.POSITION)
        )

    def manual_winch_position_command(self, delta_position: Callable[[], float]):
        return commands2.RunCommand(
            lambda: self.winch.extend_distance(self.winch.setpoint + delta_position()), self.winch
        )

    def toggle_brake_command(self):
        return commands2.InstantCommand(self.toggle_brake, self)

    def stow_command(self):
        """Command to move arm in and upright"""

        return (
            commands2.InstantCommand(lambda: self.extend_distance(0), self.winch)
            # 90 degrees is upright
            .alongWith(
                commands2.RunCommand(lambda: self.pivot.rotate_to(Rotation2d.fromDegrees(90)), self.pivot)
            ).until(self.pivot.at_goal)
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


def within_tolerance(actual: float, reference: float, tolerance: float) -> bool:
    return reference - tolerance <= actual <= reference + tolerance
