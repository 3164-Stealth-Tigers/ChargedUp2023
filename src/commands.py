import functools
import json
import math
import pathlib
from typing import Optional

import commands2
import wpilib
import wpimath.controller
import wpimath.trajectory
from wpimath.geometry import Translation2d, Translation3d

import swervelib
from map import AutoConstants
from subsystems import arm
from subsystems.arm import ArmStructure
from swervelib import u


class VisualizeTargetCommand(commands2.CommandBase):
    def __init__(self, swerve: swervelib.Swerve):
        commands2.CommandBase.__init__(self)

        self.swerve = swerve
        wpilib.SmartDashboard.putNumber("Target Gauge", 0)
        wpilib.SmartDashboard.putString("Tracking Target", "None")

        # Preload the targets on the field from a JSON file
        field_elements()

    def execute(self) -> None:
        # Determine which target to track by picking the closest one
        robot_translation = self.swerve.pose.translation()
        nearest_target = nearest(robot_translation, field_elements())

        # Clamp/multiply the distance value to get a value between -1 and 1
        display_distance = clamp(nearest_target[2], -1, 1)

        # Display that value as a number on SmartDashboard
        # TODO: Account for robot dimensions when calculating error from the target
        wpilib.SmartDashboard.putString("Tracking Target", nearest_target[0])
        wpilib.SmartDashboard.putNumber("Target Diff (x)", robot_translation.x - nearest_target[1].x)
        wpilib.SmartDashboard.putNumber("Target Diff (y)", robot_translation.y - nearest_target[1].y)


"""
Single or multiple axis positional control with driver control on the leftover axis(es)

Use the drive() function in closed loop and field relative mode. Run a PID loop for each axis that I want to use positional control on.
Feed the result of each PID loop into the appropriate parameter of the drive() function. For the remaining axis,
feed in the joystick value as done in TeleopCommand.

I will be running a positional PID loop to figure out the desired velocity of the entire robot in one or more directions.
Then, the drive() function will fuse those results together.
"""


class BalanceCommand(commands2.CommandBase):
    def __init__(self, swerve: swervelib.Swerve):
        commands2.CommandBase.__init__(self)

        self.swerve = swerve

        self.controller = wpimath.controller.PIDController(AutoConstants.BALANCE_kP, 0, 0)
        self.controller.setSetpoint(0)
        wpilib.SmartDashboard.putData("Balance PID", self.controller)

    def execute(self) -> None:
        pitch = self.swerve.pitch.degrees()
        pitch = clamp(pitch, AutoConstants.BALANCE_MAX_EFFORT, -AutoConstants.BALANCE_MAX_EFFORT)
        output = self.controller.calculate(pitch)
        wpilib.SmartDashboard.putNumber("Balance PID Output", output)
        self.swerve.drive(Translation2d(output, 0) * self.swerve.swerve_params.max_speed, 0, False, True)

    def end(self, interrupted: bool) -> None:
        self.swerve.drive(Translation2d(0, 0), 0, False, True)


class ReachTargetCommand(commands2.CommandBase):
    def __init__(self, target: Translation3d, swerve: swervelib.Swerve, arm_: ArmStructure):
        commands2.CommandBase.__init__(self)

        self.target = target
        self.swerve = swerve
        self.arm = arm_

        self.addRequirements([swerve, arm_])

    def execute(self) -> None:
        robot_translation_2d = self.swerve.pose.translation()
        robot_translation_3d = Translation3d(robot_translation_2d.x, robot_translation_2d.y, 0)

        desired_arm_angle = arm.angle_to(self.target, robot_translation_3d)
        desired_extension = arm.extension_to(self.target, desired_arm_angle, robot_translation_3d)

        self.arm.pivot.rotate_to(desired_arm_angle)
        self.arm.extend_distance(desired_extension)


class CycleCommand(commands2.CommandBase):
    """Command that can switch between multiple other commands with two methods"""

    def __init__(self, *commands: commands2.Command, run_first_command_on_init=True):
        commands2.CommandBase.__init__(self)

        self._commands = commands
        self._index = 0
        self._max = len(commands) - 1
        self._current_command: Optional[commands2.Command] = commands[0]
        self.run_first = run_first_command_on_init

        for command in commands:
            command.setGrouped(True)
            self.addRequirements(command.getRequirements())

    def next(self):
        """Switch to the next command."""
        self._index = min(self._index + 1, self._max)
        self._current_command = self._commands[self._index]
        self._current_command.initialize()

    def previous(self):
        """Switch to the previous command."""
        self._index = max(self._index - 1, 0)
        self._current_command = self._commands[self._index]
        self._current_command.initialize()

    def initialize(self) -> None:
        self._index = 0
        self._current_command = self._commands[0]

        if self.run_first:
            self._current_command.initialize()

    def execute(self) -> None:
        current_command = self._current_command
        if not current_command:
            return

        current_command.execute()
        if current_command.isFinished():
            current_command.end(False)
            self._current_command = None

    def end(self, interrupted: bool) -> None:
        current_command = self._current_command
        if current_command:
            current_command.end(interrupted)


class AlignToGridCommand(commands2.CommandBase):
    def __init__(
        self,
        swerve: swervelib.Swerve,
        trajectory_config: wpimath.trajectory.TrajectoryConfig,
        theta_controller_constraints: wpimath.trajectory.TrapezoidProfileRadians.Constraints,
    ):
        commands2.CommandBase.__init__(self)

        self.swerve = swerve
        self.trajectory_config = trajectory_config
        self.theta_controller_constraints = theta_controller_constraints
        self.inner_command = commands2.Command()

    def initialize(self) -> None:
        robot_translation = self.swerve.pose.translation()
        nearest_target = nearest(robot_translation, field_elements())

        # Offset the target for the robot's dimensions
        # TODO: Load robot dimensions
        target_translation = nearest_target[1] - Translation2d()

        # TODO: Rotate to correct orientation for each target
        trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
            self.swerve.pose,
            [],
            wpimath.geometry.Pose2d(target_translation, wpimath.geometry.Rotation2d(0)),
            self.trajectory_config,
        )
        self.swerve.field.getObject("traj").setTrajectory(trajectory)

        self.inner_command = self.swerve.follow_trajectory_command(trajectory, True, self.theta_controller_constraints)

        # Initialize the actual command to follow the path
        self.inner_command.initialize()

    def execute(self) -> None:
        self.inner_command.execute()

    def end(self, interrupted: bool) -> None:
        self.inner_command.end(interrupted)

    def isFinished(self) -> bool:
        return self.inner_command.isFinished()


@functools.cache
def field_elements() -> dict[str, wpimath.geometry.Translation2d]:
    file_path = pathlib.Path(__file__).resolve().parent / "resources" / "field_elements.json"
    with open(file_path, "r") as f:
        data = json.load(f)

        # Parse the unit from the JSON so that we know what unit to convert to metres
        unit = u.parse_expression(data["Units"])

        # Construct a dictionary with the target's name/ID as the key and a tuple containing x and y as the value
        # Convert to metres
        targets = {
            k: wpimath.geometry.Translation2d((v["x"] * unit).m_as(u.m), (v["y"] * unit).m_as(u.m))
            for (k, v) in data["FieldElements"].items()
        }
        return targets


def nearest(translation: Translation2d, elements: dict[str, Translation2d]) -> tuple[str, Translation2d, float]:
    nearest_target = ("None", Translation2d(), math.inf)
    for (name, other_translation) in elements.items():
        distance = translation.distance(other_translation)
        if distance < nearest_target[2]:
            nearest_target = (name, other_translation, distance)
    return nearest_target


def clamp(num, max_value, min_value):
    return max(min(num, max_value), min_value)
