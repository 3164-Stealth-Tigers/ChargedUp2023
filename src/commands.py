import enum
import functools
import json
import math
import pathlib
from typing import Optional

import commands2
import wpilib
import wpimath.controller
import wpimath.trajectory
from wpimath.geometry import (
    Translation2d,
    Translation3d,
    Pose2d,
    Rotation2d,
    Transform2d,
    Pose3d,
    Rotation3d,
    Transform3d,
)

import swervelib
from map import AutoConstants, RobotDimensions
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
        robot_pose = self.swerve.pose
        nearest_target = nearest(robot_pose, field_elements())

        # Clamp/multiply the distance value to get a value between -1 and 1
        display_distance = clamp(nearest_target[2], -1, 1)

        # Display that value as a number on SmartDashboard
        # TODO: Account for robot dimensions when calculating error from the target
        wpilib.SmartDashboard.putString("Tracking Target", nearest_target[0])
        wpilib.SmartDashboard.putNumber("Target Diff (x)", robot_pose.x - nearest_target[1].x)
        wpilib.SmartDashboard.putNumber("Target Diff (y)", robot_pose.y - nearest_target[1].y)


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


class ReachNearestTargetCommand(ReachTargetCommand):
    class TargetHeight(enum.Enum):
        # Derived from the spreadsheet
        BOTTOM = Transform3d(Translation3d(0, 0, 0.001), Rotation3d())
        MID = Transform3d(Translation3d((21.87 * u.inch).m_as(u.m), 0, (34.065 * u.inch).m_as(u.m)), Rotation3d())
        TOP = Transform3d(Translation3d((38.896 * u.inch).m_as(u.m), 0, (46.065 * u.inch).m_as(u.m)), Rotation3d())

    def __init__(self, height: TargetHeight, swerve: swervelib.Swerve, arm_: ArmStructure):
        ReachTargetCommand.__init__(self, Translation3d(), swerve, arm_)

        self.offset = height.value

    def execute(self) -> None:
        # Acquire the nearest GRID element
        nearest_element = Pose3d(nearest(self.swerve.pose, field_elements())[1])

        # Offset the target's pose to acquire the bottom, mid, or high pylon's translation
        self.target = nearest_element.transformBy(self.offset)
        wpilib.SmartDashboard.putString(
            "debug_TARGET_GLOBAL_POS", f"x: {self.target.x}, y: {self.target.y}, z: {self.target.z}"
        )
        wpilib.SmartDashboard.putString(
            "CLAW_DESIRED_POSE", f"x: {self.target.x}, y: {self.target.y}, z: {self.target.z}"
        )

        super().execute()


class CycleCommand(commands2.CommandBase):
    """Command that can switch between multiple other commands with two methods"""

    def __init__(self, *commands: commands2.Command, run_first_command_on_init=True):
        commands2.CommandBase.__init__(self)

        self._commands = commands
        self._index = 0
        self._max = len(commands) - 1
        self._current_command: Optional[commands2.Command] = None
        self.run_first = run_first_command_on_init

        for command in commands:
            command.setGrouped(True)
            self.addRequirements(command.getRequirements())

    def next(self):
        """Switch to the next command."""
        if not (self._current_command is None or self._current_command.isFinished()):
            self._current_command.end(True)

        self._index = min(self._index + 1, self._max)
        self._current_command = self._commands[self._index]
        self._current_command.initialize()

    def previous(self):
        """Switch to the previous command."""
        if not (self._current_command is None or self._current_command.isFinished()):
            self._current_command.end(True)

        self._index = max(self._index - 1, 0)
        self._current_command = self._commands[self._index]
        self._current_command.initialize()

    def initialize(self) -> None:
        self._index = 0

        if self.run_first:
            self._current_command = self._commands[0]
            self._current_command.initialize()

    def execute(self) -> None:
        current_command = self._current_command
        if current_command is None:
            return

        current_command.execute()
        if current_command.isFinished():
            current_command.end(False)
            self._current_command = None

    def end(self, interrupted: bool) -> None:
        current_command = self._current_command
        if current_command is not None:
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
        robot_pose = self.swerve.pose
        nearest_target = nearest(robot_pose, field_elements())

        # Offset the target for the robot's dimensions.
        # Also, add or subtract depending on which side of the field we're on.
        # Transform negatively no matter which side of the field the element is on
        # because the elements are represented as poses with rotations, and a transform is relative to its pose.
        offset = Transform2d(-RobotDimensions.BASE_LENGTH / 2, 0, 0)
        target_pose = nearest_target[1].transformBy(offset)

        # TODO: Rotate to correct orientation for each target
        trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
            self.swerve.pose,
            [],
            target_pose,
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
def field_elements() -> dict[str, Pose2d]:
    file_path = pathlib.Path(__file__).resolve().parent / "resources" / "field_elements.json"
    with open(file_path, "r") as f:
        data = json.load(f)

        # Parse the unit from the JSON so that we know what unit to convert to metres
        unit = u.parse_expression(data["Units"])

        # Construct a dictionary with the target's name/ID as the key and a tuple containing x and y as the value
        # Convert to metres
        targets = {
            k: Pose2d((v["x"] * unit).m_as(u.m), (v["y"] * unit).m_as(u.m), Rotation2d.fromDegrees(v["Rotation"]))
            for (k, v) in data["FieldElements"].items()
        }
        return targets


def nearest(reference_pose: Pose2d, elements: dict[str, Pose2d]) -> tuple[str, Pose2d, float]:
    nearest_target = ("None", Pose2d(), math.inf)
    for (name, other_pose) in elements.items():
        distance = reference_pose.translation().distance(other_pose.translation())
        if distance < nearest_target[2]:
            nearest_target = (name, other_pose, distance)
    return nearest_target


def clamp(num, max_value, min_value):
    return max(min(num, max_value), min_value)
