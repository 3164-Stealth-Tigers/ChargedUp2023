import json
import math
import pathlib

import commands2
import wpilib
import wpimath.controller
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

        # Load the targets on the field from a JSON file
        file_path = pathlib.Path(__file__).resolve().parent / "resources" / "field_elements.json"
        with open(file_path, "r") as f:
            data = json.load(f)

            # Parse the unit from the JSON so that we know what unit to convert to metres
            unit = u.parse_expression(data["Units"])

            # Construct a dictionary with the target's name/ID as the key and a tuple containing x and y as the value
            # Convert to metres
            self.TARGETS = {
                k: wpimath.geometry.Translation2d((v["x"] * unit).m_as(u.m), (v["y"] * unit).m_as(u.m))
                for (k, v) in data["FieldElements"].items()
            }

    def execute(self) -> None:
        # Determine which target to track by picking the closest one
        robot_translation = self.swerve.pose.translation()
        nearest = self.nearest(robot_translation, self.TARGETS)

        # Clamp/multiply the distance value to get a value between -1 and 1
        display_distance = clamp(nearest[2], -1, 1)

        # Display that value as a number on SmartDashboard
        # TODO: Account for robot dimensions when calculating error from the target
        wpilib.SmartDashboard.putString("Tracking Target", nearest[0])
        wpilib.SmartDashboard.putNumber("Target Diff (x)", robot_translation.x - nearest[1].x)
        wpilib.SmartDashboard.putNumber("Target Diff (y)", robot_translation.y - nearest[1].y)

    @staticmethod
    def nearest(translation: Translation2d, elements: dict[str, Translation2d]) -> (str, Translation2d, float):
        nearest = ("None", Translation2d(), math.inf)
        for (name, other_translation) in elements.items():
            distance = translation.distance(other_translation)
            if distance < nearest[2]:
                nearest = (name, other_translation, distance)
        return nearest


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
        robot_translation = self.swerve.pose.translation()
        robot_translation = Translation3d(robot_translation.x, robot_translation.y, 0)

        desired_arm_angle = arm.angle_to(self.target, robot_translation)
        desired_extension = arm.extension_to(self.target, desired_arm_angle, robot_translation)

        self.arm.pivot.rotate_to(desired_arm_angle)
        self.arm.extend_distance(desired_extension)


def clamp(num, max_value, min_value):
    return max(min(num, max_value), min_value)
