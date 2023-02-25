import json
import math
import pathlib

import commands2
import wpilib
import wpimath.controller
from wpimath.geometry import Translation2d

import swervelib
from map import AutoConstants
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
            unit = u.parse_expression(data["unit"])
            self.TARGETS = {
                k: wpimath.geometry.Translation2d((v[0] * unit).m_as(u.m), (v[1] * unit).m_as(u.m))
                for (k, v) in data["elements"].items()
            }
            print(self.TARGETS)

    def execute(self) -> None:
        # Determine which target to track by picking the closest one
        robot_translation = self.swerve.pose.translation()
        nearest = self.nearest(robot_translation, self.TARGETS)

        # Clamp/multiply the distance value to get a value between -1 and 1
        display_distance = clamp(nearest[2], -1, 1)

        # Display that value as a number on SmartDashboard
        wpilib.SmartDashboard.putNumber("Target Gauge", display_distance)
        wpilib.SmartDashboard.putString("Tracking Target", nearest[0])

    @staticmethod
    def nearest(translation: Translation2d, elements: dict[str, Translation2d]) -> (str, Translation2d, float):
        nearest = ("None", Translation2d(), math.inf)
        for (name, other_translation) in elements.items():
            distance = translation.distance(other_translation)
            if distance < nearest[2]:
                nearest = (name, other_translation, distance)
        return nearest


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


def clamp(num, max_value, min_value):
    return max(min(num, max_value), min_value)
