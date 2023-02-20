import commands2
import wpilib
import wpimath.controller
from wpimath.geometry import Translation2d

import swervelib
from map import AutoConstants


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
