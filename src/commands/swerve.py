import commands2
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

import swervelib


def ski_stop_command(swerve: swervelib.SwerveDrive):
    # fmt: off
    angles = (
        45, 315,  # Front Left, Front Right
        315, 45,  # Back Left, Back Right
    )
    # fmt: on

    states = tuple(SwerveModuleState(0, Rotation2d.fromDegrees(angle)) for angle in angles)

    return commands2.RunCommand(lambda: swerve.desire_module_states(states), swerve)
