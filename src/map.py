import math

import ctre
import robotpy_apriltag as apriltag

from wpimath.geometry import Translation2d, Rotation2d, Transform3d, Translation3d, Rotation3d

from swervelib import u
from swervelib.impl import (
    Falcon500CoaxialDriveComponent,
    Falcon500CoaxialAzimuthComponent,
    CoaxialSwerveModule,
    AbsoluteCANCoder,
)


class DrivetrainConstants:
    FIELD_RELATIVE = False
    OPEN_LOOP = True

    TRACK_WIDTH = (24.75 * u.inch).m_as(u.m)
    WHEEL_BASE = (24.75 * u.inch).m_as(u.m)

    MAX_VELOCITY = 4.00 * (u.m / u.s)
    MAX_ANGULAR_VELOCITY = 584 * (u.deg / u.s)

    DRIVE_PARAMS = Falcon500CoaxialDriveComponent.Parameters(
        wheel_circumference=4 * math.pi * u.inch,
        gear_ratio=6.75 / 1,  # SDS Mk4i L2
        max_speed=MAX_VELOCITY,
        open_loop_ramp_rate=0.25,
        closed_loop_ramp_rate=0,
        continuous_current_limit=40,
        peak_current_limit=60,
        peak_current_duration=0.01,
        neutral_mode=ctre.NeutralMode.Coast,
        kP=0.12465,
        kI=0,
        kD=0,
        kS=0.16954 / 12,
        kV=2.1535 / 12,
        kA=0.27464 / 12,
        invert_motor=False,
    )
    AZIMUTH_PARAMS = Falcon500CoaxialAzimuthComponent.Parameters(
        gear_ratio=150 / 7,  # SDS Mk4i
        max_angular_velocity=MAX_ANGULAR_VELOCITY,
        ramp_rate=0,
        continuous_current_limit=25,
        peak_current_limit=40,
        peak_current_duration=0.01,
        neutral_mode=ctre.NeutralMode.Brake,
        kP=0.3,
        kI=0,
        kD=0,
        invert_motor=True,
    )

    SWERVE_MODULES = [
        CoaxialSwerveModule(
            Falcon500CoaxialDriveComponent(4, DRIVE_PARAMS),
            Falcon500CoaxialAzimuthComponent(3, Rotation2d.fromDegrees(331.435), AZIMUTH_PARAMS, AbsoluteCANCoder(0)),
            Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        ),
        CoaxialSwerveModule(
            Falcon500CoaxialDriveComponent(1, DRIVE_PARAMS),
            Falcon500CoaxialAzimuthComponent(6, Rotation2d.fromDegrees(156.093), AZIMUTH_PARAMS, AbsoluteCANCoder(1)),
            Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        ),
        CoaxialSwerveModule(
            Falcon500CoaxialDriveComponent(7, DRIVE_PARAMS),
            Falcon500CoaxialAzimuthComponent(2, Rotation2d.fromDegrees(45.263), AZIMUTH_PARAMS, AbsoluteCANCoder(2)),
            Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        ),
        CoaxialSwerveModule(
            Falcon500CoaxialDriveComponent(5, DRIVE_PARAMS),
            Falcon500CoaxialAzimuthComponent(0, Rotation2d.fromDegrees(211.201), AZIMUTH_PARAMS, AbsoluteCANCoder(3)),
            Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        ),
    ]

    GYRO_PORT = 0
    INVERT_GYRO = True


class RobotDimensions:
    PIVOT_TO_FLOOR = (15 * u.inch).m_as(u.m)
    PIVOT_TO_MAX_VERTICAL_EXTENSION = (63 * u.inch).m_as(u.m)
    PIVOT_TO_MAX_HORIZONTAL_EXTENSION = (63 * u.inch).m_as(u.m)

    # To end of fully extended claw
    MAX_EXTENSION_FROM_PIVOT = (65.5 * u.inch).m_as(u.m)
    MIN_EXTENSION_FROM_PIVOT = (45.5 * u.inch).m_as(u.m)

    # Includes bumpers
    BASE_WIDTH = (37 * u.inch).m_as(u.m)
    BASE_LENGTH = (37 * u.inch).m_as(u.m)

    # FIELD_WIDTH = (655 * u.inch).m_as(u.m)


class AutoConstants:
    BALANCE_kP = -1
    BALANCE_MAX_EFFORT = 1


class PivotConstants:
    GEARING = 6 * (72 / 14)

    RELATIVE_POSITION = Translation3d(0, 0, 0)

    LEADER_MOTOR_PORT = 1
    FOLLOWER_MOTOR_PORT = 2
    BRAKE_PORT = 0

    kS = 0
    kG = 0
    kV = 0
    kA = 0

    # Maximum control effort. Only for testing purposes; leave at -1 and 1 respectively during competition.
    MIN_OUTPUT = -1
    MAX_OUTPUT = 1

    # TODO: Find constants
    MAX_VELOCITY = 0
    MAX_ACCELERATION = 0

    PID_TOLERANCE = 2


class WinchConstants:
    GEARING = 16
    SPOOL_CIRCUMFERENCE = (13.75 * u.mm).m_as(u.m)

    MOTOR_PORT = 5

    INVERTED = False

    PID_TOLERANCE = (1 * u.inch).m_as(u.m)


class ClawConstants:
    MOTOR_PORT = 3
    ANALOG_DISTANCE_SENSOR_PORT = 0
