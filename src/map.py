import math

import ctre
import robotpy_apriltag as apriltag

from swervelib import (
    CTRESwerveParameters,
    CTRESwerveModuleParameters,
    ModuleCorner,
    CanFDDeviceID,
    u,
    VisionParameters,
    CameraDefinition,
)
from wpimath.geometry import Translation2d, Rotation2d, Transform3d, Translation3d, Rotation3d


class DrivetrainConstants:
    FIELD_RELATIVE = False
    OPEN_LOOP = True
    TRACK_WIDTH = (24.75 * u.inch).m_as(u.m)
    WHEEL_BASE = (24.75 * u.inch).m_as(u.m)
    # fmt: off
    SWERVE_PARAMS = CTRESwerveParameters(
        wheel_circumference=4 * math.pi * u.inch,  # SDS Wheel Circumference

        drive_open_loop_ramp=0.25,
        drive_closed_loop_ramp=0,
        # angle_ramp=0.1,
        angle_ramp=0,

        # Mk4i L2 Gear Ratios
        drive_gear_ratio=6.75 / 1,
        angle_gear_ratio=(150 / 7) / 1,

        max_speed=5.12 * (u.m / u.s),
        # TODO: Measure max angular velocity
        max_angular_velocity=11.5 * (u.rad / u.s),

        angle_continuous_current_limit=25,
        angle_peak_current_limit=40,
        angle_peak_current_duration=0.01,
        angle_enable_current_limit=True,

        drive_continuous_current_limit=35,
        drive_peak_current_limit=60,
        drive_peak_current_duration=0.01,
        drive_enable_current_limit=True,

        angle_kP=0.11,
        # angle_kP=0.3,
        # angle_kP=0.25,
        angle_kI=0,
        angle_kD=0,
        angle_kF=0,

        drive_kP=0.12465,
        drive_kI=0,
        drive_kD=0,
        drive_kF=0,

        # drive_kS=0.029453 / 12,
        # drive_kV=2.3172 / 12,
        # drive_kA=0.051392 / 12,
        drive_kS=0.16954 / 12,
        drive_kV=2.1535 / 12,
        drive_kA=0.27464 / 12,

        angle_neutral_mode=ctre.NeutralMode.Brake,
        drive_neutral_mode=ctre.NeutralMode.Coast,

        invert_angle_motor=True,
        invert_drive_motor=False,
        invert_angle_encoder=False,

        invert_gyro=False,
        gyro_id=0,
    )
    SWERVE_MODULE_PARAMS = (
        CTRESwerveModuleParameters(
            corner=ModuleCorner.FRONT_LEFT,
            relative_position=Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            angle_offset=Rotation2d.fromDegrees(329.765),
            drive_motor_id=CanFDDeviceID(4),
            angle_motor_id=CanFDDeviceID(3),
            angle_encoder_id=CanFDDeviceID(0),
            fake=True,
        ),
        CTRESwerveModuleParameters(
            corner=ModuleCorner.FRONT_RIGHT,
            relative_position=Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            angle_offset=Rotation2d.fromDegrees(156.884),
            drive_motor_id=CanFDDeviceID(1),
            angle_motor_id=CanFDDeviceID(6),
            angle_encoder_id=CanFDDeviceID(1),
            fake=True,
        ),
        CTRESwerveModuleParameters(
            corner=ModuleCorner.BACK_LEFT,
            relative_position=Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            angle_offset=Rotation2d.fromDegrees(40.166),
            drive_motor_id=CanFDDeviceID(7),
            angle_motor_id=CanFDDeviceID(2),
            angle_encoder_id=CanFDDeviceID(2),
            fake=True,
        ),
        CTRESwerveModuleParameters(
            corner=ModuleCorner.BACK_RIGHT,
            relative_position=Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            angle_offset=Rotation2d.fromDegrees(210.410),
            drive_motor_id=CanFDDeviceID(5),
            angle_motor_id=CanFDDeviceID(0),
            angle_encoder_id=CanFDDeviceID(3),
            fake=True,
        ),
    )
    # fmt: on


VISION_PARAMS = VisionParameters(
    [
        CameraDefinition("front-apriltag", Transform3d(Translation3d(0, 0, 0), Rotation3d(0, 0, 0))),
    ],
    apriltag.loadAprilTagLayoutField(apriltag.AprilTagField.k2023ChargedUp),
)


class RobotDimensions:
    PIVOT_TO_FLOOR = (15 * u.inch).m_as(u.m)
    PIVOT_TO_MAX_VERTICAL_EXTENSION = (63 * u.inch).m_as(u.m)
    PIVOT_TO_MAX_HORIZONTAL_EXTENSION = (63 * u.inch).m_as(u.m)
    # TODO: Find real value
    MAX_EXTENSION_FROM_PIVOT = math.inf

    # Includes bumpers
    BASE_WIDTH = (37 * u.inch).m_as(u.m)
    BASE_LENGTH = (37 * u.inch).m_as(u.m)

    # FIELD_WIDTH = (655 * u.inch).m_as(u.m)


class AutoConstants:
    BALANCE_kP = -1
    BALANCE_MAX_EFFORT = 1


class PivotConstants:
    RELATIVE_POSITION = Translation3d(0, 0, 0)

    LEADER_MOTOR_PORT = 1
    FOLLOWER_MOTOR_PORT = 2
    BRAKE_PORT = 0

    kS = 0
    kG = 0
    kV = 0
    kA = 0

    # TODO: Find constants
    MAX_VELOCITY = 0
    MAX_ACCELERATION = 0

    PID_TOLERANCE = 2


class WinchConstants:
    MOTOR_PORT = 3

    PID_TOLERANCE = (1 * u.inch).m_as(u.m)


class ClawConstants:
    MOTOR_PORT = 4
    ANALOG_DISTANCE_SENSOR_PORT = 0
