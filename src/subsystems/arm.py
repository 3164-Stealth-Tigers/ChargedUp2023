"""Put a technical description of the telescoping arm and how it's controlled through code here."""
import math

import commands2
import rev
import wpimath.controller
from wpimath.geometry import Rotation2d, Translation2d, Translation3d

from map import ArmJointConstants


"""
Possible ideas for counteracting gravity:
    * Have min, mid, and max extension values that we switch between
    * 
"""


class Arm(commands2.SubsystemBase):
    def __init__(self):
        commands2.SubsystemBase.__init__(self)

        self.leader = rev.CANSparkMax(ArmJointConstants.LEADER_MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)
        follower = rev.CANSparkMax(ArmJointConstants.FOLLOWER_MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)
        follower.follow(self.leader)

        self.controller = self.leader.getPIDController()

        self._config_motors()

    def _config_motors(self):
        pass

    def set_angle(self, angle: Rotation2d):
        self.controller.setReference(angle.degrees(), rev.CANSparkMax.ControlType.kSmartMotion)

    def extend(self, distance: float):
        pass

    @staticmethod
    def maximum_extension(angle: Rotation2d):
        pass

    @staticmethod
    def angle_to(target: Translation3d, robot_translation: Translation3d) -> Rotation2d:
        global_arm_translation = robot_translation + ArmJointConstants.RELATIVE_POSITION
        xy_distance = global_arm_translation.toTranslation2d().distance(target.toTranslation2d())
        z_distance = abs(target.z - global_arm_translation.z)
        angle = math.atan(z_distance / xy_distance)
        return Rotation2d(angle)

    @staticmethod
    def extension_to(target: Translation3d, angle: Rotation2d, robot_translation: Translation3d) -> float:
        global_arm_translation = robot_translation + ArmJointConstants.RELATIVE_POSITION
        z_distance = abs(target.z - global_arm_translation.z)
        hyp = z_distance / angle.sin()
        return hyp
