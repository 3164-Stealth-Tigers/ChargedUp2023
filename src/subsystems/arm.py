"""Put a technical description of the telescoping arm and how it's controlled through code here."""
import commands2
import rev
from wpimath.geometry import Rotation2d

from map import ArmJointConstants


class Arm(commands2.SubsystemBase):
    def __init__(self):
        commands2.SubsystemBase.__init__(self)

        self.leader = rev.CANSparkMax(ArmJointConstants.LEADER_MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)
        follower = rev.CANSparkMax(ArmJointConstants.FOLLOWER_MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)
        follower.follow(self.leader)

    def set_angle(self, angle: Rotation2d):
        pass

    @staticmethod
    def maximum_length(angle: Rotation2d):
        pass
