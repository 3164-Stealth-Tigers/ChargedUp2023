import commands2.button
import wpilib

from subsystems.arm import ArmStructure
from subsystems.claw import Claw
from commands.game import LiftArmCommand

from map import DrivetrainConstants
from oi import XboxDriver, XboxOperator
from swervelib.impl import SwerveDrive, PigeonGyro
from tests import *


class RobotContainer:
    def __init__(self):
        """Constructor method"""
        # Configure the Driver Station not to report errors when a joystick isn't plugged in
        wpilib.DriverStation.silenceJoystickConnectionWarning(silence=True)

        # Subsystems represent self-contained parts of the robot (e.g. the drivetrain, arm, winch)
        # Subsystems expose Commands that can be arranged to make the robot run
        self.arm = ArmStructure()
        self.claw = Claw()

        # Setup drive base
        gyro = PigeonGyro(DrivetrainConstants.GYRO_PORT, DrivetrainConstants.INVERT_GYRO)
        self.swerve = SwerveDrive(
            DrivetrainConstants.SWERVE_MODULES,
            gyro,
            DrivetrainConstants.MAX_VELOCITY,
            DrivetrainConstants.MAX_ANGULAR_VELOCITY,
        )

        # Joysticks are plugged into the driver laptop and used during the teleop period to control the robot
        # Each joystick is plugged into a port, ranging from 0 to 5
        self.driver_stick = XboxDriver(0)
        self.operator_stick = XboxOperator(1)

        # Default commands run whenever no other commands are scheduled
        # This included the teleop period, so code for teleop control should be set as the default command
        self.swerve_teleop_cmd = self.swerve.teleop_command(
            self.driver_stick.forward,
            self.driver_stick.strafe,
            self.driver_stick.turn,
            DrivetrainConstants.FIELD_RELATIVE,
            DrivetrainConstants.OPEN_LOOP,
        )
        self.swerve.setDefaultCommand(self.swerve_teleop_cmd)
        wpilib.SmartDashboard.putData("Swerve TeleOp Command", self.swerve_teleop_cmd)

        self.arm.pivot.setDefaultCommand(LiftArmCommand(self.operator_stick.pivot, self.arm))
        self.arm.winch.setDefaultCommand(self.arm.manual_winch_power_command(self.operator_stick.extend))

        # Bind buttons to Commands
        self.configure_button_bindings()

        # Autonomous chooser component
        # Adds a dropdown menu to the Driver Station that allows users to pick which autonomous routine (Command group)
        # to run
        self.chooser = wpilib.SendableChooser()
        self.add_autonomous_routines()

        # Put the chooser on Smart Dashboard
        wpilib.SmartDashboard.putData(self.chooser)

    def configure_button_bindings(self):
        """Bind buttons on the Xbox controllers to run Commands"""
        # self.driver_stick.balance.whileTrue(BalanceCommand(self.swerve, self.arm))
        self.driver_stick.reset_gyro.onTrue(commands2.InstantCommand(self.swerve.zero_heading))
        self.driver_stick.toggle_field_relative.onTrue(
            commands2.InstantCommand(self.swerve_teleop_cmd.toggle_field_relative)
        )
        # TODO: Re-impl ski stop

        self.operator_stick.intake.whileTrue(self.claw.intake_command()).onFalse(self.claw.stop_command())
        self.operator_stick.outtake.whileTrue(self.claw.outtake_command()).onFalse(self.claw.stop_command())

    def get_autonomous_command(self) -> commands2.Command:
        return self.chooser.getSelected()

    def add_autonomous_routines(self):
        """Add routines to the autonomous picker"""

    def get_field_tests(self) -> list[TestCommand]:
        return [ExampleTest()]
