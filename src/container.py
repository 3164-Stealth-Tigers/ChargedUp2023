import commands2.button
import wpilib

from commands import BalanceCommand, VisualizeTargetCommand
from subsystems.arm import ArmStructure
from swervelib import Swerve

from map import DrivetrainConstants, VISION_PARAMS
from oi import XboxDriver, XboxOperator


class RobotContainer:
    def __init__(self):
        """Constructor method"""
        # Configure the Driver Station not to report errors when a joystick isn't plugged in
        wpilib.DriverStation.silenceJoystickConnectionWarning(silence=True)

        # Subsystems represent self-contained parts of the robot (e.g. the drivetrain, arm, winch)
        # Subsystems expose Commands that can be arranged to make the robot run
        self.swerve = Swerve(
            DrivetrainConstants.SWERVE_MODULE_PARAMS,
            DrivetrainConstants.SWERVE_PARAMS,
            VISION_PARAMS,
        )
        self.arm = ArmStructure()

        # Joysticks are plugged into the driver laptop and used during the teleop period to control the robot
        # Each joystick is plugged into a port, ranging from 0 to 5
        self.driver_stick = XboxDriver(0)
        self.operator_stick = XboxOperator(1)

        # Default commands run whenever no other commands are scheduled
        # This included the teleop period, so code for teleop control should be set as the default command
        self.swerve.setDefaultCommand(
            self.swerve.teleop_command(
                self.driver_stick.forward,
                self.driver_stick.strafe,
                self.driver_stick.turn,
                DrivetrainConstants.FIELD_RELATIVE,
                DrivetrainConstants.OPEN_LOOP,
            ).alongWith(VisualizeTargetCommand(self.swerve))
        )

        self.arm.setDefaultCommand(
            self.arm.manual_command(
                self.operator_stick.pivot,
                self.operator_stick.extend,
            )
        )

        # Bind buttons to Commands
        self.configure_button_bindings()

        # Autonomous chooser component
        # Adds a dropdown menu to the Driver Station that allows users to pick which autonomous routine (Command group)
        # to run
        self.chooser = wpilib.SendableChooser()
        self.add_autonomous_routines()

        # Put the chooser on Smart Dashboard
        wpilib.SmartDashboard.putData(self.chooser)

        # Enable the camera feed
        # wpilib.CameraServer.launch()

    def configure_button_bindings(self):
        """Bind buttons on the Xbox controllers to run Commands"""
        self.driver_stick.balance.whileTrue(BalanceCommand(self.swerve))
        self.driver_stick.reset_gyro.onTrue(commands2.InstantCommand(self.swerve.zero_heading))

    def get_autonomous_command(self) -> commands2.Command:
        return self.chooser.getSelected()

    def add_autonomous_routines(self):
        """Add routines to the autonomous picker"""
        """
        trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
            wpimath.geometry.Pose2d(0, 0, 0),
            [],
            wpimath.geometry.Pose2d(1, 0, 0),
            wpimath.trajectory.TrajectoryConfig(2, 4)
        )
        constraints = wpimath.trajectory.TrapezoidProfileRadians.Constraints(2, 4)
        self.chooser.setDefaultOption("Trajectory Test", self.swerve.follow_trajectory_command(trajectory, True, constraints))
        """
