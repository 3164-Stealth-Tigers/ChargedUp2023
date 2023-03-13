import commands2.button
import wpilib
import wpimath.geometry
import wpimath.trajectory

from commands import (
    BalanceCommand,
    VisualizeTargetCommand,
    CycleCommand,
    ReachTargetCommand,
    AlignToGridCommand,
    ReachNearestTargetCommand,
)
from subsystems.arm import ArmStructure
from subsystems.claw import Claw
from swervelib import Swerve

from map import DrivetrainConstants, VISION_PARAMS
from oi import XboxDriver, XboxOperator, LabTestXboxOperator
from tests import *


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
        self.claw = Claw()

        # Joysticks are plugged into the driver laptop and used during the teleop period to control the robot
        # Each joystick is plugged into a port, ranging from 0 to 5
        self.driver_stick = XboxDriver(0)
        self.operator_stick = LabTestXboxOperator(1)

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

        self.arm_cycle_cmd = CycleCommand(
            ReachNearestTargetCommand(ReachNearestTargetCommand.TargetHeight.BOTTOM, self.swerve, self.arm),
            ReachNearestTargetCommand(ReachNearestTargetCommand.TargetHeight.MID, self.swerve, self.arm),
            ReachNearestTargetCommand(ReachNearestTargetCommand.TargetHeight.TOP, self.swerve, self.arm),
            run_first_command_on_init=False,
        )
        self.arm_cycle_cmd.addRequirements(self.arm)
        self.arm.setDefaultCommand(
            self.arm_cycle_cmd.alongWith(
                self.arm.manual_pivot_command(self.operator_stick.pivot),
                self.arm.manual_winch_command(self.operator_stick.extend),
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

    def configure_button_bindings(self):
        """Bind buttons on the Xbox controllers to run Commands"""
        self.driver_stick.balance.whileTrue(BalanceCommand(self.swerve))
        self.driver_stick.reset_gyro.onTrue(commands2.InstantCommand(self.swerve.zero_heading))
        # TODO: Test on real robot
        self.driver_stick.align.whileTrue(
            AlignToGridCommand(
                self.swerve,
                wpimath.trajectory.TrajectoryConfig(2, 4),
                wpimath.trajectory.TrapezoidProfileRadians.Constraints(2, 4),
            )
        )

        self.operator_stick.cycle_next_height.onTrue(commands2.InstantCommand(self.arm_cycle_cmd.next))
        self.operator_stick.cycle_previous_height.onTrue(commands2.InstantCommand(self.arm_cycle_cmd.previous))
        self.operator_stick.stow.onTrue(
            commands2.SequentialCommandGroup(
                commands2.PrintCommand("Stowing arm"),
                self.arm.stow_command(),
                commands2.PrintCommand("Finished stowing arm"),
            )
        )

        # TODO: Temporary
        self.operator_stick.stick.Y().onTrue(
            commands2.RunCommand(
                lambda: self.arm.pivot.rotate_to(wpimath.geometry.Rotation2d.fromDegrees(90)), self.arm.pivot
            )
        )

        # fmt: off
        self.operator_stick.intake \
            .whileTrue(self.claw.intake_command()) \
            .onTrue(commands2.PrintCommand("Intaking")) \
            .onFalse(commands2.PrintCommand("Stopped"))
        self.operator_stick.outtake \
            .whileTrue(self.claw.outtake_command()) \
            .onTrue(commands2.PrintCommand("Outtaking")) \
            .onFalse(commands2.PrintCommand("Stopped"))
        # fmt: on

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

    def get_field_tests(self) -> list[TestCommand]:
        return [ExampleTest()]
