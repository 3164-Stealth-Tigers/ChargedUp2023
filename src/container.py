import commands2.button
import wpilib
import wpimath.geometry
import wpimath.trajectory

from commands.game import BalanceCommand, VisualizeTargetCommand, AlignToGridCommand, ReachNearestTargetCommand
from commands.factories import CycleCommand
from subsystems.arm import ArmStructure
from subsystems.claw import Claw
from swervelib import Swerve

from map import DrivetrainConstants, VISION_PARAMS
from oi import XboxDriver, XboxOperator, LabTestXboxOperator
from swervelib.dummy import Dummy
from tests import *

ARM_POWER_TEST = True


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
            None,  # VISION_PARAMS,
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

        # Run either the position or direct power commands... not both
        if ARM_POWER_TEST:
            self.arm_cycle_cmd = Dummy()
            self.arm.pivot.setDefaultCommand(self.arm.manual_pivot_power_command(self.operator_stick.pivot))
            self.arm.winch.setDefaultCommand(self.arm.manual_winch_power_command(self.operator_stick.extend))
        else:
            self.arm_cycle_cmd = CycleCommand(
                ReachNearestTargetCommand(ReachNearestTargetCommand.TargetHeight.BOTTOM, self.swerve, self.arm),
                ReachNearestTargetCommand(ReachNearestTargetCommand.TargetHeight.MID, self.swerve, self.arm),
                ReachNearestTargetCommand(ReachNearestTargetCommand.TargetHeight.TOP, self.swerve, self.arm),
                run_first_command_on_init=False,
            )
            """
            self.arm_cycle_cmd = CycleCommand(
                self.arm.rotate_and_extend_command(wpimath.geometry.Rotation2d(), 0),
                self.arm.rotate_and_extend_command(wpimath.geometry.Rotation2d(), 0),
                self.arm.rotate_and_extend_command(wpimath.geometry.Rotation2d(), 0),
                run_first_command_on_init=False,
            )
            """
            self.arm_cycle_cmd.addRequirements(self.arm)

            # TODO: Add multiplier for delta position
            self.arm.setDefaultCommand(
                self.arm_cycle_cmd.alongWith(
                    self.arm.manual_pivot_position_command(lambda: self.operator_stick.pivot() * 1),
                    self.arm.manual_winch_position_command(lambda: self.operator_stick.extend() * 1),
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
                wpimath.trajectory.TrajectoryConfig(
                    self.swerve.swerve_params.max_speed, DrivetrainConstants.MAX_ACCELERATION
                ),
                wpimath.trajectory.TrapezoidProfileRadians.Constraints(
                    self.swerve.swerve_params.max_angular_velocity, DrivetrainConstants.MAX_ANGULAR_ACCELERATION
                ),
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
        self.operator_stick.toggle_brake.onTrue(self.arm.toggle_brake_command())
        self.operator_stick.reset_arm_angle.onTrue(commands2.InstantCommand(lambda: self.arm.pivot.reset_angle(90)))
        self.operator_stick.reset_winch_extension.onTrue(commands2.InstantCommand(self.arm.winch.reset_distance))

        # TODO: Temporary
        """
        self.operator_stick.stick.Y().onTrue(
            commands2.RunCommand(
                lambda: self.arm.pivot.rotate_to(wpimath.geometry.Rotation2d.fromDegrees(90)), self.arm.pivot
            )
        )
        """

        self.operator_stick.intake.whileTrue(self.claw.intake_command()).onFalse(self.claw.stop_command())
        self.operator_stick.outtake.whileTrue(self.claw.outtake_command()).onFalse(self.claw.stop_command())

    def get_autonomous_command(self) -> commands2.Command:
        return self.chooser.getSelected()

    def add_autonomous_routines(self):
        """Add routines to the autonomous picker"""
        trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
            wpimath.geometry.Pose2d(0, 0, 0),
            [],
            wpimath.geometry.Pose2d(1, 0, 0),
            wpimath.trajectory.TrajectoryConfig(
                self.swerve.swerve_params.max_speed, DrivetrainConstants.MAX_ACCELERATION
            ),
        )
        theta_constraints = wpimath.trajectory.TrapezoidProfileRadians.Constraints(
            self.swerve.swerve_params.max_angular_velocity, DrivetrainConstants.MAX_ANGULAR_ACCELERATION
        )
        self.chooser.setDefaultOption(
            "Trajectory Test", self.swerve.follow_trajectory_command(trajectory, True, theta_constraints)
        )

    def get_field_tests(self) -> list[TestCommand]:
        return [ExampleTest()]
