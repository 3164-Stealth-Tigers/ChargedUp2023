from typing import Optional

import commands2
import wpilib

import field_test
from container import RobotContainer


class Robot(commands2.TimedCommandRobot):
    autonomous_command: Optional[commands2.Command] = None
    test_suite: Optional[field_test.TestSuite] = None

    def robotInit(self) -> None:
        self.container = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()

    def autonomousInit(self) -> None:
        self.autonomous_command = self.container.get_autonomous_command()

        if self.autonomous_command:
            self.autonomous_command.schedule()

    def teleopInit(self) -> None:
        if self.autonomous_command:
            self.autonomous_command.cancel()

    def testInit(self) -> None:
        # LiveWindow needs to be disabled for the CommandScheduler to run
        wpilib.LiveWindow.setEnabled(False)

        self.test_suite = field_test.TestSuite(*self.container.get_field_tests(), fail_fast=True)

        self.scheduler.cancelAll()
        if self.test_suite:
            self.test_suite.schedule()

    def testPeriodic(self):
        pass

    def testExit(self):
        if self.test_suite:
            self.test_suite.cancel()


if __name__ == "__main__":
    wpilib.run(Robot)
