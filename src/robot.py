from typing import Optional

import commands2
import wpilib

from container import RobotContainer


class Robot(commands2.TimedCommandRobot):
    autonomous_command: Optional[commands2.Command] = None

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
        self.scheduler.cancelAll()


if __name__ == "__main__":
    wpilib.run(Robot)
