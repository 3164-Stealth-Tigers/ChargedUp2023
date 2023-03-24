import typing

from field_test import TestCommand
from swervelib import u

if typing.TYPE_CHECKING:
    from container import RobotContainer


class ExampleTest(TestCommand):
    def __init__(self):
        super().__init__("Example Test", 3)

    def passed(self) -> bool:
        return True


class ExtensionTest(TestCommand):
    def __init__(self, container: "RobotContainer"):
        super().__init__("Extension Test", 3)
        self.container = container
        self.distance = (20 * u.inch).m_as(u.m)

    def passed(self) -> bool:
        return self.container.arm.pivot.at_goal()

    def execute(self) -> None:
        self.container.arm.extend_distance(self.distance)
