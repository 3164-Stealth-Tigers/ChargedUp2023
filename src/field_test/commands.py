from abc import abstractmethod, ABCMeta

import commands2
import wpilib


class CommandABCMeta(ABCMeta, type(commands2.CommandBase)):
    pass


class TestCommand(commands2.CommandBase, metaclass=CommandABCMeta):
    @abstractmethod
    def passed(self) -> bool:
        """
        Return True when the test has passed. If pass_fast is True,
        then the command will end prematurely when this function is True.
        """

    def failed(self) -> bool:
        """
        Return True when the test has failed. The test will end prematurely when this function is True. Defining
        this function is optional, because the test will also fail when it has not passed in an allotted time period.
        """
        return False

    @property
    def time_elapsed(self) -> float:
        """The time elapsed since the test began in seconds"""
        return self._timer.get()

    def __init__(self, name: str, time_limit_seconds: float, pass_fast: bool = False):
        """
        Constructor for a TestCommand

        :param name: The name that the TestSuite will display
        :param time_limit_seconds: The maximum amount of time (in seconds) that the test will run for before failing
        :param pass_fast: If True, the test will end as soon as the pass condition is true. Otherwise, the pass condition
        will only be evaluated after time_limit_seconds has elapsed.
        """
        super().__init__()

        self._timer = wpilib.Timer()
        self.time_limit = time_limit_seconds
        self.name = name
        self._should_past_fast = pass_fast

    def initialize(self) -> None:
        self._timer.restart()

    def isFinished(self) -> bool:
        return (self._should_past_fast and self.passed()) or self.failed() or self._timer.hasElapsed(self.time_limit)


class TestSuite(commands2.CommandBase):
    def __init__(self, *tests: TestCommand, fail_fast: bool = True):
        """
        Constructor for a TestSuite

        :param tests: At least one test to run
        :param fail_fast: If True, the test suite will halt after one test fails. Otherwise, all tests will run.
        """
        super().__init__()

        if not tests:
            raise ValueError("At least one test must be added to the test suite!")

        self.tests = tests
        self.should_fail_fast = fail_fast
        self.index = 0
        self.stop_early = False

        for cmd in tests:
            cmd.setGrouped(True)

    def initialize(self) -> None:
        print(f"Initializing field test suite with {len(self.tests)} test{'s' if len(self.tests) > 1 else ''}...")

        self.index = 0
        current_test: TestCommand = self.tests[self.index]
        print(f"Running {current_test.name}...")
        current_test.initialize()

    def execute(self) -> None:
        current_test: TestCommand = self.tests[self.index]
        current_test.execute()

        if current_test.isFinished():
            current_test.end(False)

            # Inform the user whether the test passed or failed
            print(
                f"{current_test.name} {'PASSED' if current_test.passed() else 'FAILED'} in {current_test.time_elapsed} seconds."
            )

            # Fail fast before beginning the next command
            if self.should_fail_fast and not current_test.passed():
                self.stop_early = True
                return

            # Start the next test
            if self.index + 1 >= len(self.tests):
                return
            self.index += 1
            self.tests[self.index].initialize()

    def end(self, interrupted: bool) -> None:
        print(f"Finished running {self.index + 1}/{len(self.tests)} tests.")

        if interrupted:
            self.tests[self.index].end(True)

    def isFinished(self) -> bool:
        current_test: TestCommand = self.tests[self.index]
        return (self.index == len(self.tests) - 1 and current_test.isFinished()) or self.stop_early
