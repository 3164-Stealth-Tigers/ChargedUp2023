from typing import Optional

import commands2


class CycleCommand(commands2.CommandBase):
    """Command that can switch between multiple other commands with two methods"""

    def __init__(self, *commands: commands2.Command, run_first_command_on_init=True):
        commands2.CommandBase.__init__(self)

        self._commands = commands
        self._index = 0
        self._max = len(commands) - 1
        self._current_command: Optional[commands2.Command] = None
        self.run_first = run_first_command_on_init

        for command in commands:
            command.setGrouped(True)
            self.addRequirements(command.getRequirements())

    def next(self):
        """Switch to the next command."""
        if not (self._current_command is None or self._current_command.isFinished()):
            self._current_command.end(True)

        self._index = min(self._index + 1, self._max)
        self._current_command = self._commands[self._index]
        self._current_command.initialize()

    def previous(self):
        """Switch to the previous command."""
        if not (self._current_command is None or self._current_command.isFinished()):
            self._current_command.end(True)

        self._index = max(self._index - 1, 0)
        self._current_command = self._commands[self._index]
        self._current_command.initialize()

    def initialize(self) -> None:
        self._index = 0

        if self.run_first:
            self._current_command = self._commands[0]
            self._current_command.initialize()

    def execute(self) -> None:
        current_command = self._current_command
        if current_command is None:
            return

        current_command.execute()
        if current_command.isFinished():
            current_command.end(False)
            self._current_command = None

    def end(self, interrupted: bool) -> None:
        current_command = self._current_command
        if current_command is not None:
            current_command.end(interrupted)
