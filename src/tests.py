from field_test import TestCommand


class ExampleTest(TestCommand):
    def __init__(self):
        super().__init__("Example Test", 3)

    def passed(self) -> bool:
        return True