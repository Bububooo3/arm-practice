import commands2
from wpilib import DataLogManager


class Robot(commands2.TimedCommandRobot):
    def robotInit(self) -> commands2.TimedCommandRobot:
        # Start recording NetworkTables to data log
        DataLogManager.start()

    def autonomousInit(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass