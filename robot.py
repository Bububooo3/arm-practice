import commands2
import constants
from wpilib import DataLogManager


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        # Start recording NetworkTables to data log
        DataLogManager.start()

    def autonomousInit(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass