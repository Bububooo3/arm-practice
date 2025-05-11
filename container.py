import commands2
from wpilib import DriverStation

from constants import *
from subsystems import arm
from pint import UnitRegistry
u = UnitRegistry()

class RobotContainer:
    def __init__(self):
        DriverStation.silenceJoystickConnectionWarning(True)


