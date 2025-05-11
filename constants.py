import math

from wpimath.geometry import Rotation2d
from pint import UnitRegistry

u = UnitRegistry()

class ArmConstants:
    MOTOR_ID = 1
    FEEDFORWARD_CONSTANTS = (0,0,0,0) # kS, kG, kV, kA <-- learn what feedforward is
    kP = 0.02

    LENGTH = (14 * u.inch).m_as(u.m)
    MASS = 5.52
    GEAR_RATIO = 125

    MINIMUM_ANGLE = Rotation2d(-math.pi/2)
    MAXIMUM_ANGLE = Rotation2d.fromDegrees(math.pi/3)

    ENCODER_OFFSET = 201.58 # degrees

    LVL_0_ROT = Rotation2d.fromDegrees(0)
    LVL_1_ROT = Rotation2d.fromDegrees(-35)
    LVL_2_ROT = Rotation2d.fromDegrees(-35)
    LVL_3_ROT = Rotation2d.fromDegrees(-35)
    LVL_4_ROT = Rotation2d.fromDegrees(-35)

    TOLERANCE = Rotation2d.fromDegrees(2)