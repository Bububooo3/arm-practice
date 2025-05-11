import commands2
import rev
import wpilib
from commands2.sysid import SysIdRoutine
import wpimath.controller
from wpilib.simulation import SingleJointedArmSim, RoboRioSim
from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor, LinearSystemId
from wpiutil import SendableBuilder
from typing import Optional
from wpilib.sysid import SysIdRoutineLog
from commands2.sysid import SysIdRoutine


from constants import *

class Arm(commands2.Subsystem):
    def __init__(self):
        # Initialization
        super().__init__()
        self.setName("Arm")

        # Real hardware definition
        self.motor = rev.SparkFlex(ArmConstants.MOTOR_ID, rev.SparkBase.MotorType.kBrushless)
        self.abs_encoder = self.motor.getAbsoluteEncoder() # learn what bore encoder is
        self.controller = self.motor.getClosedLoopController() # <-- learn what this means

        self.feedforward = wpimath.controller.ArmFeedforward(*ArmConstants.FEEDFORWARD_CONSTANTS) # <-- physics?!?!

        self.encoder_offset = ArmConstants.ENCODER_OFFSET if wpilib.RobotBase.isReal() else 0

        self.config() # <-- hrm? (defined later bc python is not static like js)

        # Sim setup
        self.abs_encoder_sim = rev.SparkAbsoluteEncoderSim(self.motor)
        gearbox = DCMotor.neoVortex(1) # <-- why's it called gearbox?
        self.motor_sim = rev.SparkFlexSim(self.motor, gearbox)
        moment = SingleJointedArmSim.estimateMOI(ArmConstants.LENGTH, ArmConstants.MASS) # huh?
        plant = LinearSystemId.singleJointedArmSystem(gearbox, moment, ArmConstants.GEAR_RATIO)

        self.arm_sim = SingleJointedArmSim(
            plant,
            gearbox,
            ArmConstants.MASS,
            ArmConstants.MASS,
            ArmConstants.MINIMUM_ANGLE.radians(),
            ArmConstants.MAXIMUM_ANGLE.radians(),
            True,
            0,
        )

        # Sim display
        mech = wpilib.Mechanism2d(5, 5)
        root = mech.getRoot("armPivot", 1, 2.5) # What's this?
        self.arm = root.appendLigament("arm", 3, self.arm_sim.getAngleDegrees())

        # SysId | idk what it's for imma try to comment it out later
        self.sysId_routine = SysIdRoutine(
            SysIdRoutine.Config(0.25, 3),
            SysIdRoutine.Mechanism(
                self.set_voltage,
                self.sysId_log,
                self
            )
        )

        # Reset encoder so it is fine if we turn it off in any rotation
        self.motor.getEncoder().setPosition(self.abs_encoder.getPosition())

        wpilib.SmartDashboard.putData("Arm Mechanism", mech)

        self.goal_rotation: Optional[Rotation2d] = None

    def simulationPeriodic(self) -> None:
        self.arm_sim.setInputVoltage(self.motor_sim.getAppliedOutput() * RoboRioSim.getVInVoltage()) # um, what?

        self.arm_sim.update(0.02) # simulated delta time
        self.motor_sim.iterate(self.arm_sim.getVelocityDps(), RoboRioSim.getVInVoltage(), 0.02) # huh...
        self.abs_encoder_sim.iterate(self.arm_sim.getVelocityDps(), 0.02)

        self.arm.setAngle(self.arm_sim.getAngle())

    def config(self):
        motor_config = rev.SparkBaseConfig()

        motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

        motor_config.absoluteEncoder \
            .positionConversionFactor(360) \
            .velocityConversionFactor(360 / 60) # ???

        motor_config.encoder \
            .positionConversionFactor(360 / ArmConstants.GEAR_RATIO) \
            .velocityConversionFactor(360 / ArmConstants.GEAR_RATIO / 60) # ???

        motor_config.closedLoop \
            .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder) \
            .pid(ArmConstants.kP, 0, 0) \
            .outputRange(-1, 1)

        motor_config.softLimit \
            .forwardSoftLimit(ArmConstants.MAXIMUM_ANGLE.degrees() + self.encoder_offset) \
            .reverseSoftLimit(ArmConstants.MINIMUM_ANGLE.degrees() + self.encoder_offset) \
            .forwardSoftLimitEnabled(True) \
            .reverseSoftLimitEnabled(True)

        self.motor.configure(
            motor_config,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )


    def set_angle(self, angle: Rotation2d):
        self.goal_rotation = angle
        self.controller.setReference(
            angle.degrees() + self.encoder_offset,
            rev.SparkBase.ControlType .kPosition
        )

    def set_duty_cycle(self, output: float): # idk
        self.motor.set(output)

    def at_rotation(self, rotation: Rotation2d) -> bool:
        return abs( rotation.radians() - self.angle().radians() ) < ArmConstants.TOLERANCE.radians()

    def at_goal_rotation(self) -> bool:
        return self.at_rotation(self.goal_rotation) if self.goal_rotation else False

    def set_voltage(self, volts: float): # for what purpose?
        self.controller.setReference(volts, rev.SparkBase.ControlType.kVoltage)

    def angle(self) -> Rotation2d:
        degrees = self.abs_encoder.getPosition() - self.encoder_offset
        return Rotation2d.fromDegrees(degrees)

    def sysId_log(self, log: SysIdRoutineLog):
        log.motor("arm-pivot") \
            .voltage(self.motor.getAppliedOutput() * self.motor.getBusVoltage()) \
            .angularPosition(self.angle().degrees()) \
            .angularVelocity(self.abs_encoder.getVelocity())

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.addStringProperty("Command", self.current_command_name, lambda _: None)
        builder.addDoubleProperty("Angle", lambda: self.angle().degrees(), lambda degrees: self.set_angle(Rotation2d.fromDegrees(degrees)))

    def current_command_name(self) -> str:
        try:
            return self.getCurrentCommand().getName()
        except AttributeError:
            return ""

    def SetAngleCommand(self, angle: Rotation2d):
        return commands2.RunCommand(lambda: self.set_angle(angle), self) \
        .until(self.at_goal_rotation)

    def SysIdQuasistatic(self, direction: SysIdRoutine.Direction):
        return self.sysId_routine.quasistatic(direction)

    def SysIdDynamic(self, direction: SysIdRoutine.Direction): # ???
        return self.sysId_routine.dynamic(direction)



